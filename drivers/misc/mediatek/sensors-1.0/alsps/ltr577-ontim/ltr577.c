/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr577.h"
#include "alsps.h"
#include <linux/rtc.h>

#include <huawei_ts_kit.h>



//#define GN_MTK_BSP_PS_DYNAMIC_CALI
#define RESET_BY_ESD
//#define DELAYED_PS_CALI
//#define DEMO_BOARD
//#define LTR577_DEBUG
//#define SENSOR_DEFAULT_ENABLED
//#define NO_ALS_CTRL_WHEN_PS_ENABLED
//#define REPORT_PS_ONCE_WHEN_ALS_ENABLED
#define NO_PS_CTRL_WHEN_SUSPEND_RESUME

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/
#define LTR577_DEV_NAME			"ltr577_alsps"

/*----------------------------------------------------------------------------*/
#define APS_TAG					"[ALS/PS] "
#define APS_FUN(f)              printk(     	APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)   printk(KERN_ERR  	APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)   printk(KERN_INFO	APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)   printk(KERN_INFO 	APS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr577_i2c_id[] = {{LTR577_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ltr577={ I2C_BOARD_INFO("ltr577_alsps", 0x53)};

static unsigned long long int_top_time;
struct alsps_hw alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
struct platform_device *alspsPltFmDev;
extern bool proximity_probe_ok;
/*----------------------------------------------------------------------------*/
static int ltr577_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr577_i2c_remove(struct i2c_client *client);
static int ltr577_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int ltr577_i2c_suspend(struct device *dev);
static int ltr577_i2c_resume(struct device *dev);

#ifdef RESET_BY_ESD
static int ltr577_init_for_esd(void);
#endif

#ifdef LTR577_DEBUG
static int ltr577_dump_reg(void);
#endif

//static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr577_priv {
	struct alsps_hw  *hw;
	struct i2c_client *client;
	struct work_struct	eint_work;

	/*misc*/
	u16 		als_modulus;
	atomic_t	i2c_retry;
	atomic_t	als_suspend;
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on; 	/*indicates if the debounce is on*/
	atomic_t	als_deb_end;	/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end; 	/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t 	trace;

#ifdef CONFIG_OF
	struct device_node *irq_node;
	int		irq;
#endif

	/*data*/
	u16			als;
	u16 		ps;
	u8			_align;
//	u16			als_level_num;
//	u16			als_value_num;
//	u32			als_level[C_CUST_ALS_LEVEL-1];
//	u32			als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;
	
	atomic_t	als_cmd_val;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_persist_val_high;
	atomic_t	ps_persist_val_low;
	atomic_t	als_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	atomic_t	als_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;
	ulong		enable; 		/*enable mask*/
	ulong		pending_intr;	/*pending interrupt*/
};

//static struct alsps_hw ltr577_hw;

static struct ltr577_priv *ltr577_obj = NULL;
static struct i2c_client *ltr577_i2c_client = NULL;

struct PS_CALI_DATA_STRUCT
{
    int noice;
	int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={-1,0,0,0};
//static int intr_flag_value = 0;
//static int ps_current_state = -1;
static unsigned int current_tp = 0;
static unsigned int current_color_temp=CWF_TEMP;
static unsigned int current_color_temp_first = 0;
static unsigned int    als_level[TP_COUNT][TEMP_COUNT][C_CUST_ALS_LEVEL]; // = {0};
static unsigned int    als_value[C_CUST_ALS_LEVEL] = {0};

static int coeff_duntai[3] = {168,131,199};   //coefficient of TP_duntai influence on als
static int coeff_huiding[3] = {190,162,237};  //coefficient of TP_huiding influence on als
static int coeff_mstar[3] = {177,152,188};    //coefficient of TP_mstar influence on als
static int *coeff_als_for_TP =coeff_duntai;
static int lightSour_dist_thr[2] ={58,15}; //light source distinction threshold for calculating als
extern struct ts_kit_device_data *g_mstar_dev_data;
extern struct ts_kit_device_data *g_focal_dev_data;
extern struct ts_kit_device_data *g_goodix_dev_data;

#define ALSPS_BUF_SIZE    256  // read ps parameters proinfo node length
#define PROINFO_CALI_DATA_OFFSET    210  //ps proinfo start address
#define GSENSOR_CALI_DATA_LEN         6   //ps proinfo read length


static struct file* fd_file = NULL;
static char backup_file_path[ALSPS_BUF_SIZE] = "/dev/block/platform/bootdevice/by-name/proinfo";
static mm_segment_t oldfs;

static unsigned int ps_nvram_none_value = 0;	 /*MMI Calibration none value stored in nvram*/
static unsigned int ps_nvram_40mm_value = 0;  /*MMI Calibration 40mm value stored in nvram*/
static unsigned int ps_nvram_25mm_value = 0;  /*MMI Calibration 25mm value stored in nvram*/
static unsigned int min_proximity =0;
//static unsigned int loop_near_report = 0, loop_far_report = 0;
static unsigned int pwindows_value = 0;//2.5-4
static unsigned int pwave_value = 0;//4-none
static unsigned int threshold_value =0;
static unsigned int ps_threshold_l = 0;
static unsigned int ps_threshold_h = 0;
static unsigned int ps_detection = 1;
#define  MAX_ADC_PROX_VALUE  2047   //max ps raw data
#define  threshold_value_default 10  // ps enable set default  threshold value when read nvram failed
#define  between_threshold_default_40mm 40  // ps set default  threshold  diff value between mRaw and mRaw40 
#define  between_threshold_default_25mm 70 // ps set default  threshold  diff value between mRaw and mRaw25

#define  FAR_THRESHOLD(x)        (min_proximity<(x)?(x): min_proximity)
#define  NEAR_THRESHOLD(x)       ((FAR_THRESHOLD(x) + pwindows_value - 1)>MAX_ADC_PROX_VALUE?MAX_ADC_PROX_VALUE:(FAR_THRESHOLD(x) + pwindows_value - 1))


static DEFINE_MUTEX(ltr577_mutex);
static DEFINE_MUTEX(ltr577_i2c_mutex);

static int ltr577_local_init(void);
static int ltr577_remove(void);
static int ltr577_init_flag =-1; // 0<==>OK -1 <==> fail

static int ps_enabled = 0;
static int als_enabled = 0;

static int irq_enabled = 0;

static struct alsps_init_info ltr577_init_info = {
		.name = "ltr577",
		.init = ltr577_local_init,
		.uninit = ltr577_remove,	
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops ltr577_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr577_i2c_suspend, ltr577_i2c_resume)
};
#endif

static struct i2c_driver ltr577_i2c_driver = {	
	.probe      = ltr577_i2c_probe,
	.remove     = ltr577_i2c_remove,
	.detect     = ltr577_i2c_detect,
	.id_table   = ltr577_i2c_id,
	.driver = {
		.name           = LTR577_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif

#ifdef CONFIG_PM_SLEEP
		.pm = &ltr577_pm_ops,
#endif
	},
};

/*----------------------------------------------------------------------------*/
#ifdef GN_MTK_BSP_PS_DYNAMIC_CALI
static int ltr577_dynamic_calibrate(void);
static int dynamic_calibrate = 0;
#endif
static u16  ps_thd_val_high_set = 0;
static u16  ps_thd_val_low_set = 0;

// The oil algo assumes oil present when there is close contact between TP and human skin like face or finger.
// A very high ps count, 1800 and above, is set as oil close threshold and used to detect the happening of this contact.
// Flag oil_present is set when ps count is above oil close threshold and cleared when ps count is below normal far threshold.
// Based on real time ps count and different threshold values, 4 states normal far/close and oil far/close are determined.
// To avoid wrong state detection, 8 consecutive stable ps data are used for oil far state checking.

// oil algo related parameters
//static int oil_far_cal = 0;					// counter for ps oil far stable calculation
//static int oil_present = 0;					// flag to indicate oil present
#define PS_STABLE_CHECK_COUNT		20		// maximum ps data count to calculate ps is oil far stable
// Note: the following values need to be obtained based on actual project
#define PS_OIL_PERSIST_HIGH			1800	// oil close threshold
#define PS_OIL_PERSIST_LOW_OFFSET	100		// offset value to get oil far threshold from normal far threshold
#define PS_COUNT_MAX_VARIATION		10		// the maximum ps count variation when ps is stable

/*-----------------------------------------------------------------------------*/

static int ltr577_prox_set_noice(int noice);
#include <ontim/ontim_dev_dgb.h>
static char ltr577_prox_version[]="ltr577_mtk_1.0";
static char ltr577_prox_vendor_name[20]="ltr577_alsps";
DEV_ATTR_DECLARE(als_prox)
DEV_ATTR_DEFINE("version",ltr577_prox_version)
DEV_ATTR_DEFINE("vendor",ltr577_prox_vendor_name)
DEV_ATTR_EXEC_DEFINE("set_ps_noice",&ltr577_prox_set_noice)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(als_prox,als_prox,8);
static int current_color_ratio;

/* 
 * #########
 * ## I2C ##
 * #########
 */
static int ltr577_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err;
	u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &beg, },
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data, }
	};

	mutex_lock(&ltr577_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr577_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&ltr577_i2c_mutex);
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	mutex_unlock(&ltr577_i2c_mutex);
	if (err != 2) {
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else {
		err = 0;	/*no error */
	}
	return err;
}

static int ltr577_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];

	err = 0;
	mutex_lock(&ltr577_i2c_mutex);
	if (!client) {
		mutex_unlock(&ltr577_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&ltr577_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++) {
		buf[num++] = data[idx];
	}

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		APS_ERR("send command error!!\n");
		mutex_unlock(&ltr577_i2c_mutex);
		return -EFAULT;
	}
	mutex_unlock(&ltr577_i2c_mutex);
	return err;
}

static int ltr577_prox_set_noice(int noice)
{
    if ((noice > -1 ) &&  (noice < 1600))
    {
        ps_cali.noice = noice;
    }
    else
    {
        ps_cali.noice = -1;
    }
    printk(KERN_ERR "%s: noice = %d; Set noice to %d\n",__func__,noice,ps_cali.noice);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr577_master_recv(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr577_obj->trace);
	int max_try = atomic_read(&ltr577_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr577_i2c_read_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(recv) %d/%d\n", retry - 1, max_try);
		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static int ltr577_master_send(struct i2c_client *client, u16 addr, u8 *buf, int count)
{
	int ret = 0, retry = 0;
	int trc = atomic_read(&ltr577_obj->trace);
	int max_try = atomic_read(&ltr577_obj->i2c_retry);

	while (retry++ < max_try) {
		ret = ltr577_i2c_write_block(client, addr, buf, count);
		if (ret == 0)
			break;
		udelay(100);
	}

	if (unlikely(trc)) {
		if ((retry != 1) && (trc & 0x8000)) {
			APS_LOG("(send) %d/%d\n", retry - 1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}

/*----------------------------------------------------------------------------*/
static void ltr577_power(struct alsps_hw *hw, unsigned int on)
{

}
/********************************************************************/
/*
 * ###############
 * ## PS CONFIG ##
 * ###############
 */
/*
#define BUFFER_SIZE 8
static unsigned int ps_data[BUFFER_SIZE];
static unsigned int data_counter = 0, buffer_full = 0, stable_counter = 0;

static int get_stable_ps(unsigned int ps_count)
{
	int ps_avg = 0;
	int ps_avg_h;
	int ps_avg_l;
	int i;
	long ps_data_sum = 0;

	if (data_counter >= BUFFER_SIZE)
		buffer_full = 1;

	if (buffer_full) {
		data_counter %= BUFFER_SIZE;
	}

	ps_data[data_counter] = ps_count;

	if (buffer_full) {
		for (i = 0; i < BUFFER_SIZE; i++) {
			APS_LOG("LTR577 %s: ps_data[%d] = %d \n", __func__, i, ps_data[i]);
			ps_data_sum += ps_data[i];
		}

		ps_avg = ps_data_sum / BUFFER_SIZE;
		ps_avg_h = ps_avg + PS_COUNT_MAX_VARIATION;
		ps_avg_l = ps_avg - PS_COUNT_MAX_VARIATION;

		for (i = 0; i < BUFFER_SIZE; i++)
		{
			if (ps_data[i] < ps_avg_h && ps_data[i] > ps_avg_l)
				stable_counter++;
			else
				stable_counter = 0;
		}
	}
	else {
		stable_counter = 0;
	}

	data_counter++;

	if (stable_counter >= BUFFER_SIZE - 1) {
		return 1;	// ps data stable
	}
	else {
		return 0;	// ps data not stable
	}
}
*/
static int ltr577_ps_set_thres(void)
{
	int res;
	u8 databuf[2];

	struct i2c_client *client = ltr577_obj->client;
	APS_FUN();

	databuf[0] = LTR577_PS_THRES_LOW_0;
	databuf[1] = (u8)(ps_thd_val_low_set & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = LTR577_PS_THRES_LOW_1;
	databuf[1] = (u8)((ps_thd_val_low_set >> 8) & 0x00FF);

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = LTR577_PS_THRES_UP_0;
	databuf[1] = (u8)(ps_thd_val_high_set & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	databuf[0] = LTR577_PS_THRES_UP_1;
	databuf[1] = (u8)((ps_thd_val_high_set >> 8) & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	res = 0;
	return res;

EXIT_ERR:
	APS_ERR("set thres err: %d\n", res);
	res = LTR577_ERR_I2C;
	return res;
}

/*****************************************
 *** openFile
 *****************************************/
static struct file *openFile(char *path,int flag,int mode)
{
    struct file *fp = NULL;

    fp = filp_open(path, flag, mode);

    if (IS_ERR(fp) || !fp->f_op)
    {
        APS_ERR("Calibration File filp_open return NULL\n");
        return NULL;
    }
    else
    {
        return fp;
    }
}
/*****************************************
 *** seekFile
      whence--- SEEK_END/SEEK_CUR/SEEK_SET
 *****************************************/
static int seekFile(struct file *fp,int offset,int whence)
{
    if (fp->f_op && fp->f_op->llseek)
        return fp->f_op->llseek(fp,(loff_t)offset, whence);
    else
        return -1;
}

/*****************************************
 *** readFile
 *****************************************/

static int readFile(struct file *fp,char *buf,int readlen)
{
		if (fp && fp->f_op)
		return __vfs_read(fp, buf, readlen, &fp->f_pos);
    else
        return -1;
}

/*****************************************
 *** closeFile
 *****************************************/
static int closeFile(struct file *fp)
{
    filp_close(fp,NULL);
    return 0;
}

/*****************************************
 *** initKernelEnv
 *****************************************/
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    printk(KERN_INFO "initKernelEnv\n");
}

/*****************************************
 *** ltr577_read_cali_file
 *****************************************/
static int ltr577_read_cali_file(struct i2c_client *client)
{
	int err = 0;
	char buf[GSENSOR_CALI_DATA_LEN] = {0};
	int mRaw;
	int mRaw40;
	int mRaw25;

	initKernelEnv();
	fd_file = openFile(backup_file_path, O_RDONLY,0);

	 if(fd_file == NULL)
	{
		APS_ERR(" %s:fail to open proinfo file: %s\n", __func__, backup_file_path);
		set_fs(oldfs);
		return -1;
	}
	APS_DBG(" Open proinfo file successfully: %s\n", backup_file_path);
	if (seekFile(fd_file,PROINFO_CALI_DATA_OFFSET,SEEK_SET)<0)
	{
		APS_ERR(" %s:fail to seek proinfo file: %s;\n", __func__, backup_file_path);

		goto open_file_err;
	}
	memset(buf,0,sizeof(buf));
	if ((err = readFile(fd_file, buf, sizeof(buf))) > 0){
		APS_DBG("%s cali_file: buf:%s\n",__func__,buf);
	}else{
		APS_ERR("%s read file error %d\n",__func__,err);
		goto open_file_err;
	}

	closeFile(fd_file);
	set_fs(oldfs);
	mRaw   = (int)(((int)buf[0])<<8 | (0xFF & (int)buf[1]));  //get ps none value from proinfo
	mRaw40 = (int)(((int)buf[2])<<8 | (0xFF & (int)buf[3]));  //get ps 40mm threshold value from proinfo
	mRaw25 = (int)(((int)buf[4])<<8 | (0xFF & (int)buf[5]));   //get ps 25mm threshold value from proinfo

	APS_LOG("%s mRaw=%d, mRaw40=%d, mRaw25=%d\n", __func__, mRaw, mRaw40, mRaw25);

	if(mRaw + 8 < mRaw40 && mRaw40 + 8 < mRaw25){   // difference  8 value between mRaw and mRaw40 and mRaw25
		ps_nvram_none_value = mRaw;
		ps_nvram_40mm_value = mRaw40;
		ps_nvram_25mm_value= mRaw25;

		APS_LOG( "%s none_value=%d,40_value=%d,25_value=%d\n",__func__,ps_nvram_none_value, ps_nvram_40mm_value, ps_nvram_25mm_value);
	}
	else{
		ps_nvram_none_value = threshold_value_default;
		ps_nvram_40mm_value = between_threshold_default_40mm;
		ps_nvram_25mm_value= between_threshold_default_25mm;

		 APS_ERR( "%s none_value=%d,ps_nvram_none_value=%d\n",__func__,mRaw,ps_nvram_none_value);
	}
	return 0;
open_file_err:
	closeFile(fd_file);
	set_fs(oldfs);
	return -1;
}
/********************************************************************/
static int ltr577_ps_read(struct i2c_client *client, u16 *data)
{
	int psdata, ret = 0;
	u8 buf[2];

	ret = ltr577_master_recv(client, LTR577_PS_DATA_0, buf, 0x02);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

//	APS_DBG("ps_rawdata_lo = %d\n", buf[0]);
 //   APS_DBG("ps_rawdata_hi = %d\n", buf[1]);
 
	psdata = ((buf[1] & 0x07) << 8) | (buf[0]);
	*data = psdata;
	APS_DBG("ltr577_ps_read: ps_rawdata = %d\n", psdata);

	final_prox_val = psdata;
	return psdata;
}


/********************************************************************/
static int ltr577_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata;
	int err;

#ifdef RESET_BY_ESD
	u8 regdata_pulse,regdata_interrupt;
#endif

	APS_LOG("ltr577_ps_enable() ...start!\n");

	if (enable != 0 && ps_enabled == 1)
	{
		APS_LOG("PS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && ps_enabled == 0)
	{
		APS_LOG("PS: Already disabled \n");
		return 0;
	}

#ifdef RESET_BY_ESD
	//added this for esd reset 20171206
	err = ltr577_master_recv(client, LTR577_PS_PULSES, &regdata_pulse, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	err = ltr577_master_recv(client, LTR577_INT_CFG, &regdata_interrupt, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	if((regdata_pulse == 0x08) || (regdata_interrupt != 0x01)){
		APS_LOG("als_ps reset by esd. \n");
		err = ltr577_init_for_esd();
	}
#endif

	err = ltr577_master_recv(client, LTR577_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	regdata &= 0xEF;	// Clear reset bit
	
	if (enable != 0) {
		APS_LOG("PS: enable ps only \n");
		regdata |= 0x01;
	}
	else {
		APS_LOG("PS: disable ps only \n");
		regdata &= 0xFE;
	}

	err = ltr577_master_send(client, LTR577_MAIN_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);
	err = ltr577_master_recv(client, LTR577_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}


	if (0 == ltr577_obj->hw->polling_mode_ps && enable != 0)
	{
		ltr577_read_cali_file(ltr577_obj->client);
		pwindows_value = ps_nvram_25mm_value -ps_nvram_40mm_value;//2.5-4
		pwave_value = ps_nvram_40mm_value-ps_nvram_none_value;//4-none
		threshold_value = ps_nvram_none_value;
		min_proximity = ltr577_obj->hw->ps_threshold_low;
		ps_threshold_l = min_proximity-pwave_value-1;
		ps_threshold_h = min_proximity-pwave_value;
		if ((0 == pwindows_value) && (0 == pwave_value) && (0 == threshold_value))
		{
			pwindows_value = ltr577_obj->hw->ps_threshold_high;
			pwave_value = ltr577_obj->hw->ps_threshold_low;
			threshold_value = threshold_value_default;
		}

		ps_thd_val_low_set = ps_threshold_l;
		ps_thd_val_high_set = ps_threshold_h;
		APS_LOG("%s  pwindows_value = %d, pwave_value = %d,threshold_value=%d\n", __func__, pwindows_value, pwave_value,threshold_value);
		APS_LOG("%s  ps_thd_val_low_set = %d, ps_thd_val_high_set = %d\n", __func__, ps_thd_val_low_set, ps_thd_val_high_set);

		ps_enabled = 1;    //ps enalbe
		ltr577_ps_set_thres();
	}
	else if (0 == ltr577_obj->hw->polling_mode_ps && enable == 0)
	{
		ps_enabled = 0;
	}

	if ((irq_enabled == 1) && (enable != 0))
	{
		irq_enabled = 2;
	}

	return err;
}

/********************************************************************/
static int ltr577_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;
	u8 regdata = 0;

	if (enable != 0 && als_enabled == 1)
	{
		APS_LOG("ALS: Already enabled \n");
		return 0;
	}

	if (enable == 0 && als_enabled == 0)
	{
		APS_LOG("ALS: Already disabled \n");
		return 0;
	}
	err = ltr577_master_recv(client, LTR577_MAIN_CTRL, &regdata, 0x01);
	if (err < 0) {
		APS_DBG("i2c error: %d\n", err);
	}

	regdata &= 0xEF;	// Clear reset bit

	if (enable != 0) {
		APS_LOG("ALS(1): enable als only \n");
		regdata |= 0x02;
	}
	else {
		APS_LOG("ALS(1): disable als only \n");
		regdata &= 0xFD;
	}

	err = ltr577_master_send(client, LTR577_MAIN_CTRL, (char *)&regdata, 1);
	if (err < 0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}

	mdelay(WAKEUP_DELAY);

	if (enable != 0)
		als_enabled = 1;
	else
		als_enabled = 0;

	return 0;
}

static int ltr577_als_read(struct i2c_client *client, u16* data)
{
	int alsval = 0, clearval = 0;
	int luxdata_int;
	u8 buf[3];
	int ret;
	int coeff_count = 300;  //coefficient of making als data being integer
	int coeff_ratio = 10;  //coefficient of making ratio increase 10 times
	int ratio;
	struct ts_kit_device_data *tp_dev_data;

 	if(current_color_temp_first == 0)
       {
	       current_color_temp_first = 1;

		if(g_focal_dev_data != NULL){
			tp_dev_data = g_focal_dev_data;
		}
		else if(g_goodix_dev_data != NULL){
			tp_dev_data = g_goodix_dev_data;
		}else if(g_mstar_dev_data != NULL){
			tp_dev_data = g_mstar_dev_data;
		}else{
			APS_ERR("get_touch_info fail!!!use default parameters output als value!\n");
			goto als_default;
		}

		if (tp_dev_data->module_name == NULL)
		{
			APS_ERR("tp_dev_data->module_nam is null ,use default parameters output als value!\n");
			goto als_default;
		}else{
			APS_LOG("Get_touch_info is  %s\n",tp_dev_data->module_name);
			if(strstr(tp_dev_data->module_name,"truly") != NULL)
			{
				APS_LOG("Get_touch_info HTL OK!buffer :%s\n",tp_dev_data->module_name);
				current_tp = TP_duntai;
				coeff_als_for_TP = coeff_duntai;
			}
			else if (strstr(tp_dev_data->module_name,"shenyue") != NULL)
			{
				APS_LOG("Get_touch_info HTL OK!buffer :%s\n",tp_dev_data->module_name);
				current_tp = TP_huiding;
				coeff_als_for_TP = coeff_huiding;
			}
			else if (strstr(tp_dev_data->module_name,"hlt") != NULL)
			{
				APS_LOG("Get_touch_info HTL OK!buffer :%s\n",tp_dev_data->module_name);
				current_tp = TP_mstar;
				coeff_als_for_TP = coeff_mstar;
			}

			APS_LOG(" ltr577 current_tp =%d\n", current_tp);
		}

	}

als_default:
	ret = ltr577_master_recv(client, LTR577_ALS_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	alsval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("alsval_0 = %d,alsval_1=%d,alsval_2=%d,alsval=%d\n", buf[0], buf[1], buf[2], alsval);	

	ret = ltr577_master_recv(client, LTR577_CLEAR_DATA_0, buf, 0x03);
	if (ret < 0) {
		APS_DBG("i2c error: %d\n", ret);
	}

	clearval = (buf[2] * 256 * 256) + (buf[1] * 256) + buf[0];
	APS_DBG("clearval_0 = %d,clearval_1=%d,clearval_2=%d,clearval=%d\n", buf[0], buf[1], buf[2], clearval);

	if (alsval == 0)
	{
		luxdata_int = 0;
		goto out;
	}

	ratio = coeff_ratio*clearval/alsval;
	current_color_ratio = ratio;
	if (ALS_USE_CLEAR_DATA == 1)
	{
		if(ratio > lightSour_dist_thr[0])
		{
		   luxdata_int = (alsval*coeff_als_for_TP[0])/coeff_count;  //  incA
		   current_color_temp = A_TEMP;
		}else if(ratio > lightSour_dist_thr[1])
		{
			luxdata_int = (alsval*coeff_als_for_TP[1])/coeff_count;  // D65
		   current_color_temp = D65_TEMP;
		}else{
		   luxdata_int = (alsval*coeff_als_for_TP[2])/coeff_count;  // CWF
		   current_color_temp = CWF_TEMP;
		}
	}

	APS_DBG(" ltr577_als_read: als_value_lux = %d,ratio=%d,clearval=%d,alsval=%d,current_tp=%d\n", luxdata_int,ratio,clearval,alsval,current_tp);

out:
	*data = luxdata_int;
	final_lux_val = luxdata_int;
	return luxdata_int;
}
/********************************************************************/
static  int ltr577_get_ps_value(struct ltr577_priv *obj, u16 ps)
{
	int ps_data = ps;

/*	u8 buf[2] = {0};
	char databuf[6] = {0};
	int far = 0, near = 0;
	far = (atomic_read(&obj->ps_thd_val_low));
	near = (atomic_read(&obj->ps_thd_val_high));
	ALSPS_LOG("ltr578 far = %d, near = %d\n", far, near);
	ret = ltr577_master_recv(obj->client, LTR577_PS_THRES_LOW_0, buf, 0x02);
	far = ((buf[1]&0x07)<<8)|buf[0];
	ret = ltr577_master_recv(obj->client, LTR577_PS_THRES_UP_0, buf, 0x02);
	near = ((buf[1]&0x07)<<8)|buf[0];
	ALSPS_LOG("ltr578 chect reg far = %d, near = %d\n", far, near);
*/
	if (((ps_data + pwave_value) < min_proximity) && (ps_data >= 0)) {
		min_proximity = ps_data + pwave_value;
		ps_threshold_l = FAR_THRESHOLD(threshold_value);
		ps_threshold_h = NEAR_THRESHOLD(threshold_value);
		ps_thd_val_low_set = ps_threshold_l;
		ps_thd_val_high_set = ps_threshold_h;
		APS_DBG("ltr577 go into schedule min_proximity = %d, shold_l = %d, shold_h = %d\n", min_proximity, ps_threshold_l, ps_threshold_h);
	}
	if (ps_data >= ps_threshold_h) {
		ps_detection = 0;
		ps_thd_val_high_set = MAX_ADC_PROX_VALUE;
		ps_thd_val_low_set = ps_threshold_l;
		APS_DBG("ltr577 ps near!pdata=%d!td_l=%d,td_h=%d \n", ps_data, ps_threshold_l, ps_threshold_h);
		APS_DBG("ltr577 set_l(%d),set_h(%d) \n", ps_thd_val_low_set,ps_thd_val_high_set);
	} else if (ps_data <= ps_threshold_l) {
		ps_detection = 1;
		ps_thd_val_high_set = ps_threshold_h;
		ps_thd_val_low_set = 0;
		APS_DBG("ltr577 set_l = %d , set_h=%d \n", ps_thd_val_low_set,ps_thd_val_high_set);
		APS_DBG("ltr577 ps far! pdata=%d!td_l(%d),td_h(%d)\n", ps_data, ps_threshold_l, ps_threshold_h);
	} else {
		APS_DBG("ltr577 ps between!, ps_data=%d, min_pro= %d, td_l = %d, td_h = %d\n", ps_data , min_proximity, ps_threshold_l, ps_threshold_h);
	}
	return ps_detection;
}

/********************************************************************/
#if 0
static int ltr577_get_als_value(struct ltr577_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	static unsigned int last_lum=0;
    static int last_ratio=0;
	unsigned int lum;

	struct timeval tv = { 0 };
    struct rtc_time tm_android;
    struct timeval tv_android = { 0 };
    do_gettimeofday(&tv);
    tv_android = tv;
    tv_android.tv_sec -= sys_tz.tz_minuteswest * 60;   // time unit conversion
    rtc_time_to_tm(tv_android.tv_sec, &tm_android);

	APS_DBG("als  = %d\n",als);
	for(idx = 0; idx < C_CUST_ALS_LEVEL; idx++)
	{
		APS_DBG( "als_level[current_tp][current_color_temp][idx]  = %d\n",als_level[current_tp][current_color_temp][idx]);
		if(als < als_level[current_tp][current_color_temp][idx])
		{
			break;
		}
	}

	if(idx >= C_CUST_ALS_LEVEL)
	{
		APS_ERR("exceed range\n"); 
		idx = C_CUST_ALS_LEVEL - 1;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG( "ALS: %05d => %05d\n", als, als_level[current_tp][current_color_temp][idx]);
		lum=(als_value[idx]-als_value[idx-1])*(als-als_level[current_tp][current_color_temp][idx-1])/
            (als_level[current_tp][current_color_temp][idx]-als_level[current_tp][current_color_temp][idx-1]);

        lum += als_value[idx-1];

        if( lum != last_lum  || current_color_ratio != last_ratio)
        {
            last_lum =lum;
            last_ratio = current_color_ratio;
            APS_LOG("ALS: %02d:%02d:%02d.%3d. LSY lum=%05d ;ratio=%d;als=%d;current_color_temp=%d;current_tp=%d\n",
                tm_android.tm_hour, tm_android.tm_min, tm_android.tm_sec,(unsigned int)tv_android.tv_usec,
                    lum,current_color_ratio,als,current_color_temp,current_tp);
        }
		return lum;
	}
	else
	{
		APS_DBG("ALS: %05d => %05d (-1)\n", als, als_level[current_tp][current_color_temp][idx]);
		return -1;
	}
}
#endif
/*-------------------------------attribute file for debugging----------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t ltr577_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d)\n", 
		atomic_read(&ltr577_obj->i2c_retry), atomic_read(&ltr577_obj->als_debounce), 
		atomic_read(&ltr577_obj->ps_mask), atomic_read(&ltr577_obj->ps_thd_val), atomic_read(&ltr577_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, thres;
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	
	if(5 == sscanf(buf, "%d %d %d %d %d", &retry, &als_deb, &mask, &thres, &ps_deb))
	{ 
		atomic_set(&ltr577_obj->i2c_retry, retry);
		atomic_set(&ltr577_obj->als_debounce, als_deb);
		atomic_set(&ltr577_obj->ps_mask, mask);
		atomic_set(&ltr577_obj->ps_thd_val, thres);        
		atomic_set(&ltr577_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ltr577_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&ltr577_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %zu\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_als(struct device_driver *ddri, char *buf)
{
	int res;
		
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	res = ltr577_als_read(ltr577_obj->client, &ltr577_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X \n",  res);
	
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_ir(struct device_driver *ddri, char *buf)
{
	int res;
		
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	res = ltr577_als_read(ltr577_obj->client, &ltr577_obj->als);
	return snprintf(buf, PAGE_SIZE, "0x%04X \n", current_color_ratio);
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	res = ltr577_ps_read(ltr577_obj->client, &ltr577_obj->ps);
    return snprintf(buf, PAGE_SIZE, "0x%04X(%d)\n", res, res);
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
			   0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	int ret;
	u8 buffer;

	for(i=0;i<27;i++)
	{
		ret = ltr577_master_recv(ltr577_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return len;
}

#ifdef LTR577_DEBUG
static int ltr577_dump_reg(void)
{
	int i=0;
	int reg[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0d,0x0e,0x0f,
		       0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26};
	int ret;
	u8 buffer;

	for(i=0;i<27;i++)
	{
		ret = ltr577_master_recv(ltr577_obj->client, reg[i], &buffer, 0x01);
		if (ret < 0) {
			APS_DBG("i2c error: %d\n", ret);
		}

		APS_DBG("reg:0x%04X value: 0x%04X\n", reg[i], buffer);
	}
	return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	//****************************
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
	
	if(ltr577_obj->hw)
	{	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr577_obj->hw->i2c_num, ltr577_obj->hw->power_id, ltr577_obj->hw->power_vol);		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr577_obj->als_suspend), atomic_read(&ltr577_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))

static ssize_t ltr577_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}

//	for(idx = 0; idx < ltr577_obj->als_level_num; idx++)
	for(idx = 0; idx < C_CUST_ALS_LEVEL; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", als_level[current_tp][current_color_temp][idx]); //ltr577_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
/*
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr577_obj->als_level, ltr577_obj->hw->als_level, sizeof(ltr577_obj->als_level));
	}
	else if(ltr577_obj->als_level_num != read_int_from_buf(ltr577_obj, buf, count, 
			ltr577_obj->hw->als_level, ltr577_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
*/
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}

//	for(idx = 0; idx < ltr577_obj->als_value_num; idx++)
	for(idx = 0; idx < C_CUST_ALS_LEVEL; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", als_value[idx]);  //  ltr577_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr577_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return 0;
	}
/*
	else if(!strcmp(buf, "def"))
	{
		memcpy(ltr577_obj->als_value, ltr577_obj->hw->als_value, sizeof(ltr577_obj->als_value));
	}
	else if(ltr577_obj->als_value_num != read_int_from_buf(ltr577_obj, buf, count,
			ltr577_obj->hw->als_value, ltr577_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}
*/
	return count;
}
/*---------------------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr577_show_als,		NULL);
static DRIVER_ATTR(ir,     S_IWUSR | S_IRUGO, ltr577_show_ir,		NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr577_show_ps,		NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr577_show_config,	ltr577_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr577_show_alslv,	ltr577_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr577_show_alsval,	ltr577_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, ltr577_show_trace,	ltr577_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr577_show_status,	NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, ltr577_show_send,	ltr577_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, ltr577_show_recv,	ltr577_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr577_show_reg,		NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr577_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ir,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int ltr577_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr577_attr_list)/sizeof(ltr577_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, ltr577_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr577_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int ltr577_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr577_attr_list)/sizeof(ltr577_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr577_attr_list[idx]);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static void ltr577_ps_update(int value)
{

	int res = 0;

	ALSPS_FUN();

	ltr577_ps_set_thres();

	res = ps_report_interrupt_data(value);

	return;
}

/*----------------------------------interrupt functions--------------------------------*/


/*----------------------------------------------------------------------------*/
static void ltr577_eint_work(struct work_struct *work)
{
	struct ltr577_priv *obj = (struct ltr577_priv *)container_of(work, struct ltr577_priv, eint_work);
//	int value = 1;
	u8 regdata;
	unsigned int distance = 0;

	// clear interrupt
	ltr577_master_recv(obj->client, LTR577_MAIN_STATUS, &regdata, 0x01);

	//get raw data
	obj->ps = ltr577_ps_read(obj->client, &obj->ps);
	if (obj->ps < 0)
	{
		goto EXIT_INTR;
	}

	//APS_DBG("ltr577_eint_work: rawdata ps=%d!\n", obj->ps);
	//value = ltr577_get_ps_value(obj, obj->ps);
	//if (value >= 0)
	//	ltr577_ps_update(value);
	distance = ltr577_get_ps_value(obj, obj->ps);
	ltr577_ps_update(distance);

EXIT_INTR:
#ifdef CONFIG_OF
	enable_irq(obj->irq);
	enable_irq_wake(obj->irq);
#endif
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void ltr577_eint_func(void)
{
	struct ltr577_priv *obj = ltr577_obj;
	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}
#ifdef CONFIG_OF
static irqreturn_t ltr577_eint_handler(int irq, void *desc)
{
	APS_DBG("ltr577_eint_handler\n");
	if (irq_enabled == 2)
	{
		disable_irq_nosync(ltr577_obj->irq);
		disable_irq_wake(ltr577_obj->irq);
		ltr577_eint_func();
	}

	return IRQ_HANDLED;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int ltr577_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = { 0, 0 };	

	APS_FUN();

	alspsPltFmDev = get_alsps_platformdev();
//	ltr577_obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,als_ps");
	ltr577_obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	/* gpio setting */
	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}

	/* eint request */
	if (ltr577_obj->irq_node) {

		of_property_read_u32_array(ltr577_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

//	    pinctrl_select_state(pinctrl, pins_cfg);
		ltr577_obj->irq = irq_of_parse_and_map(ltr577_obj->irq_node, 0);
		APS_LOG("ltr577_obj->irq = %d\n", ltr577_obj->irq);
		if (!ltr577_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(ltr577_obj->irq, ltr577_eint_handler, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		//enable_irq(ltr577_obj->irq);
		irq_enabled = 1;
	}
	else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	return 0;
}
/**********************************************************************************************/

/*-------------------------------MISC device related------------------------------------------*/
static int ltr577_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr577_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/************************************************************/
static int ltr577_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/************************************************************/
static long ltr577_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr577_priv *obj = i2c_get_clientdata(client);
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_cali_temp;
	int threshold[2] = {0};
	printk(KERN_ERR"cmd= %08X\n", cmd);
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			APS_ERR("ALSPS_SET_PS_MODE\n");
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr577_ps_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable ps fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_PS, &obj->enable);
			else
				clear_bit(CMC_BIT_PS, &obj->enable);
			break;

		case ALSPS_GET_PS_MODE:
			APS_ERR("ALSPS_GET_PS_MODE\n");
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_ERR("ALSPS_GET_PS_DATA\n");
			obj->ps = ltr577_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}

			dat = ltr577_get_ps_value(obj, obj->ps);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
		APS_DBG("ALSPS_GET_PS raw _DATA\n");
			obj->ps = ltr577_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_SET_ALS_MODE:
			APS_ERR("ALSPS_SET_ALS_MODE\n");
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			err = ltr577_als_enable(obj->client, enable);
			if (err < 0)
			{
				APS_ERR("enable als fail: %d en: %d\n", err, enable);
				goto err_out;
			}
			if (enable)
				set_bit(CMC_BIT_ALS, &obj->enable);
			else
				clear_bit(CMC_BIT_ALS, &obj->enable);
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			APS_ERR("ALSPS_GET_ALS_DATA\n");
			obj->als = ltr577_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat =obj->als ; // ltr577_get_als_value(obj, obj->als);
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_RAW_DATA:
			APS_ERR("ALSPS_GET_ALS_RAW_DATA\n");
			obj->als = ltr577_als_read(obj->client, &obj->als);
			if (obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if (copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			APS_ERR("ALSPS_GET_PS_TEST_RESULT\n");
			obj->ps = ltr577_ps_read(obj->client, &obj->ps);
			if (obj->ps < 0)
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_low))
				dat = 1;
			else
				dat = 0;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_CLR_CALI:
			APS_ERR("ALSPS_IOCTL_CLR_CALI\n");
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
			break;

		case ALSPS_IOCTL_GET_CALI:
			APS_ERR("ALSPS_IOCTL_GET_CALI\n");
			ps_cali_temp = obj->ps_cali ;
			if(copy_to_user(ptr, &ps_cali_temp, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_IOCTL_SET_CALI:
			APS_ERR("ALSPS_IOCTL_SET_CALI\n");
			if(copy_from_user(&ps_cali_temp, ptr, sizeof(ps_cali_temp)))
			{
				err = -EFAULT;
				goto err_out;
			}
			obj->ps_cali = ps_cali_temp;
			 APS_ERR("%s CALI set_cali %d;\n", __func__, obj->ps_cali);
			break;

		case ALSPS_SET_PS_THRESHOLD:
			APS_ERR("%s ALSPS_SET_PS_THRESHOLD ;\n", __func__);
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			APS_ERR("%s ALSPS_SET_PS_THRESHOLD ps_thd_val_high = %d, ps_thd_val_low = %d\n", __func__, atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));
			//ps_thd_val_high_temp = atomic_read(&obj->ps_thd_val_high);
			//ps_thd_val_low_temp = atomic_read(&obj->ps_thd_val_low);

			//ps_cali.close = (threshold[0]+obj->ps_cali);
			//ps_cali.far_away = (threshold[1]+obj->ps_cali);
			//ltr577_ps_set_thres();
			break;

		case ALSPS_GET_PS_THRESHOLD_HIGH:
			APS_ERR("ALSPS_GET_PS_THRESHOLD_HIGH\n");
			 if (ps_cali.noice > -1)
                      {
                            threshold[0] = ps_cali.close - obj->ps_cali;
                      }
                     else
                     {
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
                     }
                     APS_ERR("%s CALI get threshold high: %d;%d;\n", __func__, threshold[0],obj->ps_cali);
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
			APS_ERR("ALSPS_GET_PS_THRESHOLD_LOW\n");
			if (ps_cali.noice > -1)
                     {
                             threshold[0] = ps_cali.far_away;
                     }
                     else
                    {
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
                     }
                   APS_ERR("%s CALI get threshold low: %d;%d;\n", __func__, threshold[0],obj->ps_cali);
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
/*------------------------------------------------------------------------------------------*/

		default:
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;
}
#ifdef CONFIG_COMPAT
static long ltr577_compat_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int err = 0;
	void __user *ptr64 = compat_ptr(arg);
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	printk(KERN_ERR"111111111111cmd= %08X\n", cmd);
	switch (cmd)
	{
		case COMPAT_ALSPS_SET_PS_MODE:
		case COMPAT_ALSPS_GET_PS_MODE:
		case COMPAT_ALSPS_GET_PS_DATA:
		case COMPAT_ALSPS_GET_PS_RAW_DATA:
		case COMPAT_ALSPS_SET_ALS_MODE:
		case COMPAT_ALSPS_GET_ALS_MODE:
		case COMPAT_ALSPS_GET_ALS_DATA:
		case COMPAT_ALSPS_GET_ALS_RAW_DATA:
		case COMPAT_ALSPS_GET_PS_TEST_RESULT:
		case COMPAT_ALSPS_IOCTL_CLR_CALI:
		case COMPAT_ALSPS_IOCTL_GET_CALI:
		case COMPAT_ALSPS_IOCTL_SET_CALI:
		case COMPAT_ALSPS_SET_PS_THRESHOLD:
		case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
		case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
			if(ptr64 == NULL){
				err = -EINVAL;
				break;
			}
			err = file->f_op->unlocked_ioctl(file,cmd,(unsigned long)ptr64);
			if(err <0){
				APS_ERR("cmd %08X is failed !\n",cmd);
			}
			break;
		default:
			APS_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}
	return err;
}
#endif
/********************************************************************/
/*------------------------------misc device related operation functions------------------------------------*/
static struct file_operations ltr577_fops = {
	.owner = THIS_MODULE,
	.open = ltr577_open,
	.release = ltr577_release,
	.unlocked_ioctl = ltr577_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ltr577_compat_ioctl,
#endif
};

static struct miscdevice ltr577_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr577_fops,
};

#ifdef RESET_BY_ESD
static int ltr577_init_for_esd(void)
{
	int res;
	int init_als_gain;
	u8 buf;

	struct i2c_client *client = ltr577_obj->client;

	struct ltr577_priv *obj = ltr577_obj;

	mdelay(10);

	buf = 16; // 16 pulses
	res = ltr577_master_send(client, LTR577_PS_PULSES, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS Pulses error...\n");
		goto EXIT_ERR;
	}

	buf = 0x36;	// 60khz & 100mA
	res = ltr577_master_send(client, LTR577_PS_LED, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS LED error...\n");
		goto EXIT_ERR;
	}

	buf = 0x5C;	// 11bits & 50ms time
	res = ltr577_master_send(client, LTR577_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS time error...\n");
		goto EXIT_ERR;
	}

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps)
	{
		ltr577_ps_set_thres();

		buf = 0x01;
		res = ltr577_master_send(client, LTR577_INT_CFG, (char *)&buf, 1);
		if (res < 0)
		{
			goto EXIT_ERR;
		}

		buf = 0x02;
		res = ltr577_master_send(client, LTR577_INT_PST, (char *)&buf, 1);
		if (res < 0)
		{
			goto EXIT_ERR;
		}
	}
	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE_9;
	als_gainrange = init_als_gain;//Set global variable
	APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

	switch (als_gainrange)
	{
	case ALS_RANGE_1:
		buf = MODE_ALS_Range1;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_3:
		buf = MODE_ALS_Range3;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_6:
		buf = MODE_ALS_Range6;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_9:
		buf = MODE_ALS_Range9;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_18:
		buf = MODE_ALS_Range18;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	default:
		buf = MODE_ALS_Range3;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;
	}

	buf = ALS_RESO_MEAS;	// 18 bit & 100ms measurement rate
	res = ltr577_master_send(client, LTR577_ALS_MEAS_RATE, (char *)&buf, 1);
	APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return 1;
}
#endif


/*--------------------------------------------------------------------------------*/
static int ltr577_init_client(void)
{
	int res;
	int init_als_gain;
	u8 buf;

	struct i2c_client *client = ltr577_obj->client;

	struct ltr577_priv *obj = ltr577_obj;

	mdelay(PON_DELAY);

	/* ===============
	* ** IMPORTANT **
	* ===============
	* Other settings like timing and threshold to be set here, if required.
	* Not set and kept as device default for now.
	*/
	buf = 16; // 16 pulses
	res = ltr577_master_send(client, LTR577_PS_PULSES, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS Pulses error...\n");
		goto EXIT_ERR;
	}

	buf = 0x36;	// 60khz & 100mA
	res = ltr577_master_send(client, LTR577_PS_LED, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS LED error...\n");
		goto EXIT_ERR;
	}

	buf = 0x5C;	// 11bits & 50ms time
	res = ltr577_master_send(client, LTR577_PS_MEAS_RATE, (char *)&buf, 1);
	if (res<0)
	{
		APS_LOG("ltr577_init_client() PS time error...\n");
		goto EXIT_ERR;
	}

	/*for interrup work mode support */
	if (0 == obj->hw->polling_mode_ps)
	{
		ltr577_ps_set_thres();

		buf = 0x01;
		res = ltr577_master_send(client, LTR577_INT_CFG, (char *)&buf, 1);
		if (res < 0)
		{
			goto EXIT_ERR;
		}

		buf = 0x02;
		res = ltr577_master_send(client, LTR577_INT_PST, (char *)&buf, 1);
		if (res < 0)
		{
			goto EXIT_ERR;
		}
	}

	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE_9;
	als_gainrange = init_als_gain;//Set global variable
	APS_ERR("ALS sensor gainrange %d!\n", init_als_gain);

	switch (als_gainrange)
	{
	case ALS_RANGE_1:
		buf = MODE_ALS_Range1;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_3:
		buf = MODE_ALS_Range3;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_6:
		buf = MODE_ALS_Range6;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_9:
		buf = MODE_ALS_Range9;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	case ALS_RANGE_18:
		buf = MODE_ALS_Range18;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;

	default:
		buf = MODE_ALS_Range3;
		res = ltr577_master_send(client, LTR577_ALS_GAIN, (char *)&buf, 1);
		break;
	}

	buf = ALS_RESO_MEAS;	// 18 bit & 100ms measurement rate
	res = ltr577_master_send(client, LTR577_ALS_MEAS_RATE, (char *)&buf, 1);
	APS_ERR("ALS sensor resolution & measurement rate: %d!\n", ALS_RESO_MEAS);

	if ((res = ltr577_setup_eint(client)) != 0)
	{
		APS_ERR("setup eint: %d\n", res);
		goto EXIT_ERR;
	}

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return 1;
}
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr577_obj als enable value = %d\n", en);

	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return -1;
	}

	res = ltr577_als_enable(ltr577_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr577_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr577_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr577_obj->enable);
	mutex_unlock(&ltr577_mutex);

	return 0;
}

static int als_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int als_get_data(int* value, int* status)
{
	int err = 0;
	
	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return -1;
	}

	ltr577_obj->als = ltr577_als_read(ltr577_obj->client, &ltr577_obj->als);
	if (ltr577_obj->als < 0)
		err = -1;
	else {
		*value = ltr577_obj->als ; //*value = ltr577_get_als_value(ltr577_obj, ltr577_obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("ltr577_obj ps enable value = %d\n", en);

	if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return -1;
	}

	res = ltr577_ps_enable(ltr577_obj->client, en);
	if (res < 0) {
		APS_ERR("ps_enable_nodata is failed!!\n");
		return -1;
	}

	mutex_lock(&ltr577_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr577_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr577_obj->enable);
	mutex_unlock(&ltr577_mutex);

	return 0;
}

static int ps_set_delay(u64 ns)
{
	// Do nothing
	return 0;
}

static int ps_get_data(int* value, int* status)
{
    int err = 0;

    if(!ltr577_obj)
	{
		APS_ERR("ltr577_obj is null!!\n");
		return -1;
	}
	ltr577_obj->ps = ltr577_ps_read(ltr577_obj->client, &ltr577_obj->ps);
	if (ltr577_obj->ps < 0)
		err = -1;
	else {
		*value = ltr577_get_ps_value(ltr577_obj, ltr577_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	return err;
}

static int ltr_als_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    return 0;
}
static int ltr_als_flush(void)
{
    int err = 0;

    err = als_flush_report();
    pr_err("add for modify: flush complete \n");
    return err;
}

static int ltr_ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    return 0;
}
static int ltr_ps_flush(void)
{
    int err = 0;

    err = ps_flush_report();
    pr_err("add for modify: flush complete \n");
    return err;
}

/*-----------------------------------------------------------------------------------*/

/*-----------------------------------i2c operations----------------------------------*/
static int ltr577_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr577_priv *obj = NULL;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	int err = 0;

	if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
	{
		return -EIO;
	}
	APS_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

#if 0
	err = get_alsps_dts_func(client->dev.of_node, obj->hw);
    if (err < 0)
    {
        printk(KERN_ERR "%s:get dts info fail\n",__func__);
        goto exit_init_failed;
    }
    ltr577_hw = obj->hw;

    i2c_register_board_info(obj->hw->i2c_num, &i2c_ltr577, 1);
#endif

	ltr577_obj = obj;

	obj->hw = hw;
	INIT_WORK(&obj->eint_work, ltr577_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);

	/*-----------------------------value need to be confirmed-----------------------------------------*/
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	atomic_set(&obj->als_thd_val_high,  obj->hw->als_threshold_high);
	atomic_set(&obj->als_thd_val_low,  obj->hw->als_threshold_low);
	//ps_thd_val_high_temp = atomic_read(&obj->ps_thd_val_high);
	//ps_thd_val_low_temp = atomic_read(&obj->ps_thd_val_low);


	obj->enable = 0;
	obj->pending_intr = 0;
	obj->ps_cali = 0;
//	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
//	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100	
	/*-----------------------------value need to be confirmed-----------------------------------------*/
/*
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
*/
	BUG_ON(sizeof(als_level) != sizeof(obj->hw->als_level));
    memcpy(als_level, obj->hw->als_level, sizeof(als_level));
    BUG_ON(sizeof(als_value) != sizeof(obj->hw->als_value));
    memcpy(als_value, obj->hw->als_value, sizeof(als_value));
	atomic_set(&obj->i2c_retry, 3);

	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr577_init_client() start...!\n");
	ltr577_i2c_client = client;
	err = ltr577_init_client();
	if(err)
	{
		goto exit_init_failed;
	}
	APS_LOG("ltr577_init_client() OK!\n");
	err = misc_register(&ltr577_device);
	if(err)
	{
		APS_ERR("ltr577_device register failed\n");
		goto exit_misc_device_register_failed;
	}

    als_ctl.is_use_common_factory =false;
	ps_ctl.is_use_common_factory = false;

	/*------------------------ltr577 attribute file for debug--------------------------------------*/
	err = ltr577_create_attr(&(ltr577_init_info.platform_diver_addr->driver));
	//err = ltr577_create_attr(&(ltr577_i2c_driver.driver));
	if(err)
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	/*------------------------ltr577 attribute file for debug--------------------------------------*/

	min_proximity = obj->hw->ps_threshold_low;
	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
//	als_ctl.is_support_batch = false;

	als_ctl.batch = ltr_als_batch;
    als_ctl.flush = ltr_als_flush;

#ifdef CUSTOM_KERNEL_SENSORHUB
    als_ctl.is_support_batch = obj->hw->is_batch_supported_als;
#else
    als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
//	ps_ctl.is_support_batch = false;

	ps_ctl.batch = ltr_ps_batch;
    ps_ctl.flush = ltr_ps_flush;
#ifdef CUSTOM_KERNEL_SENSORHUB
    ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;
#else
    ps_ctl.is_support_batch = false;
#endif
	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

#if 0
	err = batch_register_support_info(ID_LIGHT,als_ctl.is_support_batch, 1, 0);
	if(err)
	{
		APS_ERR("register light batch support err = %d\n", err);
	}
	err = batch_register_support_info(ID_PROXIMITY,ps_ctl.is_support_batch, 1, 0);
	if(err)
	{
		APS_ERR("register proximity batch support err = %d\n", err);
	}
#endif
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	ltr577_init_flag =0;
	APS_LOG("%s: OK\n", __func__);
	proximity_probe_ok = 1;
	return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
exit_misc_device_register_failed:
	misc_deregister(&ltr577_device);
exit_init_failed:
	kfree(obj);
exit:
	ltr577_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr577_init_flag =-1;
	return err;
}

static int ltr577_i2c_remove(struct i2c_client *client)
{
	int err;

	//err = ltr577_delete_attr(&(ltr577_init_info.platform_diver_addr->driver));
	err = ltr577_delete_attr(&(ltr577_i2c_driver.driver));
	if(err)
	{
		APS_ERR("ltr577_delete_attr fail: %d\n", err);
	}

	misc_deregister(&ltr577_device);

//	err = misc_deregister(&ltr577_device);
//	if(err)
//	{
//		APS_ERR("misc_deregister fail: %d\n", err);
//	}
	ltr577_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static int ltr577_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, LTR577_DEV_NAME);
	return 0;
}

static int ltr577_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr577_priv *obj = i2c_get_clientdata(client);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	atomic_set(&obj->als_suspend, 1);
	err = ltr577_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als: %d\n", err);
		return err;
	}
#ifndef NO_PS_CTRL_WHEN_SUSPEND_RESUME
	atomic_set(&obj->ps_suspend, 1);
	err = ltr577_ps_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable ps:  %d\n", err);
		return err;
	}

	ltr577_power(obj->hw, 0);
#endif		
	return 0;
}

static int ltr577_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr577_priv *obj = i2c_get_clientdata(client);
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
#ifndef NO_PS_CTRL_WHEN_SUSPEND_RESUME
	ltr577_power(obj->hw, 1);

	atomic_set(&obj->ps_suspend, 0);
	if (test_bit(CMC_BIT_PS, &obj->enable))
	{
		err = ltr577_ps_enable(obj->client, 1);
		if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);
		}
	}
#endif
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		err = ltr577_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);
		}
	}

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int ltr577_remove(void)
{
	APS_FUN();

	ltr577_power(hw, 0);
	i2c_del_driver(&ltr577_i2c_driver);
	ltr577_init_flag = -1;

	return 0;
}
/*----------------------------------------------------------------------------*/
static int  ltr577_local_init(void)
{
	APS_FUN();

	ltr577_power(hw, 1);
	
	if(i2c_add_driver(&ltr577_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == ltr577_init_flag)
	{
	   return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init ltr577_init(void)
{
#if 1
	const char *name = "mediatek,ltr577";

    struct device_node *node = of_find_compatible_node(NULL, NULL, name);

	APS_FUN();

 //   hw = get_alsps_dts_func(name, hw);
//	if (!hw)
//		APS_ERR("get dts info fail\n");

    if (get_alsps_dts_func(node, hw))
    {
        printk(KERN_ERR "%s:get dts info fail\n",__func__);
        return -1;
    }
 //   hw = ltr577_hw;

    i2c_register_board_info(hw->i2c_num, &i2c_ltr577, 1);
#endif

	alsps_driver_add(&ltr577_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr577_exit(void)
{
	APS_FUN();
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
module_init(ltr577_init);
module_exit(ltr577_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Liteon");
MODULE_DESCRIPTION("LTR-577ALSPS Driver");
MODULE_LICENSE("GPL");

