
#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_type.h>
#include "fan54005.h"
#include "mtk_charger_intf.h"
#include "mtk_switch_charging.h"
#include <ontim/ontim_dev_dgb.h>
static char charge_ic_vendor_name[50]="FAN54005";
static char g_Second_charger_ic_name[50]="HL7005D";
static char g_third_charger_ic_name[50]="BQ24157";
DEV_ATTR_DECLARE(charge_ic)
DEV_ATTR_DEFINE("vendor",charge_ic_vendor_name)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(charge_ic,charge_ic,8);
#define CHARGING_CURRENT_MAXI 800000
#ifndef BATTERY_BOOL
#define BATTERY_BOOL
typedef enum {
	KAL_FALSE = 0,
	KAL_TRUE  = 1,
} kal_bool;
#endif
#ifdef CONFIG_HUAWEI_CHARGER
#define REGISTER_BUF_SIZE   (100)
#endif/*CONFIG_HUAWEI_CHARGER*/
/* ============================================================ // */
/* Define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_FAIL	1
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


const u32 FAN54005_VBAT_CV_VTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000
};

const u32 FAN54005_CS_VTH[] = {
	550000, 650000, 750000, 850000, 1050000, 1150000, 1350000, 1450000
};

const u32 FAN54005_INPUT_CS_VTH[] = {
	100000, 500000, 800000, 3200001
};
#if 0
const u32 FAN54005_VCDT_HV_VTH_FOR_OLD[] = {
	BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V, BATTERY_VOLT_04_300000_V,
	    BATTERY_VOLT_04_350000_V,
	BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V, BATTERY_VOLT_04_500000_V,
	    BATTERY_VOLT_04_550000_V,
	BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V, BATTERY_VOLT_06_500000_V,
	    BATTERY_VOLT_07_000000_V,
	BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V, BATTERY_VOLT_09_500000_V,
	    BATTERY_VOLT_10_500000_V
};

const u32 FAN54005_VCDT_HV_VTH[] = {
	BATTERY_VOLT_03_860000_V, BATTERY_VOLT_03_900000_V, BATTERY_VOLT_03_950000_V,
	    BATTERY_VOLT_04_000000_V,
	BATTERY_VOLT_04_040000_V, BATTERY_VOLT_04_087500_V, BATTERY_VOLT_04_137500_V,
        BATTERY_VOLT_04_180000_V,
    BATTERY_VOLT_04_225000_V, BATTERY_VOLT_05_450000_V, BATTERY_VOLT_06_000000_V,
        BATTERY_VOLT_06_400000_V,
    BATTERY_VOLT_07_000000_V, BATTERY_VOLT_08_000000_V, BATTERY_VOLT_08_500000_V,
        BATTERY_VOLT_09_500000_V
};
#endif

#define fan54005_SLAVE_ADDR_WRITE_ERROR   0xDA
#define fan54005_SLAVE_ADDR_READ    0xD5
#define fan54005_SLAVE_ADDR_WRITE   0xD4
#define FAN54005_SAFETY_VALUE	0x7b
static struct i2c_client *new_client;
static const struct i2c_device_id fan54005_i2c_id[] = { {"fan54005", 0}, {} };

static bool is_hl7005_flag = false;
#define CV_VOLTAGE_4_420 0x2e
#define CV_VOLTAGE_4_400 0x2d
#define CV_VOLTAGE_3_540 0x02
kal_bool chargin_hw_init_done = KAL_FALSE;
//extern kal_bool chargin_hw_init_done;
static int fan54005_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#define RECHARGE_SOC 98
#define FULL_SOC 100

#ifdef CONFIG_OF
static const struct of_device_id fan54005_of_match[] = {
	{.compatible = "fan54005",},
	{},
};

MODULE_DEVICE_TABLE(of, fan54005_of_match);
#endif
struct fan54005_info {
	struct charger_device *chg_dev;
	struct device *dev;
	struct charger_properties chg_props;
	const char *chg_dev_name;
	const char *eint_name;
};


static struct i2c_driver fan54005_driver = {
	.driver = {
		   .name = "fan54005",
#ifdef CONFIG_OF
		   .of_match_table = fan54005_of_match,
#endif
	},
	.probe = fan54005_driver_probe,
	.id_table = fan54005_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char fan54005_reg[fan54005_REG_NUM] = { 0 };

static DEFINE_MUTEX(fan54005_i2c_access);


int g_fan54005_hw_exist = -1;
static unsigned int g_input_current;

/**********************************************************
  *
  *   [I2C Function For Read/Write fan54005]
  *
  *********************************************************/
#ifdef CONFIG_MTK_I2C_EXTENSION
unsigned int fan54005_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&fan54005_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	new_client->ext_flag =
		((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;
		mutex_unlock(&fan54005_i2c_access);

		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;
	mutex_unlock(&fan54005_i2c_access);

	return 1;
}

unsigned int fan54005_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&fan54005_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&fan54005_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&fan54005_i2c_access);
	return 1;
}
#else
unsigned int fan54005_read_byte(unsigned char cmd, unsigned char *returnData)
{
	unsigned char xfers = 2;
	int ret, retries = 1;

	mutex_lock(&fan54005_i2c_access);

	do {
		struct i2c_msg msgs[2] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1,
				.buf = &cmd,
			},
			{

				.addr = new_client->addr,
				.flags = I2C_M_RD,
				.len = 1,
				.buf = returnData,
			}
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
		if (ret != 2)
			chr_err("i2c transfer error\n");
	} while (ret != xfers && --retries);

	mutex_unlock(&fan54005_i2c_access);

	return ret == xfers ? 1 : -1;
}

unsigned int fan54005_write_byte(unsigned char cmd, unsigned char writeData)
{
	unsigned char xfers = 1;
	int ret, retries = 1;
	unsigned char buf[8];

	mutex_lock(&fan54005_i2c_access);

	buf[0] = cmd;
	memcpy(&buf[1], &writeData, 1);

	do {
		struct i2c_msg msgs[1] = {
			{
				.addr = new_client->addr,
				.flags = 0,
				.len = 1 + 1,
				.buf = buf,
			},
		};

		/*
		 * Avoid sending the segment addr to not upset non-compliant
		 * DDC monitors.
		 */
		ret = i2c_transfer(new_client->adapter, msgs, xfers);

		if (ret == -ENXIO) {
			pr_info("skipping non-existent adapter %s\n", new_client->adapter->name);
			break;
		}
		if (ret != 1)
			chr_err("i2c transfer error\n");
	} while (ret != xfers && --retries);

	mutex_unlock(&fan54005_i2c_access);

	return ret == xfers ? 1 : -1;
}
#endif

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int fan54005_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan54005_reg = 0;
	int ret = 0;

	ret = fan54005_read_byte(RegNum, &fan54005_reg);

	pr_debug("[fan54005_read_interface] Reg[%x]=0x%x\n", RegNum, fan54005_reg);

	fan54005_reg &= (MASK << SHIFT);
	*val = (fan54005_reg >> SHIFT);

	//battery_log(BAT_LOG_CRTI, "[fan54005_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int fan54005_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char fan54005_reg = 0;
	int ret = 0;

	ret = fan54005_read_byte(RegNum, &fan54005_reg);
	pr_debug( "[fan54005_config_interface] Reg[%x]=0x%x\n", RegNum, fan54005_reg);

	fan54005_reg &= ~(MASK << SHIFT);
	fan54005_reg |= (val << SHIFT);

	if (RegNum == fan54005_CON4 && val == 1 && MASK == CON4_RESET_MASK
	    && SHIFT == CON4_RESET_SHIFT) {
		/* RESET bit */
	} else if (RegNum == fan54005_CON4) {
		fan54005_reg &= ~0x80;	/* RESET bit read returs 1, so clear it */
        //battery_log(BAT_LOG_CRTI, "[fan54005_config_interface] fan54005_reg:0x%x; line:%d\n",fan54005_reg,__LINE__);

	}

	ret = fan54005_write_byte(RegNum, fan54005_reg);
	pr_debug( "[fan54005_config_interface] write Reg[%x]=0x%x\n", RegNum,
		    fan54005_reg);

	return ret;
}

void fan54005_dump_register(void)
{
	int i = 0;

    printk("[fan54005] ");
	for (i = 0; i < fan54005_REG_NUM; i++) {
		fan54005_read_byte(i, &fan54005_reg[i]);
        printk("[0x%x]=0x%x ", i, fan54005_reg[i]);
	}
    printk("\n");
}

void charge_ic_dump_register(char *buf)
{
	int i = 0;
    unsigned int used_length = 0;

	for (i = 0; i < fan54005_REG_NUM; i++) {
		fan54005_read_byte(i, &fan54005_reg[i]);
        snprintf(buf + used_length, REGISTER_BUF_SIZE - used_length, "0x%x ", fan54005_reg[i]);
        used_length = strlen(buf);
	}
}


/* write one register directly */
unsigned int fan54005_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	int ret = 0;

	ret = fan54005_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0 */

void fan54005_set_tmr_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_TMR_RST_MASK),
				       (unsigned char) (CON0_TMR_RST_SHIFT)
	    );
}

unsigned int fan54005_get_otg_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_OTG_MASK),
				     (unsigned char) (CON0_OTG_SHIFT)
	    );
	return val;
}

void fan54005_set_en_stat(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_STAT_MASK),
				       (unsigned char) (CON0_EN_STAT_SHIFT)
	    );
}

unsigned int fan54005_get_chip_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_STAT_MASK),
				     (unsigned char) (CON0_STAT_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_boost_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_BOOST_MASK),
				     (unsigned char) (CON0_BOOST_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_fault_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON0),
				     (&val), (unsigned char) (CON0_FAULT_MASK),
				     (unsigned char) (CON0_FAULT_SHIFT)
	    );
	return val;
}

/* CON1 */

void fan54005_set_input_charging_current(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LIN_LIMIT_MASK),
				       (unsigned char) (CON1_LIN_LIMIT_SHIFT)
	    );
}

void fan54005_set_v_low(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_LOW_V_MASK),
				       (unsigned char) (CON1_LOW_V_SHIFT)
	    );
}

void fan54005_set_te(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_TE_MASK),
				       (unsigned char) (CON1_TE_SHIFT)
	    );
}

void fan54005_set_ce(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CE_MASK),
				       (unsigned char) (CON1_CE_SHIFT)
	    );
}

int fan54005_set_hz_mode(struct charger_device *chg_dev, int val)
{
	unsigned int ret = 0;
	if (!chg_dev)
		return -1;
	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_HZ_MODE_MASK),
				       (unsigned char) (CON1_HZ_MODE_SHIFT)
	    );
	return ret;
}

void fan54005_set_opa_mode(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OPA_MODE_MASK),
				       (unsigned char) (CON1_OPA_MODE_SHIFT)
	    );
}

/* CON2 */

void fan54005_set_oreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OREG_MASK),
				       (unsigned char) (CON2_OREG_SHIFT)
	    );
}

void fan54005_set_otg_pl(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_PL_MASK),
				       (unsigned char) (CON2_OTG_PL_SHIFT)
	    );
}

void fan54005_set_otg_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_OTG_EN_MASK),
				       (unsigned char) (CON2_OTG_EN_SHIFT)
	    );
}

/* CON3 */

unsigned int fan54005_get_vender_code(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_VENDER_CODE_MASK),
				     (unsigned char) (CON3_VENDER_CODE_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_pn(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_PIN_MASK),
				     (unsigned char) (CON3_PIN_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_revision(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON3),
				     (&val), (unsigned char) (CON3_REVISION_MASK),
				     (unsigned char) (CON3_REVISION_SHIFT)
	    );
	return val;
}

/* CON4 */

void fan54005_set_reset(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_RESET_MASK),
				       (unsigned char) (CON4_RESET_SHIFT)
	    );
}

void fan54005_set_iocharge(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_CHR_MASK),
				       (unsigned char) (CON4_I_CHR_SHIFT)
	    );
}

void fan54005_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_I_TERM_MASK),
				       (unsigned char) (CON4_I_TERM_SHIFT)
	    );
}

/* CON5 */

void fan54005_set_dis_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_DIS_VREG_MASK),
				       (unsigned char) (CON5_DIS_VREG_SHIFT)
	    );
}

void fan54005_set_io_level(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_IO_LEVEL_MASK),
				       (unsigned char) (CON5_IO_LEVEL_SHIFT)
	    );
}

unsigned int fan54005_get_sp_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON5),
				     (&val), (unsigned char) (CON5_SP_STATUS_MASK),
				     (unsigned char) (CON5_SP_STATUS_SHIFT)
	    );
	return val;
}

unsigned int fan54005_get_en_level(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface((unsigned char) (fan54005_CON5),
				     (&val), (unsigned char) (CON5_EN_LEVEL_MASK),
				     (unsigned char) (CON5_EN_LEVEL_SHIFT)
	    );
	return val;
}
int fan54005_set_vsp(unsigned int val)
{
    int ret = 0;
    ret = fan54005_config_interface((unsigned char) (fan54005_CON5),
                        (unsigned char) (val),
                        (unsigned char) (CON5_VSP_MASK),
                        (unsigned char) (CON5_VSP_SHIFT)
                        );
    return ret;
}
/* CON6 */

void fan54005_set_i_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_ISAFE_MASK),
				       (unsigned char) (CON6_ISAFE_SHIFT)
	    );
}

void fan54005_set_v_safe(unsigned int val)
{
	unsigned int ret = 0;

	ret = fan54005_config_interface((unsigned char) (fan54005_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VSAFE_MASK),
				       (unsigned char) (CON6_VSAFE_SHIFT)
	    );
}

static u32 charging_value_to_parameter(const u32 *parameter, const u32 array_size, const u32 val)
{
	if (val < array_size)
		return parameter[val];
	chr_err( "Can't find the parameter \r\n");
	return parameter[0];
}

static u32 charging_parameter_to_value(const u32 *parameter, const u32 array_size, const u32 val)
{
	u32 i;

	for (i = 0; i < array_size; i++)
		if (val == *(parameter + i))
			return i;

	chr_err( "NO register value match \r\n");

	return 0;
}


static u32 bmt_find_closest_level(const u32 *pList, u32 number, u32 level)
{
	u32 i;
	u32 max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = KAL_TRUE;
	else
		max_value_in_last_element = KAL_FALSE;

	if (max_value_in_last_element == KAL_TRUE) {
		for (i = (number - 1); i != 0; i--)	/* max value in the last element */
			if (pList[i] <= level)
				return pList[i];

		chr_err( "Can't find closest level, small value first \r\n");
		return pList[0];
		/* return CHARGE_CURRENT_0_00_MA; */
	} else {
		for (i = 0; i < number; i++)	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];

		chr_err( "Can't find closest level, large value first \r\n");
		return pList[number - 1];
		/* return CHARGE_CURRENT_0_00_MA; */
	}
}
static char *g_Second_charger_ic=NULL;

char * ontim_get_g_Second_charger_ic(void)
{
	return g_Second_charger_ic;
}

static int charging_hw_init(struct charger_device *chg_dev)
{
	u32 status = STATUS_OK;
	static kal_bool charging_init_flag = KAL_FALSE;
	unsigned char fan54005_CON6_BYTE = 0;
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
	mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
#endif
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // ISAFE = 1450mA, VSAFE = 4.42V
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // set safety reg twice according to ds. fixme
	fan54005_read_byte(fan54005_CON6, &fan54005_CON6_BYTE);
	if (fan54005_CON6_BYTE != FAN54005_SAFETY_VALUE)
		chr_err("%s %d fan54005 set safety register failed!\n", __func__, __LINE__);

	fan54005_reg_config_interface(0x00,0xC0); //kick chip watch dog
	//fan54005_reg_config_interface(0x05,0x04);
	if ( !charging_init_flag ) {
		fan54005_reg_config_interface(0x04,0x73); //200mA
		fan54005_reg_config_interface(0x01,0x78);	//TE=1, CE=0, HZ_MODE=0, OPA_MODE=0
		charging_init_flag = KAL_TRUE;
	}
	fan54005_dump_register();
	return status;
}

static int fan54005_set_dpm(struct charger_device *chg_dev, unsigned char dpmbyte)
{
	u32 status = STATUS_OK;
	fan54005_set_vsp(dpmbyte);
	return status;
}

static int charging_dump_register(struct charger_device *chg_dev)
{
	u32 status = STATUS_OK;

	fan54005_dump_register();

	return status;
}


static int charging_enable(struct charger_device *chg_dev, bool en)
{
	u32 status = STATUS_OK;
	u32 enable = en;
	if (!chg_dev)
		return -1;
	if (KAL_TRUE == enable) {
		fan54005_set_ce(0);
		fan54005_set_hz_mode(chg_dev, 0);
		fan54005_set_opa_mode(0);
		
	} else {
#if defined(CONFIG_USB_MTK_HDRC_HCD)
		if (mt_usb_is_device())
#endif

			fan54005_set_ce(1);
	}

	return status;
}


static int charging_set_cv_voltage(struct charger_device *chg_dev, u32 cv)
{
	u32 status = STATUS_OK;
	u16 register_value;
	u32 set_cv;
	u32 array_size;
	array_size = GETARRAYNUM(FAN54005_VBAT_CV_VTH);
	set_cv = bmt_find_closest_level(FAN54005_VBAT_CV_VTH, array_size, cv);
	register_value = charging_parameter_to_value(FAN54005_VBAT_CV_VTH, array_size, set_cv);
	if (is_hl7005_flag)
	{
		if (register_value > CV_VOLTAGE_4_420)
			register_value = CV_VOLTAGE_4_400;
		else if (register_value < CV_VOLTAGE_3_540)
			return status;
		else
			register_value -= 0x02;//cv down to 2 stage below due to hl7005 hardware mistake
	}
	chr_err( " set CV to %d,  register_value =0x%x\n", cv, register_value);
	fan54005_set_oreg(register_value);

	return status;
}


static int charging_get_current(struct charger_device *chg_dev, u32 *ichg)
{
	u32 status = STATUS_OK;
	u32 array_size;
	u8 reg_value;

	/* Get current level */
	array_size = GETARRAYNUM(FAN54005_CS_VTH);
    fan54005_read_interface(0x4, &reg_value, 0x7, 0x4);	/* IINLIM */
	*ichg = charging_value_to_parameter(FAN54005_CS_VTH, array_size, reg_value);

	return status;
}



static int charging_set_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;
	//battery_log(BAT_LOG_CRTI, "%s: set current to %d\n",__func__,current_value);
	if (current_value <= 35000) {
		fan54005_set_io_level(1);
	} else {
		fan54005_set_io_level(0);
		array_size = GETARRAYNUM(FAN54005_CS_VTH);
		set_chr_current = bmt_find_closest_level(FAN54005_CS_VTH, array_size, current_value);

		//battery_log(BAT_LOG_CRTI, "charging_set_current  set_chr_current=%d\n", set_chr_current);

		register_value = charging_parameter_to_value(FAN54005_CS_VTH, array_size, set_chr_current);
		chr_err( "charging_set_current  register_value=%d\n", register_value);
		fan54005_set_iocharge(register_value);
	}
	return status;
}

static int charging_get_input_current(struct charger_device *chg_dev, u32 *aicr)
{
	int ret = 0;

	*aicr = g_input_current;

	return ret;
}

static int charging_set_input_current(struct charger_device *chg_dev, u32 current_value)
{
	u32 status = STATUS_OK;
	u32 set_chr_current;
	u32 array_size;
	u32 register_value;

	//battery_log(BAT_LOG_CRTI, "%s: set input current  to %d\n",__func__,*(u32 *) data);
	if (current_value > CHARGING_CURRENT_MAXI) {
		register_value = 0x3;
	chr_err( "charging_set_input_current  register_value=%d\n", register_value);
	} else {
		array_size = GETARRAYNUM(FAN54005_INPUT_CS_VTH);
		set_chr_current = bmt_find_closest_level(FAN54005_INPUT_CS_VTH, array_size, current_value);
	//battery_log(BAT_LOG_CRTI, "charging_set_input_current  set_chr_current=%d\n", set_chr_current);
		register_value =
		    charging_parameter_to_value(FAN54005_INPUT_CS_VTH, array_size, set_chr_current);
	//battery_log(BAT_LOG_CRTI, "charging_set_input_current  register_value=%d\n", register_value);
	}
	//if(ontim_get_g_Second_charger_ic()==NULL)
		fan54005_set_input_charging_current(register_value);

	return status;
}



static int charging_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	u32 status = STATUS_OK;
	u32 ret_val;

	signed int soc = battery_get_bat_soc();
       signed int ui_soc = battery_get_bat_uisoc();
	struct charger_manager *info = (struct charger_manager *)chg_dev->driver_data;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	ret_val = fan54005_get_chip_status();

	if (ret_val == 0x2)
	{
		if (swchgalg->state == CHR_CC)
			*is_done = KAL_TRUE;
		 else  if(swchgalg->state == CHR_BATFULL)
		 {
			*is_done = KAL_TRUE;
			if (soc <= RECHARGE_SOC && ui_soc == FULL_SOC)
				*is_done = KAL_FALSE;
		 }
	}
	else
		*is_done = KAL_FALSE;

	return status;
}


static int charging_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	u32 status = STATUS_OK;
	unsigned char fan54005_CON6_BYTE = 0;
#ifdef CONFIG_ONTIM_DSM
	 if (fan54005_dsm_client && (fan54005_get_chip_status()==0x03))
	 {
	 	u8 reg[8];
		int i;
		for (i=0;i<7;i++)
		{
		    fan54005_read_interface(i, &reg[i],0xFF,0);
		}
		fan54005_read_interface(0x10,&reg[i],0xFF,0);
	 	if ( (fan54005_dsm_client ) && dsm_client_ocuppy(fan54005_dsm_client))
	 	{
			int error=OMTIM_DSM_CHARGER_ERROR;
	 		if ((fan54005_dsm_client->dump_buff) && (fan54005_dsm_client->buff_size)&&(fan54005_dsm_client->buff_flag == OMTIM_DSM_BUFF_OK))
	 		{
				fan54005_dsm_client->used_size = sprintf(fan54005_dsm_client->dump_buff,"Type=%d; ID=%d; error_id=%d;  Charger info:%s; ",fan54005_dsm_client->client_type,fan54005_dsm_client->client_id,error,fan54005_dsm_client->client_name);
				for (i=0;i<7;i++)
				{
					fan54005_dsm_client->used_size += sprintf(fan54005_dsm_client->dump_buff+fan54005_dsm_client->used_size,"Reg[0x%x]=0x%x; ",i,reg[i]);
				}
				fan54005_dsm_client->used_size += sprintf(fan54005_dsm_client->dump_buff+fan54005_dsm_client->used_size,"Reg[0x10]=0x%x; ",reg[i]);
				fan54005_dsm_client->used_size += sprintf(fan54005_dsm_client->dump_buff+fan54005_dsm_client->used_size,"\n");
				dsm_client_notify(fan54005_dsm_client,error);
	 		}
	 	}
		else
		{
			printk(KERN_ERR "%s: dsm ocuppy error!!!",__func__);
		}

	 }
#endif
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // ISAFE = 1450mA, VSAFE = 4.42V
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // set safety reg twice according to ds. fixme
	fan54005_read_byte(fan54005_CON6, &fan54005_CON6_BYTE);
	if (fan54005_CON6_BYTE != FAN54005_SAFETY_VALUE)
		chr_err("%s %d fan54005 set safety register failed!\n", __func__, __LINE__);
	fan54005_set_tmr_rst(1);

	return status;
}

#if 0
static int charging_set_hv_threshold(void *data)
{
	u32 status = STATUS_OK;

	u32 set_hv_voltage;
	u32 array_size;
	u16 register_value;
	u32 voltage = *(u32 *) (data);
    if (global_hw_type == 1)//modified vcdt
    {
        array_size = GETARRAYNUM(FAN54005_VCDT_HV_VTH);
        set_hv_voltage = bmt_find_closest_level(FAN54005_VCDT_HV_VTH, array_size, voltage);
        register_value = charging_parameter_to_value(FAN54005_VCDT_HV_VTH, array_size, set_hv_voltage);
        pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
    }
    else
    {
        array_size = GETARRAYNUM(FAN54005_VCDT_HV_VTH_FOR_OLD);
        set_hv_voltage = bmt_find_closest_level(FAN54005_VCDT_HV_VTH_FOR_OLD, array_size, voltage);
        register_value = charging_parameter_to_value(FAN54005_VCDT_HV_VTH_FOR_OLD, array_size, set_hv_voltage);
        pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
    }
	return status;
}


static int charging_get_hv_status(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif
	return status;
}


static int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;

#if 1 //defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	//battery_log(BAT_LOG_CRTI, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	unsigned int val = 0;

	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool *) (data) = KAL_FALSE;
	}
#endif

	return status;
}


static int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif

	*(kal_bool *) (data) = val;

	return status;
}

static int charging_get_charger_type(void *data)
{
	u32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif

	return status;
}

static int charging_get_is_pcm_timer_trigger(void *data)
{
	u32 status = STATUS_OK;
/* M migration
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;
	battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
*/
	*(kal_bool *)(data) = KAL_FALSE;
	return status;
}

static int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");
	/* arch_reset(0,NULL); */
#endif

	return status;
}

static int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *) (data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return status;
}

static int charging_get_power_source(void *data)
{
	u32 status = STATUS_UNSUPPORTED;

	return status;
}

static int charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static int charging_set_hz_mode(void *data)
{
	unsigned int status = STATUS_OK;
        unsigned int enable = *(unsigned int *) (data);

        fan54005_set_hz_mode(enable);

        return status;
}
#endif
static int fan54005_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	pr_info("%s: event = %d\n", __func__, event);
	switch (event) {
	case EVENT_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case EVENT_RECHARGE:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}
static int set_vindpm_voltage(struct charger_device *chg_dev, u32 vindpm)
{
	return STATUS_UNSUPPORTED;
}

static int fan54005_enable_otg(struct charger_device *chg_dev, bool isenable)
{
	unsigned char fan54005_CON6_BYTE = 0;
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // ISAFE = 1450mA, VSAFE = 4.42V
	fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // set safety reg twice according to ds. fixme
	fan54005_read_byte(fan54005_CON6, &fan54005_CON6_BYTE);
	if (fan54005_CON6_BYTE != FAN54005_SAFETY_VALUE)
		chr_err("%s %d fan54005 set safety register failed!\n", __func__, __LINE__);
	//fan54005_set_opa_mode(isenable);
	fan54005_set_otg_en(isenable);
	printk(KERN_ERR " OTG %s %d\n", __func__, __LINE__);
	return 0;
}



/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/

void fan54005_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = fan54005_read_interface(0x03, &val, 0xFF, 0x0);
	if (val == 0x41)
	{
           g_Second_charger_ic=g_Second_charger_ic_name;
		   is_hl7005_flag = true;
	}
	if (val == 0x51)
	{
           g_Second_charger_ic=g_third_charger_ic_name;
	}
	if (val == 0x94 ||val == 0x41||val == 0x51)  //0x94--FAN54005; 0x41--HL7005D; 0x51--BQ24157
		g_fan54005_hw_exist = 0;
	else
		g_fan54005_hw_exist = 1;

	chr_err( "[fan54005_hw_component_detect] exist=%d, Reg[03]=0x%x\n", g_fan54005_hw_exist, val);
}

int is_fan54005_exist(void)
{
	chr_err( "[is_fan54005_exist] g_fan54005_hw_exist=%d\n", g_fan54005_hw_exist);

	return g_fan54005_hw_exist;
}



static struct charger_ops fan54005_chg_ops = {
#if 0
	.enable_hz = bq25890_enable_hz,
#endif

	/* Normal charging */
	.charger_init = charging_hw_init,
	.dump_registers = charging_dump_register,
	.enable = charging_enable,
	.get_charging_current = charging_get_current,
	.set_charging_current = charging_set_current,
	.get_input_current = charging_get_input_current,
	.set_input_current = charging_set_input_current,
	/*.get_constant_voltage = bq25890_get_battery_voreg,*/
	.set_constant_voltage = charging_set_cv_voltage,
	.kick_wdt = charging_reset_watch_dog_timer,
	.set_mivr = set_vindpm_voltage,
	.is_charging_done = charging_get_charging_status,
	.event = fan54005_do_event,
	.set_dpm = fan54005_set_dpm,
	.enable_otg = fan54005_enable_otg,
	.set_hz_mode = fan54005_set_hz_mode,
};
extern bool charger_probe_ok;//bit5
static int fan54005_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int ret = 0;
		unsigned char fan54005_CON6_BYTE = 0;
		struct fan54005_info *info = NULL;
		new_client = client;
        new_client->addr = fan54005_SLAVE_ADDR_WRITE;
        new_client->addr =  new_client->addr >>1;
		if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
			return -EIO;
		fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // ISAFE = 1450mA, VSAFE = 4.42V
		fan54005_reg_config_interface(fan54005_CON6, FAN54005_SAFETY_VALUE); // set safety reg twice according to ds. fix me
		fan54005_read_byte(fan54005_CON6, &fan54005_CON6_BYTE);
		if (fan54005_CON6_BYTE != FAN54005_SAFETY_VALUE)
			chr_err("%s %d fan54005 set safety register failed!\n", __func__, __LINE__);
		fan54005_hw_component_detect();
        fan54005_dump_register();
        if(is_fan54005_exist()==0)
        {
            if(g_Second_charger_ic)
				strncpy(charge_ic_vendor_name, g_Second_charger_ic, strlen(g_Second_charger_ic)+1);
			chr_err( "%s: line=%d, find charger %s\n", __func__, __LINE__, charge_ic_vendor_name);
			REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
            chargin_hw_init_done = KAL_TRUE;
			charger_probe_ok = 1;
        }
        else
        {
			chr_err( "%s: line=%d, not find charger!!\n", __func__, __LINE__);
            return -1;//is_fan54005_exist = 1 //ret = -1;
        }
		info = devm_kzalloc(&client->dev, sizeof(struct fan54005_info), GFP_KERNEL);
		if (!info)
			return -ENOMEM;
	
		new_client = client;
		info->dev = &client->dev;
		info->chg_dev_name = "primary_chg";
		info->chg_props.alias_name = "fan54005";
		info->eint_name = "chr_stat";
		
		/* Register charger device */
		info->chg_dev = charger_device_register(info->chg_dev_name,
			&client->dev, info, &fan54005_chg_ops, &info->chg_props);
		if (IS_ERR_OR_NULL(info->chg_dev)) {
			chr_err("%s: register charger device failed\n", __func__);
			ret = PTR_ERR(info->chg_dev);
			return ret;
		}
		
        chr_err( "fan54005_driver_probe  line=%d\n", __LINE__);

	return ret;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
#define FAN54005_BUSNUM 5
static struct i2c_board_info __initdata i2c_fan54005 = { I2C_BOARD_INFO("fan54005", (fan54005_SLAVE_ADDR_WRITE>>1))};
static int __init fan54005_init(void)
{
	struct device_node *node = of_find_compatible_node(NULL, NULL, "fan54005");

	chr_err("[fan54005_init] init start\n");

	if (!node)  
	{
		chr_err("[fan54005_init] dts not found\n");
		i2c_register_board_info(FAN54005_BUSNUM, &i2c_fan54005, 1);
	}

	if (i2c_add_driver(&fan54005_driver) != 0) {
		chr_err(
			    "[fan54005_init] failed to register fan54005 i2c driver.\n");
	} else {
		chr_err(
			    "[fan54005_init] Success to register fan54005 i2c driver.\n");
	}

	return 0;
}

static void __exit fan54005_exit(void)
{
	i2c_del_driver(&fan54005_driver);
}

module_init(fan54005_init);
module_exit(fan54005_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C fan54005 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
