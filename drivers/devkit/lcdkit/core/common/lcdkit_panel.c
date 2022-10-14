#include "lcdkit_panel.h"
#include "lcdkit_dbg.h"
#include "lcdkit_parse.h"
#include "primary_display.h"
#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#include "lcdkit_dsm.h"
#endif


static char* sence_array[SENCE_ARRAY_SIZE] = {
	"LCD_INCOME0",   "MMI0",   "RUNNINGTEST0", "PROJECT_MENU0",
	"LCD_INCOME1",   "MMI1",   "RUNNINGTEST1",  "PROJECT_MENU1",
	"LCD_INCOME2",   "MMI2",   "RUNNINGTEST2",  "PROJECT_MENU2",
	"LCD_INCOME3",   "MMI3",   "RUNNINGTEST3",  "PROJECT_MENU3",
	"LCD_INCOME4",   "MMI4",   "RUNNINGTEST4",  "PROJECT_MENU4",
	"LCD_INCOME5",   "MMI5",   "RUNNINGTEST5",  "PROJECT_MENU5",
	"LCD_INCOME6",   "MMI6",   "RUNNINGTEST6",  "PROJECT_MENU6",
	"LCD_INCOME7",   "MMI7",   "RUNNINGTEST7",  "PROJECT_MENU7",
	"LCD_INCOME8",   "MMI8",   "RUNNINGTEST8",  "PROJECT_MENU8",
	"LCD_INCOME9",   "MMI9",   "RUNNINGTEST9",  "PROJECT_MENU9",
	"LCD_INCOME10",  "MMI10",  "RUNNINGTEST10",  "PROJECT_MENU10",
	"LCD_INCOME11",  "MMI11",  "RUNNINGTEST11",  "PROJECT_MENU11",
	"LCD_INCOME12",  "MMI12",  "RUNNINGTEST12",  "PROJECT_MENU12",
	"LCD_INCOME13",  "MMI13",  "RUNNINGTEST13",  "PROJECT_MENU13",
	"LCD_INCOME14",  "MMI14",  "RUNNINGTEST14",  "PROJECT_MENU14",
	"LCD_INCOME15",  "MMI15",  "RUNNINGTEST15",  "PROJECT_MENU15",
	"LCD_INCOME16",  "MMI16",  "RUNNINGTEST16",  "PROJECT_MENU16",
	"LCD_INCOME17",  "MMI17",  "RUNNINGTEST17",  "PROJECT_MENU17",
	"CURRENT1_0",	"CURRENT1_1", "CURRENT1_2",  "CURRENT1_3",
	"CURRENT1_4",	"CURRENT1_5", "CHECKSUM1",  "CHECKSUM2",
	"CHECKSUM3",	 "CHECKSUM4", "BL_OPEN_SHORT",  "PCD_ERRORFLAG",
	"DOTINVERSION",  "CHECKREG", "COLUMNINVERSION",   "POWERONOFF",
	"BLSWITCH",};

static char* cmd_array[SENCE_ARRAY_SIZE] = {
	"CURRENT1_0",   "CURRENT1_0",  "CURRENT1_0",  "CURRENT1_0",//current test0
	"CURRENT1_1",   "CURRENT1_1",  "CURRENT1_1",  "CURRENT1_1",//current test1
	"CURRENT1_2",   "CURRENT1_2",  "CURRENT1_2",  "CURRENT1_2",//current test2
	"CURRENT1_3",   "CURRENT1_3",  "CURRENT1_3",  "CURRENT1_3",//current test3
	"CURRENT1_4",   "CURRENT1_4",  "CURRENT1_4",  "CURRENT1_4",//current test4
	"CURRENT1_5",   "CURRENT1_5",  "CURRENT1_5",  "CURRENT1_5",//current test5
	"CHECKSUM1",   "CHECKSUM1",   "CHECKSUM1", "CHECKSUM1",//checksum1
	"CHECKSUM2",   "CHECKSUM2",   "CHECKSUM2", "CHECKSUM2",//checksum2
	"CHECKSUM3",	"CHECKSUM3",   "CHECKSUM3", "CHECKSUM3",//checksum3
	"CHECKSUM4",   "CHECKSUM4",   "CHECKSUM4", "CHECKSUM4",//checksum4
	"BL_OPEN_SHORT",   "BL_OPEN_SHORT",   "BL_OPEN_SHORT", "BL_OPEN_SHORT",//backlight open short test
	"PCD_ERRORFLAG",   "PCD_ERRORFLAG",  "PCD_ERRORFLAG", "PCD_ERRORFLAG", // PCD and errorflag test
	"DOTINVERSION",	"DOTINVERSION",  "DOTINVERSION", "DOTINVERSION",// dot inversion test
	"CHECKREG",	"CHECKREG",  "CHECKREG", "CHECKREG",// check ic status reg
	"COLUMNINVERSION", "COLUMNINVERSION", "COLUMNINVERSION", "COLUMNINVERSION", //column inversion test
	"POWERONOFF",   "POWERONOFF",  "POWERONOFF",  "POWERONOFF",// power on/off test
	"BLSWITCH",	"BLSWITCH",  "BLSWITCH", "BLSWITCH",// backlight switch test
	"GPU_TEST",   "GPU_TEST",  "GPU_TEST", "GPU_TEST", //GPU SLT test
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/ina231/ina231_0/ina231_set," \
	"/sys/class/ina231/ina231_0/ina231_value," \
	"1,9999999,1,9999999,1,99999",
	"/sys/class/graphics/fb0/lcd_checksum",
	"/sys/class/graphics/fb0/lcd_checksum",
	"/sys/class/graphics/fb0/lcd_checksum",
	"/sys/class/graphics/fb0/lcd_checksum",
	"/sys/class/lm36923/lm36923/self_test",
	"/sys/class/graphics/fb0/amoled_pcd_errflag_check",
	"/sys/class/graphics/fb0/lcd_inversion_mode",
	"/sys/class/graphics/fb0/lcd_check_reg",
	"/sys/class/graphics/fb0/lcd_inversion_mode",
	"/sys/class/graphics/fb0/lcd_check_reg",
	"/sys/class/graphics/fb0/lcd_check_reg",};

struct lcdkit_adapt_func lcdkit_adapt_func_array[] = {
	{"JDI_NT35696 5.1' CMD TFT 1920 x 1080", lcdkit_jdi_nt35696_5p5_gram_check_show, lcdkit_jdi_nt35696_5p5_reg_read_show},//checksum show
};
uint32_t checksum_start = LCDKIT_CHECKSUM_END;

/***********************************************************
  *function: LCD resoure init, include gpio,regulator,xml parse and so on.
  *input:
  *@np: this node is used to parse DTS file.
  *output: get the pinfo struct
*/
void lcdkit_init(struct device_node* np, void* pdata)
{

}

/***********************************************************
  *function: send display on cmds of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@cmds:  the display on code
*/
void lcdkit_on_cmd(void* pdata, struct lcdkit_dsi_panel_cmds* cmds)
{
	lcdkit_dsi_tx(pdata, cmds);
}

/***********************************************************
  *function:  send display off cmds of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@cmds:  the display off code
*/
void lcdkit_off_cmd(void* pdata, struct lcdkit_dsi_panel_cmds* cmds)
{
	lcdkit_dsi_tx(pdata, cmds);
}

/***********************************************************
  *function: this function is used to get the lcdkit_panel_data struct.
*/
struct lcdkit_panel_data* lcdkit_get_panel_info()
{
	return &lcdkit_info;
}

/***********************************************************
  *function: this function is used to get the support flag.
*/
uint32_t lcdkit_get_dbc_set_boost_ctrl_flag(void)
{
	LCDKIT_INFO("bl_dbc_set_boost_ctrl_flag=%u\n",lcdkit_info.panel_infos.bl_dbc_set_boost_ctrl_flag);
	return lcdkit_info.panel_infos.bl_dbc_set_boost_ctrl_flag;
}

/***********************************************************
  *function:  this function is used to get the type of LCD
  *output:
  *@buf: get the type of lcd
*/
static ssize_t lcdkit_model_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", lcdkit_info.panel_infos.model_name);
}

/***********************************************************
  *function:  this function is used to get the panel info of LCD
  *output:
  *@buf: get the type of lcd
*/
static ssize_t lcdkit_panel_info_show(char* buf)
{
	return snprintf(buf,PAGE_SIZE,"blmax:%d,blmin:%d,blmax_nit_actual:%d,blmax_nit_standard:%d,lcdtype:LCD\n",
			lcdkit_info.panel_infos.bl_level_max,lcdkit_info.panel_infos.bl_level_min,lcdkit_info.panel_infos.bl_max_nit,lcdkit_info.panel_infos.bl_max_nit);
}

/***********************************************************
  *function:  this function is used to get the panel status of LCD
  *output:
  *@buf: get the type of lcd
*/

static ssize_t lcdkit_panel_status_show(char* buf)
{
	unsigned char value;
	unsigned char* idle;//idle mode on/off
	unsigned char* sleep;//sleep in/out
	unsigned char* normal;//display normal mode on/off
	unsigned char* display;//display on/off
	lcdkit_dsi_rx(lcdkit_info.panel_infos.panel_status_cmds,&value);
	if ((value >> 6) & 0x01){
		idle = "on";
	}else{
		idle = "off";
	}

	if ((value >> 4) & 0x01){
		sleep = "out";
	}else{
		sleep = "in";
	}

	if((value >> 3) & 0x01){
		normal = "on";
	}else{
		normal = "0ff";
	}

	if((value >> 2) & 0x01){
		display = "on";
	}else{
		display = "off";
	}
	return snprintf(buf,PAGE_SIZE,"panel_info:Idle Mode %s,Sleep %s,Display Normal Mode %s,Display %s\n",idle,sleep,normal,display);
}

#ifdef CONFIG_LCT_CABC_MODE_SUPPORT
extern int primary_display_setcabc(unsigned int mode);
#endif
#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT
extern int primary_display_setscanmode(unsigned int mode);
#endif
#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT
extern int primary_display_setinversionmode(unsigned int mode);
#endif

extern int primary_display_lcd_state(void);

/***********************************************************
  *function: this function is used to get the cabc mode of LCD
  *output:
  *@buf: get the cabc mode of lcd
*/
static ssize_t lcdkit_cabc_mode_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.cabc_mode);
}

/***********************************************************
 *function: this function is used to set the cabc mode of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@buf: the value of cabc mode to be set
*/
static ssize_t lcdkit_cabc_mode_store(void* pdata, const char* buf)
{
	int ret = 0;
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		LCDKIT_ERR("invalid data!\n");
		return -EINVAL;
	}
#ifdef CONFIG_LCT_CABC_MODE_SUPPORT
	primary_display_setcabc(val);
#endif
	lcdkit_info.panel_infos.cabc_mode = (int)val;
	LCDKIT_INFO("cabc_mode = %d\n", lcdkit_info.panel_infos.cabc_mode);

	return ret;
}

/***********************************************************
  *function: this function is used to get the inversion mode of LCD
  *output:
  *@buf: get the inversion mode of lcd
*/
static ssize_t lcdkit_inversion_mode_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.inversion_mode);
}

/***********************************************************
  *function: this function is used to set the inversion mode of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@buf: the value of inversion mode to be set
*/
static ssize_t lcdkit_inversion_mode_store(void* pdata, const char* buf)
{
	ssize_t ret = 0;
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);
	if (ret)
	{
		LCDKIT_ERR("%s invalid data!\n",__func__);
		return -EINVAL;
	}

#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT
	primary_display_setinversionmode(val);
#endif
	lcdkit_info.panel_infos.inversion_mode = (int)val;
	LCDKIT_INFO("inversion_mode = %d\n", lcdkit_info.panel_infos.inversion_mode);

	return ret;
}

/***********************************************************
  *function: this function is used to get the scan mode of LCD
  *output:
  *@buf: get the scan mode of lcd.
*/
static ssize_t lcdkit_scan_mode_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.scan_mode);
}

/***********************************************************
  *function: this function is used to set the inversion mode of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@buf: the value of scan mode to be set
*/
static ssize_t lcdkit_scan_mode_store(void* pdata,  const char* buf)
{
	ssize_t ret = 0;
	unsigned long val;
	ret = strict_strtoul(buf, 0, &val);
	if (ret)
	{
		LCDKIT_ERR("%s invalid data!\n",__func__);
		return -EINVAL;
	}

#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT
	primary_display_setscanmode(val);
#endif

	lcdkit_info.panel_infos.scan_mode = (int)val;
	LCDKIT_INFO("scan_mode = %d\n", lcdkit_info.panel_infos.scan_mode);
	return ret;
}

/***********************************************************
  *function: this function is used to check the status reg of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *output:
   *@buf: output the check result, "OK" means SUCCESS, "FAIL" means FAIL
*/
#define LCDKIT_REG_CHECK_NUM	5
static ssize_t lcdkit_check_reg_show(void* pdata, char* buf)
{
    int ret = 0;
	int i = 0;
    unsigned int count;
	unsigned char reg_for_check;
	int pram_num = 0;
    unsigned char value[5];
	unsigned char pram_expect[5] = {0,0,0,0,0};

	reg_for_check = lcdkit_info.panel_infos.reg_for_check_lcm;
	pram_num = lcdkit_info.panel_infos.pram_num_lcm;

	for (i=0;i<pram_num;i++)
	{
		pram_expect[i] = lcdkit_info.panel_infos.pram_expect_lcm[i];
	}



    for (count = 0; count < LCDKIT_REG_CHECK_NUM; count++)
    {
        ret = 0;
		lcdkit_dsi_rx_block_data(reg_for_check,value,pram_num);
		for(i=0;i<pram_num;i++)
		{
        if(value[i] != pram_expect[i])
        {
            ret = 1;
            LCDKIT_INFO("%s value[%d]=0x%0x\n",__func__,i,value[i]);
            LCDKIT_INFO("%s pram_expect[%d]=0x%0x\n",__func__,i,pram_expect[i]);
        }
            LCDKIT_INFO("%s value[%d]=0x%0x\n",__func__,i,value[i]);
            LCDKIT_INFO("%s pram_expect[%d]=0x%0x\n",__func__,i,pram_expect[i]);
		}
        if (0 == ret)
        {
            break;
        }
		else
		{
			continue;
		}
    }
    if (0 == ret)
    {
        ret = snprintf(buf, PAGE_SIZE, "OK\n");
    }
    else
    {
        ret = snprintf(buf, PAGE_SIZE, "ERROR\n");
        lcdkit_report_dsm_err(DSM_LCD_STATUS_ERROR_NO, 0, 0, 0);
    }
    return ret;
}

/***********************************************************
  *function: this function is used to check the esd status reg of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@cmds: the esd reg to be check.
  *output:
*/
static ssize_t lcdkit_check_esd(void* pdata)
{
	ssize_t ret = 0;
	return ret;
}

/***********************************************************
  *function: this function is used to check the gram status reg of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *output:
  *@buf: output the check result, "OK" means SUCCESS, "FAIL" means FAIL
*/
static ssize_t lcdkit_gram_check_show(void* pdata, char* buf)
{
	ssize_t ret = 0;
	return ret;
}

static ssize_t lcdkit_gram_check_store(void* pdata, const char* buf)
{
	 ssize_t ret = 0;
	unsigned long val = 0;
	static int pic_4_check_count = 0;
	int pic_index = 0;

	ret = strict_strtoul(buf, 0, &val);
	if (ret)
	{
		return ret;
	}
	LCDKIT_INFO("val=%ld\n", val);

	pic_index = val - 2;
	if (LCDKIT_CHECKSUM_END == checksum_start){
		/* The first pic num 2 used to instruct that checksum start */
		if (TEST_PIC_0 == pic_index){
			LCDKIT_INFO("start gram checksum...\n");
			checksum_start = LCDKIT_CHECKSUM_START;
			lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.checksum_enable_cmds);
			lcdkit_effect_switch_ctrl(pdata, 1);
			lcdkit_info.panel_infos.checksum_pic_num = 0xff;
			LCDKIT_INFO("Enable checksum\n");
			return ret;
		}else{
			LCDKIT_INFO("gram checksum not start, and first cmd not start:2\n");
			return -1;
		}
	}else{
		if (TEST_PIC_2 == pic_index) pic_4_check_count++;
		if (CHECKSUM_CHECKCOUNT == pic_4_check_count){
			LCDKIT_INFO("gram checksum pic %ld test 5 times, set checkout end\n", val);
			checksum_start = LCDKIT_CHECKSUM_END;
			pic_4_check_count = 0;
		}
	}

	switch(pic_index){
		case TEST_PIC_0:
		case TEST_PIC_1:
		case TEST_PIC_2:
			LCDKIT_INFO("gram checksum set pic [%d]\n", pic_index);
			lcdkit_info.panel_infos.checksum_pic_num = pic_index;
			break;
		default:
			LCDKIT_INFO("gram checksum set pic [%d] unknown\n", pic_index);
			lcdkit_info.panel_infos.checksum_pic_num = 0xff;
			break;
	}

	return ret;
}

static ssize_t lcdkit_dynamic_sram_check_show(void* pdata, char* buf)
{
	ssize_t ret = 0;
	return ret;
}

static ssize_t lcdkit_dynamic_sram_check_store(void* pdata, const char* buf)
{
	ssize_t ret = 0;
	return ret;
}

static ssize_t lcdkit_ce_mode_show(void* pdata, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.ce_mode);
}

static ssize_t lcdkit_ce_mode_store(void* pdata, const char* buf, size_t count)
{
	int ret = 0;
	unsigned long val = 0;
	int flag=-1;

	ret = strict_strtoul(buf, 0, &val);
	if(ret)
	   return ret;

	flag=(int)val;
	lcdkit_info.panel_infos.ce_mode = flag;

	return count;
}

static ssize_t lcdkit_sleep_ctrl_show(char* buf)
{
	return  snprintf(buf, PAGE_SIZE, "enable_PT_test=%d, PT_test_support=%d\n",
					 lcdkit_info.panel_infos.enable_PT_test, lcdkit_info.panel_infos.PT_test_support);
}

static ssize_t lcdkit_sleep_ctrl_store(const char* buf)
{
	int ret = 0;
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		LCDKIT_ERR("invalid parameter!\n");
		return ret;
	}
	if(val < 0 || val >2)
		return ret;

	if ( lcdkit_info.panel_infos.PT_test_support )
	{
	     if(primary_display_lcd_state())
                  lcdkit_info.panel_infos.enable_PT_test = val;
	     else
                  LCDKIT_ERR("Sleep State set invald !\n");
	}
#if 0//del for lcdkit,related to tpkit
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		LCDKIT_ERR("invalid parameter!\n");
		return ret;
	}

	if ( lcdkit_info.panel_infos.PT_test_support )
	{
		lcdkit_info.panel_infos.enable_PT_test = val;
	}

	if (lcdkit_info.panel_infos.enable_PT_test == 2)
	{
		g_tskit_pt_station_flag = 1;	//used for pt  current test, tp sleep
	}
	else
	{
		g_tskit_pt_station_flag = 0;	//used for pt  current test, tp power off
	}

	LCDKIT_INFO("exit!\n");
#endif
	return ret;

}

static ssize_t lcdkit_lp2hs_mipi_check_show(char* buf)
{
	if (lcdkit_info.panel_infos.g_lp2hs_mipi_check_result)
	{
		return snprintf(buf, PAGE_SIZE, "OK\n");
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "FAIL\n");
	}
}

static ssize_t lcdkit_lp2hs_mipi_check_store(void* pdata, const char* buf)
{
	int ret = 0;
	unsigned long val = 0;
	int flag = -1;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{ return ret; }

	flag = (int)val;

	if (flag == LP2HS_MIPI_TEST_OFF)
	{
		lcdkit_info.panel_infos.lp2hs_mipi_check_support = false;
		lcdkit_info.panel_infos.g_lp2hs_mipi_check_result = false;
		lcdkit_info.panel_infos.lcd_self_testing = false;
		LCDKIT_INFO("lp2hs_mipi test OFF\n");
	}
	else  if (flag == LP2HS_MIPI_TEST_ON)
	{
		lcdkit_info.panel_infos.lp2hs_mipi_check_support = true;
		lcdkit_info.panel_infos.g_lp2hs_mipi_check_result = true;
		lcdkit_info.panel_infos.lcd_self_testing = true;
		LCDKIT_INFO("lp2hs_mipi test ON\n");
	}

	return ret;

}

static ssize_t lcdkit_bist_check_show(void* pdata, char* buf)
{
	ssize_t ret = 0;
	return ret;
}

/*mipi detect*/
/*function:
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *output:
  *@buf: output the test result,OK means success, FAIL means fail.
*/
static ssize_t lcdkit_mipi_detect_show(void* pdata, char* buf)
{
	ssize_t ret = 0;
	return ret;
}

static ssize_t lcdkit_hkadc_debug_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.hkadc_buf * 4);
}

static ssize_t lcdkit_hkadc_debug_store(const char* buf)
{
	ssize_t ret = 0;
	int channel = 0;

	ret = sscanf(buf, "%u", &channel);

	if (ret <= 0)
	{
		LCDKIT_ERR("ivalid parameter!\n");
		return ret;
	}

	lcdkit_info.panel_infos.hkadc_buf = adc_get_value(channel);
	return ret;
}


static ssize_t lcdkit_pcd_errflag_check(char* buf )
{
	int pcd_gpio = 0;
	int errflag_gpio = 0;
	ssize_t ret = 0;
	u8 result_value = 0;

	pcd_gpio = gpio_get_value_cansleep(lcdkit_info.panel_infos.gpio_pcd);
	errflag_gpio = gpio_get_value_cansleep(lcdkit_info.panel_infos.gpio_err_flag);

	if ((pcd_gpio == 1) && (errflag_gpio == 0))
	{
		result_value = 0; // PCD_ERR_FLAG_SUCCESS
	}
	else if ((pcd_gpio == 0) && (errflag_gpio == 0))
	{
		result_value = 1; //only  PCD_FAIL
	}
	else if ((pcd_gpio == 1) && (errflag_gpio == 1))
	{
		result_value = 2; //only ERRFLAG FAIL
	}
	else if ((pcd_gpio == 0) && (errflag_gpio == 1))
	{
		result_value = 3; //PCD_ERR_FLAG_FAIL
	}
	else
	{
		result_value = 0;
	}

	LCDKIT_INFO("Test cmd is pcd_errflag,and the result_value = %d\n",result_value);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", result_value);
	return ret;

}

static ssize_t lcdkit_amoled_acl_ctrl_show(char* buf)
{
	ssize_t ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.acl_ctrl_mode);
	return ret;
}

static ssize_t lcdkit_amoled_acl_ctrl_store(void* pdata, const char* buf)
{
	ssize_t ret = 0;
	unsigned long val = 0;
	int flag = 0;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		return ret;
	}

	flag = (int)val;

	if (flag == lcdkit_info.panel_infos.acl_ctrl_mode)
	{
		return ret;
	}

	lcdkit_info.panel_infos.acl_ctrl_mode = flag;
	lcdkit_info.panel_infos.acl_enable_cmds.cmds->payload[1] = flag;

	switch (flag)
	{
		case ACL_OFF:
			lcdkit_info.panel_infos.acl_ctrl_mode = flag;
			break;

		case ACL_ON_SET1:
			lcdkit_info.panel_infos.acl_ctrl_mode = flag;
			break;

		case ACL_ON_SET2:
			lcdkit_info.panel_infos.acl_ctrl_mode = flag;
			break;

		case ACL_ON_SET3:
			lcdkit_info.panel_infos.acl_ctrl_mode = flag;
			break;

		default:
			LCDKIT_ERR("check your input parameter !\n");
			return ret;
	}

	lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.acl_enable_cmds);
	return ret;
}

static ssize_t lcdkit_amoled_vr_mode_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE,  "%d\n", lcdkit_info.panel_infos.vr_mode);
}

static ssize_t lcdkit_amoled_vr_mode_store(void* pdata, const char* buf)
{
	ssize_t ret = 0;
	unsigned long val = 0;
	int flag = -1;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		LCDKIT_ERR("string to l int error, buf[%s], ret:%ld\n", buf, ret);
		return ret;
	}

	flag = (int)val;

	switch (flag)
	{
		case VR_MODE_ENABLE:
			LCDKIT_INFO("VR CYCLE SET vr mode\n");
			lcdkit_info.panel_infos.vr_mode = VR_MODE_ENABLE;
			lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.vr_mode_enable_cmds);
			break;

		case  VR_MODE_DISABLE:
			LCDKIT_INFO("VR CYCLE SET normal mode\n");
			lcdkit_info.panel_infos.vr_mode = VR_MODE_DISABLE;
			lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.vr_mode_disable_cmds);
			break;

		default:
			lcdkit_info.panel_infos.vr_mode = VR_MODE_DISABLE;
			LCDKIT_INFO("VR CYCLE SET unknown mode\n");
			break;
	}

	return ret;
}

static ssize_t lcdkit_amoled_hbm_ctrl_show(char* buf)
{
	return  snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.hbm_ctrl_mode);
}

static ssize_t lcdkit_amoled_hbm_ctrl_store(void* pdata, const char* buf)
{
	ssize_t ret = 0;
	unsigned long val = 0;

	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{ return ret; }

	lcdkit_info.panel_infos.hbm_ctrl_mode = (int)val;
	lcdkit_info.panel_infos.hbm_enable_cmds.cmds->payload[1] =  lcdkit_info.panel_infos.hbm_ctrl_mode ? 0xe8 : 0x20;

	lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.hbm_enable_cmds);

	return ret;
}

static ssize_t lcdkit_support_mode_show(char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.effect_support_mode);
}

static ssize_t lcdkit_support_mode_store(const char* buf)
{
	ssize_t ret = 0;
	unsigned long val = 0;
	ret = strict_strtoul(buf, 0, &val);

	if (ret)
	{
		return ret;
	}

	lcdkit_info.panel_infos.effect_support_mode = (int)val;
	return ret;
}

static ssize_t lcdkit_support_checkmode_show(char* buf)
{
	ssize_t ret = 0;

	ret = snprintf(buf, PAGE_SIZE, "checksum:%d;lp2hs_mipi_check:%d\n", lcdkit_info.panel_infos.checksum_support, lcdkit_info.panel_infos.lp2hs_mipi_check_support);

	return ret;
}

static ssize_t lcdkit_current_detect(void* pdata)
{
	ssize_t current_check_result = 0;
	unsigned char val = 0x00;
	LCDKIT_INFO("current detect enter!\n");

	lcdkit_dsi_rx(0xD9,&val);//register 0xD9 only for hx8394f

	if(val == 0x00) {
		current_check_result = 0;
		LCDKIT_INFO("[%s]:no current over\n",__func__);
	}else if (val == 0x80) {
		current_check_result = 1;
		LCDKIT_INFO("[%s]:current over\n",__func__);
	}else {
		current_check_result = 1;
		LCDKIT_INFO("[%s]:unknow status\n",__func__);
	}
	return current_check_result;
}

static ssize_t lcdkit_lv_detect(void* pdata)
{
	ssize_t lv_detect_result = 0;
	return lv_detect_result;
}


static ssize_t lcdkit_test_config_show(void* pdata, char* buf)
{
	int i = 0;
	int test_result = 0;

	for (i = 0; i < SENCE_ARRAY_SIZE; i++)
	{
		if (sence_array[i] == NULL) {
			LCDKIT_INFO("Sence cmd is end, total num is:%d\n", i);
			break;
		}
		if ( !strncmp(lcdkit_info.panel_infos.lcd_cmd_now, sence_array[i], strlen(lcdkit_info.panel_infos.lcd_cmd_now)))
		{
			LCDKIT_INFO("current test cmd:%s,return cmd:%s\n",
						lcdkit_info.panel_infos.lcd_cmd_now, cmd_array[i]);
			return snprintf(buf, PAGE_SIZE, cmd_array[i]);
		}
	}

	/* lcd support over current and over voltage detect */
	if (!strncmp(lcdkit_info.panel_infos.lcd_cmd_now, "CURRENT_TEST_MODE", strlen("CURRENT_TEST_MODE"))) {
		return snprintf(buf, PAGE_SIZE, "LCD_CUR_DET_MODE");
	/* over current detect */
	} else if (!strncmp(lcdkit_info.panel_infos.lcd_cmd_now, "OVER_CURRENT_DETECTION", strlen("OVER_CURRENT_DETECTION"))) {
		test_result = lcdkit_current_detect(pdata);
		return snprintf(buf, PAGE_SIZE, "%d", test_result);
	/* over voltage detect */
	} else if (!strncmp(lcdkit_info.panel_infos.lcd_cmd_now, "OVER_VOLTAGE_DETECTION", strlen("OVER_VOLTAGE_DETECTION"))) {
		test_result = lcdkit_lv_detect(pdata);
		return snprintf(buf, PAGE_SIZE, "%d", test_result);
	/* input invaild */
	} else {
		LCDKIT_INFO("cmd invaild,current test cmd:%s\n", lcdkit_info.panel_infos.lcd_cmd_now);
		return snprintf(buf, PAGE_SIZE, "INVALID");
	}
}

static ssize_t lcdkit_test_config_store(const char* buf)
{
	if (strlen(buf) < LCDKIT_CMD_NAME_MAX)
	{
		memcpy(lcdkit_info.panel_infos.lcd_cmd_now, buf, strlen(buf) + 1);
		LCDKIT_INFO("current test cmd:%s\n", lcdkit_info.panel_infos.lcd_cmd_now);
	}
	else
	{
		memcpy(lcdkit_info.panel_infos.lcd_cmd_now, "INVALID", strlen("INVALID") + 1);
		LCDKIT_INFO("invalid test cmd:%s\n", lcdkit_info.panel_infos.lcd_cmd_now);
	}

	return 0;
}

static ssize_t lcdkit_fps_scence_handle(struct platform_device* pdev, uint32_t scence)
{
	lcdkit_fps_scence_adaptor_handle(pdev, scence);

	/*for video panel, updt porch*/
	if (!lcdkit_is_cmd_panel())
	{
		lcdkit_updt_porch(pdev, scence);
	}

	return 0;
}

static ssize_t lcdkit_fps_updt_handle(void* pdata)
{
	lcdkit_fps_updt_adaptor_handle(pdata);
	return 0;
}

int lcdkit_fps_support_query(void)
{
	return lcdkit_info.panel_infos.fps_func_switch;
}

int lcdkit_fps_tscall_support_query(void)
{
	return lcdkit_info.panel_infos.fps_tscall_support;
}

/*function: this function is used to called by ts thread to let lcd driver
  *known that there is a tp touch event report in tp ic driver
  *input:void
  *@callback_type: callback_type use to tell lcd driver what to do.
*/
void lcdkit_fps_ts_callback(void)
{
	lcdkit_fps_adaptor_ts_callback();
	return ;
}

void lcdkit_fps_timer_init(void)
{
	lcdkit_fps_timer_adaptor_init();
	return ;
}

/*function: this function is used to set the Partial  display of LCD
  *input:
  *@pdata: this void point is used to converte to fb data struct.
  *@dirty: the region of partial display need
*/
static ssize_t lcdkit_set_display_region(void* pdata, void* dirty)
{
	struct dirty_rect* dirty_region = NULL;
	ssize_t ret = 0;

	if ( dirty == NULL)
	{
		LCDKIT_ERR("dirty is null point!\n");
		return ret;
	}

	dirty_region = (struct dirty_rect*) dirty;

	lcdkit_dump_cmds(&lcdkit_info.panel_infos.display_region_cmds);

	lcdkit_info.panel_infos.display_region_cmds.cmds[0].payload[1] = ((uint32_t)(dirty_region->x) >> 8) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[0].payload[2] = ((uint32_t)(dirty_region->x)) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[0].payload[3] = ((dirty_region->x + dirty_region->w - 1) >> 8) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[0].payload[4] = (dirty_region->x + dirty_region->w - 1) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[1].payload[1] = ((uint32_t)(dirty_region->y) >> 8) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[1].payload[2] = ((uint32_t)(dirty_region->y)) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[1].payload[3] = ((dirty_region->y + dirty_region->h - 1) >> 8) & 0xff;
	lcdkit_info.panel_infos.display_region_cmds.cmds[1].payload[4] = (dirty_region->y + dirty_region->h - 1) & 0xff;

	lcdkit_dump_cmds(&lcdkit_info.panel_infos.display_region_cmds);
	lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.display_region_cmds);

	return ret;
}

static void lcdkit_vsp_enable(bool en)
{
	LCDKIT_INFO("vsp enable(%d)\n", en);

	if(lcdkit_bias_is_gpio_ctrl_power())
	{
		if(en){
		gpio_set_value(lcdkit_info.panel_infos.gpio_lcd_vsp, 1);
		}else{
		gpio_set_value(lcdkit_info.panel_infos.gpio_lcd_vsp, 0);
	}
	}
	else if(lcdkit_bias_is_regulator_ctrl_power())
	{
		if(en)
		{
			;
		}
		else
		{
			;
		}
	}
}

static void lcdkit_vsn_enable(bool en)
{
	LCDKIT_INFO("vsn enable(%d)\n", en);
	if(lcdkit_bias_is_gpio_ctrl_power())
	{
		if(en){
		gpio_set_value(lcdkit_info.panel_infos.gpio_lcd_vsn, 1);
		}else{
		gpio_set_value(lcdkit_info.panel_infos.gpio_lcd_vsn, 0);
	}
	}
	else if(lcdkit_bias_is_regulator_ctrl_power())
	{
		if(en)
		{
			;
		}
		else
		{
			;
		}
	}
}


static void lcdkit_vddio_enable(bool en)
{
	LCDKIT_INFO("obsolescent\n");

}

static void lcdkit_vci_enable(bool en)
{
	LCDKIT_INFO("obsolescent\n");
}

static int lcdkit_fake_panel_bl_update(void *pdata, uint32_t bl_level)
{
	int ret = 0;
	ret = lcdkit_fake_update_bl(pdata,bl_level);
	return ret;
}

static ssize_t  lcdkit_voltage_enable_store(void *pdata, const char* buf)
{
	char command[10] = {0};

	uint32_t bl_value = 0;
	int ret = 0;
	char bl_value_buf[10] = {0};

	if (strlen(buf) > (sizeof(command)-1)){
		LCDKIT_ERR("buf underflow\n");
		return -EINVAL;
	}

	if (!sscanf(buf, "%s", command)) {
		LCDKIT_INFO("bad command(%s)\n", command);
		return -EINVAL;
	}

	LCDKIT_INFO("command(%s)\n", command);
	if (!strncmp("vsp:", command, strlen("vsp:"))) {
		if('0' == command[strlen("vsp:")])
			lcdkit_vsp_enable(false);
		else
			lcdkit_vsp_enable(true);
	}

	if (!strncmp("vsn:", command, strlen("vsn:"))) {
		if('0' == command[strlen("vsn:")])
			lcdkit_vsn_enable(false);
		else
			lcdkit_vsn_enable(true);
	}

	if (!strncmp("vddio:", command, strlen("vddio:"))) {
		if('0' == command[strlen("vddio:")])
			lcdkit_vddio_enable(false);
		else
			lcdkit_vddio_enable(true);
	}

	if (!strncmp("vci:", command, strlen("vci:"))) {
		if('0' == command[strlen("vci:")])
			lcdkit_vci_enable(false);
		else
			lcdkit_vci_enable(true);
	}

	if (!strncmp("bl:", command, strlen("bl:"))) {
		if (((strlen(command) - strlen("bl:")) > 0) &&((strlen(command) - strlen("bl:")) < sizeof(bl_value_buf))) {
			memcpy(bl_value_buf, &command[strlen("bl:")], strlen(command)-strlen("bl:"));
			bl_value = simple_strtoul(bl_value_buf, NULL, 0);
			LCDKIT_INFO("bl_value_buf is %s, bl_value is %d\n", bl_value_buf, bl_value);
		} else {
			bl_value = 0;
		}

		ret = lcdkit_fake_panel_bl_update(pdata, bl_value);
	}

	return ret;
}

static ssize_t lcdkit_reg_read_show(void* pdata, char *buf)
{
	char lcd_reg_buf[LCD_REG_LENGTH_MAX] = {0};
	uint32_t read_value[LCD_REG_LENGTH_MAX] = {0};
	unsigned char str_tmp[LCD_REG_LENGTH_MAX] = {0};
	char lcd_reg[] = {0x00};
	int i = 0;
	int ret = 0;
	struct lcdkit_dsi_cmd_desc lcd_reg_cmd[] = {
		{DTYPE_GEN_READ1, 0, 0, 0, 10, LCDKIT_WAIT_TYPE_US,
		sizeof(lcd_reg), lcd_reg},
	};
	for (i = 0; i < lcdkit_info.panel_infos.gama_correct_reg.cnt; i++) {
		if ((lcdkit_info.panel_infos.gama_reg_addr == lcdkit_info.panel_infos.gama_correct_reg.buf[i]) &&
				lcdkit_info.panel_infos.gama_reg_length == lcdkit_info.panel_infos.gama_reg_len.buf[i]) {
			break;
		}
	}
	if (i == lcdkit_info.panel_infos.gama_correct_reg.cnt) {
		LCDKIT_ERR("reg is not expect, reg:0x%x\n", lcdkit_info.panel_infos.gama_reg_addr);
		goto error_out;
	}
	for (i = 0; i < (int)(sizeof(lcdkit_adapt_func_array)/sizeof(lcdkit_adapt_func_array[0])); i++) {
		if (!strncmp(lcdkit_info.panel_infos.panel_name, lcdkit_adapt_func_array[i].name, strlen(lcdkit_adapt_func_array[i].name))) {
			if (lcdkit_adapt_func_array[i].lcdkit_reg_read_show) {
				ret = lcdkit_adapt_func_array[i].lcdkit_reg_read_show(pdata, buf);
				return ret;
			}
		}
	}

	memset(lcd_reg_buf, 0, sizeof(lcd_reg_buf));
	memset(read_value, 0, sizeof(read_value));
	lcd_reg[0] = lcdkit_info.panel_infos.gama_reg_addr;
	if (lcdkit_info.panel_infos.gama_cmd_page != 0) {
		lcdkit_info.panel_infos.lcd_reg_check_enter_cmds.cmds->payload[1] = lcdkit_info.panel_infos.gama_cmd_page & 0xFF;
		lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.lcd_reg_check_enter_cmds);
	}

	ret = lcdkit_lread_reg(pdata, read_value, &lcd_reg_cmd, lcdkit_info.panel_infos.gama_reg_length);
	if (ret) {
		LCDKIT_INFO("read error, ret=%d\n", ret);
		goto error_out;
	}
	if (lcdkit_info.panel_infos.gama_cmd_page != 0) {
		lcdkit_dsi_tx(pdata, &lcdkit_info.panel_infos.lcd_reg_check_exit_cmds);
	}
	for (i = 0; i < (lcdkit_info.panel_infos.gama_reg_length + 3) / 4; i++) {
		LCDKIT_INFO("0x%8x\n", read_value[i]);
	}
	snprintf(lcd_reg_buf, sizeof(lcd_reg_buf), "1,");
	for (i = 0; i < lcdkit_info.panel_infos.gama_reg_length; i++) {
		switch (i % 4) {
		case 0:
			if (i == lcdkit_info.panel_infos.gama_reg_length - 1) {
				snprintf(str_tmp, sizeof(str_tmp), "%d", read_value[i / 4] & 0xFF);
			} else {
				snprintf(str_tmp, sizeof(str_tmp), "%d,", read_value[i / 4] & 0xFF);
			}
			break;
		case 1:
			if (i == lcdkit_info.panel_infos.gama_reg_length - 1) {
				snprintf(str_tmp, sizeof(str_tmp), "%d", (read_value[i / 4] >> 8) & 0xFF);
			} else {
				snprintf(str_tmp, sizeof(str_tmp), "%d,", (read_value[i / 4] >> 8) & 0xFF);
			}
			break;
		case 2:
			if (i == lcdkit_info.panel_infos.gama_reg_length - 1) {
				snprintf(str_tmp, sizeof(str_tmp), "%d", (read_value[i / 4] >> 16) & 0xFF);
			} else {
				snprintf(str_tmp, sizeof(str_tmp), "%d,", (read_value[i / 4] >> 16) & 0xFF);
			}
			break;
		case 3:
			if (i == lcdkit_info.panel_infos.gama_reg_length - 1) {
				snprintf(str_tmp, sizeof(str_tmp), "%d", (read_value[i / 4] >> 24) & 0xFF);
			} else {
				snprintf(str_tmp, sizeof(str_tmp), "%d,", (read_value[i / 4] >> 24) & 0xFF);
			}
			break;
		default:
			break;
		}
		strncat(lcd_reg_buf, str_tmp, strlen(str_tmp));
	}
	LCDKIT_INFO("%s\n", lcd_reg_buf);
	ret = snprintf(buf, sizeof(lcd_reg_buf), "%s\n", lcd_reg_buf);
	return ret;
error_out:
	LCDKIT_INFO("error out, reg addr=%d, reg length=%d\n", lcdkit_info.panel_infos.gama_reg_addr, lcdkit_info.panel_infos.gama_reg_length);
	ret = snprintf(buf, PAGE_SIZE, "0,%d,%d\n", (int)lcdkit_info.panel_infos.gama_reg_addr, lcdkit_info.panel_infos.gama_reg_length);
	return ret;
}
static ssize_t lcdkit_reg_read_store(void* pdata, const char *buf)
{
	int ret = 0;
	unsigned int reg_value[100];
	char *cur;
	char *token;
	int i = 0;
	cur = (char*)buf;
	token = strsep(&cur, ",");
	while (token) {
		reg_value[i++] = simple_strtol(token, NULL, 0);
		token = strsep(&cur, ",");
		if (i >= 100)
		{
			LCDKIT_INFO("count is too long\n");
			return -1;
		}
	}
	lcdkit_info.panel_infos.gama_cmd_page = ((reg_value[0] >> 8) & 0xFF);
	lcdkit_info.panel_infos.gama_reg_addr = (unsigned char)(reg_value[0] & 0xFF);
	lcdkit_info.panel_infos.gama_reg_length = reg_value[1];
	LCDKIT_INFO("cmd page:0x%x, reg addr:0x%x, reg length:%d\n", lcdkit_info.panel_infos.gama_cmd_page, lcdkit_info.panel_infos.gama_reg_addr, lcdkit_info.panel_infos.gama_reg_length);
	return ret;
}
static ssize_t lcdkit_bl_mode_store(void* pdata, const char *buf)
{
	ssize_t ret = -1;
	unsigned long val = 0;
	unsigned int bl_work_mode = 0;

	if (NULL == buf)
	{
		return ret;
	}

	if (lcdkit_info.panel_infos.bl_support_mode)
	{
		ret = strict_strtoul(buf, 0, &val);
		if (ret)
		{
			LCDKIT_ERR("get bl mode fail!\n");
			return ret;
		}

		bl_work_mode = (unsigned int)val;

		LCDKIT_INFO("lcdkit_info.bl_work_mode=%u, val=%ld, bl_work_mode=%u\n", lcdkit_info.panel_infos.bl_work_mode, val,bl_work_mode);
		if (lcdkit_info.panel_infos.bl_work_mode == bl_work_mode)
		{
			LCDKIT_INFO("It is same bl mode!\n");
			return ret;
		}

		lcdkit_info.panel_infos.bl_work_mode = bl_work_mode;
		switch (val)
		{
			case LCDKIT_BL_NORMAL_MODE:
				ret = lcdkit_set_bl_normal_mode_reg(pdata);
				break;
			case LCDKIT_BL_ENHANCE_MODE:
				ret = lcdkit_set_bl_enhance_mode_reg(pdata);
				break;
			default:
				break;
		}
	}
	return ret;
}

static ssize_t lcdkit_bl_mode_show(char *buf)
{
	if (NULL == buf)
	{
		return -1;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.bl_work_mode);
}

static ssize_t lcdkit_support_bl_mode_show(char* buf)
{
	if (NULL == buf)
	{
		return -1;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", lcdkit_info.panel_infos.bl_support_mode);
}

static ssize_t lcdkit_oem_info_show(void* pdata, char* buf)
{
	if (lcdkit_info.panel_infos.is_hostprocessing == 1) {
		return host_panel_oem_info_show(pdata, buf);
	} else {
		LCDKIT_INFO("It is normal panel!\n");
	}
	return 0;
}

static ssize_t lcdkit_oem_info_store(void* pdata, const char* buf)
{
	if (lcdkit_info.panel_infos.is_hostprocessing == 1) {
		return host_panel_oem_info_store(pdata, buf);
	} else {
		LCDKIT_INFO("It is normal panel!\n");
	}
	return 0;
}

struct lcdkit_panel_data lcdkit_info =
{
	.lcdkit_init = lcdkit_init,
	.lcdkit_on_cmd  = lcdkit_on_cmd,
	.lcdkit_off_cmd = lcdkit_off_cmd,
	.lcdkit_model_show = lcdkit_model_show,
	.lcdkit_panel_info_show =  lcdkit_panel_info_show,
	.lcdkit_panel_status_show =  lcdkit_panel_status_show,
	.lcdkit_cabc_mode_show = lcdkit_cabc_mode_show,
	.lcdkit_cabc_mode_store = lcdkit_cabc_mode_store,
	.lcdkit_inversion_mode_store = lcdkit_inversion_mode_store,
	.lcdkit_inversion_mode_show = lcdkit_inversion_mode_show,
	.lcdkit_scan_mode_store = lcdkit_scan_mode_store,
	.lcdkit_scan_mode_show = lcdkit_scan_mode_show,
	.lcdkit_check_reg_show = lcdkit_check_reg_show,
	.lcdkit_check_esd  = lcdkit_check_esd,
	.lcdkit_gram_check_show = lcdkit_gram_check_show,
	.lcdkit_gram_check_store = lcdkit_gram_check_store,
	.lcdkit_dynamic_sram_check_show = lcdkit_dynamic_sram_check_show,
	.lcdkit_dynamic_sram_check_store = lcdkit_dynamic_sram_check_store,
	.lcdkit_sleep_ctrl_show = lcdkit_sleep_ctrl_show,
	.lcdkit_sleep_ctrl_store = lcdkit_sleep_ctrl_store,
	.lcdkit_lp2hs_mipi_check_show = lcdkit_lp2hs_mipi_check_show,
	.lcdkit_lp2hs_mipi_check_store = lcdkit_lp2hs_mipi_check_store,
	.lcdkit_bist_check_show = lcdkit_bist_check_show,
	.lcdkit_mipi_detect_show = lcdkit_mipi_detect_show,
	.lcdkit_hkadc_debug_show = lcdkit_hkadc_debug_show,
	.lcdkit_hkadc_debug_store = lcdkit_hkadc_debug_store,
	.lcdkit_voltage_enable_store = lcdkit_voltage_enable_store,
	.lcdkit_pcd_errflag_check = lcdkit_pcd_errflag_check,
	.lcdkit_amoled_acl_ctrl_show = lcdkit_amoled_acl_ctrl_show,
	.lcdkit_amoled_acl_ctrl_store = lcdkit_amoled_acl_ctrl_store,
	.lcdkit_amoled_vr_mode_show = lcdkit_amoled_vr_mode_show,
	.lcdkit_amoled_vr_mode_store = lcdkit_amoled_vr_mode_store,
	.lcdkit_amoled_hbm_ctrl_show = lcdkit_amoled_hbm_ctrl_show,
	.lcdkit_amoled_hbm_ctrl_store = lcdkit_amoled_hbm_ctrl_store,
	.lcdkit_oem_info_show = lcdkit_oem_info_show,
	.lcdkit_oem_info_store = lcdkit_oem_info_store,
	.lcdkit_support_mode_show = lcdkit_support_mode_show,
	.lcdkit_support_mode_store = lcdkit_support_mode_store,
	.lcdkit_support_checkmode_show = lcdkit_support_checkmode_show,
	.lcdkit_test_config_show = lcdkit_test_config_show,
	.lcdkit_test_config_store = lcdkit_test_config_store,
	.lcdkit_fps_scence_handle = lcdkit_fps_scence_handle,
	.lcdkit_fps_updt_handle = lcdkit_fps_updt_handle,
	.lcdkit_set_display_region = lcdkit_set_display_region,
	.lcdkit_current_detect = lcdkit_current_detect,
	.lcdkit_lv_detect = lcdkit_lv_detect,
	.lcdkit_ce_mode_show = lcdkit_ce_mode_show,
	.lcdkit_ce_mode_store = lcdkit_ce_mode_store,
	.lcdkit_reg_read_show = lcdkit_reg_read_show,
	.lcdkit_reg_read_store = lcdkit_reg_read_store,
	.lcdkit_fps_timer_init = lcdkit_fps_timer_init,
	.lcdkit_bl_mode_show   = lcdkit_bl_mode_show,
	.lcdkit_bl_mode_store  = lcdkit_bl_mode_store,
	.lcdkit_support_bl_mode_show = lcdkit_support_bl_mode_show,
};
