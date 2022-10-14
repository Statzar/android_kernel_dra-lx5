/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#include "lcdkit_panel.h"
#ifdef CONFIG_LCDKIT_DRIVER
#include "lcdkit_fb_util.h"
#endif
#if defined (CONFIG_HUAWEI_DSM)
#include <dsm/dsm_pub.h>
#include "lcdkit_dsm.h"
#endif
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif


static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

extern int NT50358A_write_byte(unsigned char addr, unsigned char value);
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT									(1440)
#define LCM_DENSITY										(320)

#define LCM_PHYSICAL_WIDTH									(64800)
#define LCM_PHYSICAL_HEIGHT									(129600)




#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY	0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	0xFFFE
#define REGFLAG_RESET_HIGH	0xFFFF

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif



struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0x0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0x0, {} },
	{REGFLAG_DELAY, 120, {} },
};

static const struct LCM_setting_table init_setting[] = {
	{0xB9, 0x03, {0xFF,0x83,0x94}},
	{0xBA, 0x06, {0x62,0x03,0x68,0x6B,0xB2,0xC0}},
	{0xB1, 0x0A, {0x48,0x0F,0x6F,0x09,0x32,0x54,0x71,0x51,0x57,0x43}},
	{0xB2, 0x06, {0x00,0x80,0x78,0x0C,0x0D,0x22}},
	{0xB4, 0x15, {0x69,0x6A,0x69,0x6A,0x69,0x6A,0x01,0x0C,0x7C,0x55,0x00,0x3F,0x69,0x6A,0x69,0x6A,0x69,0x6A,0x01,0x0C,0x7C}},
	{0xB6, 0x02, {0x6E,0x6E}},
	{0xD3, 0x21, {0x00,0x00,0x07,0x07,0x40,0x07,0x0C,0x00,0x08,0x10,0x08,0x00,0x08,0x54,0x15,0xAA,0x05,0xAA,0x02,0x15,0x06,0x05,0x06,0x47,0x44,0x0A,0x0A,0x4B,0x10,0x07,0x07,0x0C,0x40}},
	{0xD5, 0x2C, {0x20,0x18,0x18,0x21,0x26,0x18,0x24,0x27,0x1C,0x25,0x1D,0x1C,0x00,0x1D,0x02,0x01,0x04,0x03,0x18,0x18,0x06,0x05,0x18,0x18,0x08,0x07,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x09,0x18,0x18,0x0A,0x0B,0x18,0x18,0x18,0x18}},
	{0xD6, 0x2C, {0x25,0x18,0x18,0x24,0x27,0x18,0x21,0x26,0x1C,0x20,0x1D,0x1C,0x0B,0x1D,0x09,0x0A,0x07,0x08,0x18,0x18,0x05,0x06,0x18,0x18,0x03,0x04,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x18,0x18,0x01,0x00,0x18,0x18,0x18,0x18}},
	{0xE0, 0x3A, {0x00,0x02,0x0A,0x0E,0x10,0x13,0x16,0x13,0x27,0x38,0x48,0x48,0x53,0x66,0x6E,0x73,0x84,0x88,0x86,0x9A,0xAC,0x56,0x56,0x5C,0x5F,0x60,0x60,0x5E,0x5C,0x00,0x02,0x0A,0x0E,0x10,0x13,0x16,0x13,0x27,0x38,0x48,0x48,0x53,0x66,0x6D,0x73,0x83,0x88,0x87,0x9A,0xAC,0x56,0x56,0x5C,0x5F,0x60,0x60,0x5E,0x5C}},
	{0xC0, 0x02, {0x1F,0x31}},
	{0xCC, 0x01, {0x0B}},
	{0xD4, 0x01, {0x02}},
	{0xBD, 0x01, {0x02}},
	{0xD8, 0x0C, {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},
	{0xBD, 0x01, {0x00}},
	{0xBD, 0x01, {0x01}},
	{0xB1, 0x01, {0x00}},
	{0xBD, 0x01, {0x00}},
	{0xC6, 0x01, {0xED}},
	{0xC9, 0x03, {0x0B,0x01,0xAA}},
	{0x51, 0x01, {0x00}},
	{0x53, 0x01, {0x24}},
	{0x55, 0x01, {0x00}},
	{0x11, 0x01, {0x00}},
	{REGFLAG_DELAY, 120,{}},
	{0x29, 0x01, {0x00}},
	{REGFLAG_DELAY, 20,{}}
};
#ifdef CONFIG_LCT_CABC_MODE_SUPPORT

#define  CABC_MODE_SETTING_DIS 0
#define  CABC_MODE_SETTING_UI 1
#define  CABC_MODE_SETTING_STILL 2
#define  CABC_MODE_SETTING_MV  3

static struct LCM_setting_table lcm_setting_dis[] = {
	{0x55,0x01,{0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,0x01,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_ui[] = {
	{0x55,0x01,{0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,0x01,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_still[] = {
	{0x55,0x01,{0x02}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,0x01,{0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_mv[] = {
	{0x55,0x01,{0x03}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,0x01,{0x2c}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table_v22(void *handle, struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {

		unsigned cmd;
		void *cmdq = handle;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}

static void lcm_cabc_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case CABC_MODE_SETTING_DIS:
			push_table_v22(handle,lcm_setting_dis, sizeof(lcm_setting_dis) /sizeof(struct LCM_setting_table), 1);
		        break;
		case CABC_MODE_SETTING_UI:
			push_table_v22(handle,lcm_setting_ui, sizeof(lcm_setting_ui) /sizeof(struct LCM_setting_table), 1);
		         break;
		case CABC_MODE_SETTING_STILL:
			 push_table_v22(handle,lcm_setting_still, sizeof(lcm_setting_still) /sizeof(struct LCM_setting_table), 1);
			break;
		case CABC_MODE_SETTING_MV:
			 push_table_v22(handle,lcm_setting_mv, sizeof(lcm_setting_mv) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_ui, sizeof(lcm_setting_ui) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif

#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT

#define  NORMAL_SCAN 0
#define  REVERSE_SCAN 1

static struct LCM_setting_table lcm_setting_normalscan[] = {
	{0xCC, 0x01, {0x0B}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_reversescan[] = {
	{0xCC, 0x01, {0x07}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void lcm_scanmode_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case NORMAL_SCAN:
			push_table_v22(handle,lcm_setting_normalscan, sizeof(lcm_setting_normalscan) /sizeof(struct LCM_setting_table), 1);
		        break;
		case REVERSE_SCAN:
			push_table_v22(handle,lcm_setting_reversescan, sizeof(lcm_setting_reversescan) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_normalscan, sizeof(lcm_setting_normalscan) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif

#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT

#define	COLUMN_INVERSION  0
#define	DOT_INVERSION_1  1
#define	DOT_INVERSION_2 2
#define	DOT_INVERSION_3 3
#define	DOT_INVERSION_4 4

static struct LCM_setting_table lcm_setting_column[] = {
	{0xB2,0x01,{0x40}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_1_dot[] = {
	{0xB2,0x01,{0x41}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_2_dot[] = {
	{0xB2,0x01,{0x42}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_3_dot[] = {
	{0xB2,0x01,{0x43}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_4_dot[] = {
	{0xB2,0x01,{0x44}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_inversionmode_cmdq(void *handle, unsigned int mode)
{
	switch(mode){
		case COLUMN_INVERSION:
			push_table_v22(handle,lcm_setting_column, sizeof(lcm_setting_column) /sizeof(struct LCM_setting_table), 1);
		        break;
		case DOT_INVERSION_1:
			push_table_v22(handle,lcm_setting_1_dot, sizeof(lcm_setting_1_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_2:
			push_table_v22(handle,lcm_setting_2_dot, sizeof(lcm_setting_2_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_3:
			push_table_v22(handle,lcm_setting_3_dot, sizeof(lcm_setting_3_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		case DOT_INVERSION_4:
			push_table_v22(handle,lcm_setting_4_dot, sizeof(lcm_setting_4_dot) /sizeof(struct LCM_setting_table), 1);
		         break;
		default:
			 push_table_v22(handle,lcm_setting_column, sizeof(lcm_setting_column) /sizeof(struct LCM_setting_table), 1);
	}
}
#endif
static struct LCM_setting_table bl_level[] = {
	{0x51, 0x01, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
				if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density = LCM_DENSITY;


#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif
	LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 14;
	params->dsi.vertical_frontporch = 32;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 30;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 3;
	params->dsi.PLL_CLOCK = 325;	/* this value must be in MTK suggested table */
	params->dsi.HS_TRAIL = 7;
	params->dsi.CLK_HS_PRPR = 6;
	params->dsi.CLK_HS_POST = 36;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x81;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x83;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x06;
	params->dsi.lcm_esd_check_table[1].cmd = 0x54;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x24;

}

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0x0F;  //up to +/-5.5V
	int ret = 0;
	LCM_LOGI("kernel lcm_init enter\n");
	set_gpio_lcd_enp(1);
	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("boyi_hx8394f----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("boyi_hx8394f----nt50358a----cmd=%0x--i2c write success----\n", cmd);

	cmd = 0x01;
	data = 0x0F;

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("boyi_hx8394f----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("boyi_hx8394f----nt50358a----cmd=%0x--i2c write success----\n", cmd);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(10);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	LCM_LOGI("kernel lcm_init exit\n");
}

static void lcm_suspend(void)
{
	LCM_LOGI("[kernel] lcm_suspend enter\n");
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(10);
	set_gpio_lcd_enp(0);

	LCM_LOGI("[kernel] lcm_suspend exit\n");

}


static void lcm_resume(void)
{
	lcm_init();
}



static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	LCM_LOGI("%s,hx8394f backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = level;
	push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}


#ifdef CONFIG_LCDKIT_DRIVER
#include "lcdkit_fb_util.h"
#endif

static void  set_lcdkit_info(void)
{
           lcdkit_info.panel_infos.cabc_support = 1;
           lcdkit_info.panel_infos.cabc_mode = 0;
           lcdkit_info.panel_infos.inversion_support = 1;
           lcdkit_info.panel_infos.inversion_mode = 0;
           lcdkit_info.panel_infos.scan_support = 1;
           lcdkit_info.panel_infos.scan_mode = 0;
           lcdkit_info.panel_infos.esd_support = 1;
           lcdkit_info.panel_infos.mipi_detect_support = 1;
           lcdkit_info.panel_infos.lp2hs_mipi_check_support = 1;
           lcdkit_info.panel_infos.check_reg_support = 1;
           lcdkit_info.panel_infos.bias_power_ctrl_mode |= POWER_CTRL_BY_GPIO;
           lcdkit_info.panel_infos.gpio_lcd_vsn = 90;
           lcdkit_info.panel_infos.gpio_lcd_vsp = 17;
           lcdkit_info.panel_infos.enable_PT_test = 0;
           lcdkit_info.panel_infos.PT_test_support = 1;
           lcdkit_info.panel_infos.model_name = "BOYI_HX8394F 5.45' VIDEO TFT 720x1440";
           lcdkit_info.panel_infos.reg_for_check_lcm = 0x0A;
           lcdkit_info.panel_infos.pram_num_lcm = 1;
           lcdkit_info.panel_infos.pram_expect_lcm[0] = 0x1C;
           lcdkit_info.panel_infos.bl_level_max = 255;
           lcdkit_info.panel_infos.bl_level_min = 2;
           lcdkit_info.panel_infos.bl_max_nit = 0;
           lcdkit_info.panel_infos.panel_status_cmds = 0x0a;
}
LCM_DRIVER boyi_hx8394f_5p45_hd_video_lcm_drv = {
	.name = "boyi_hx8394f_5p45_hd_video_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.set_lcm_panel_support = set_lcdkit_info,
#ifdef CONFIG_LCT_CABC_MODE_SUPPORT
	.set_cabc_cmdq = lcm_cabc_cmdq,
#endif
#ifdef CONFIG_LCT_SCAN_MODE_SUPPORT
	.set_scanmode_cmdq = lcm_scanmode_cmdq,
#endif
#ifdef CONFIG_LCT_INVERSION_MODE_SUPPORT
	.set_inversionmode_cmdq = lcm_inversionmode_cmdq,
#endif


};
