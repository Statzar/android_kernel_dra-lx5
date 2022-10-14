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
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 60, {} },
};

static const struct LCM_setting_table init_setting[] = {
	{ 0xFF, 0x03, {0x98, 0x81, 0x03} },
	/* GIP_1 */
	{ 0x01, 0x01, {0x00} },
	{ 0x02, 0x01, {0x00} },
	{ 0x03, 0x01, {0x53} },
	{ 0x04, 0x01, {0x13} },
	{ 0x05, 0x01, {0x00} },
	{ 0x06, 0x01, {0x04} },
	{ 0x07, 0x01, {0x00} },
	{ 0x08, 0x01, {0x00} },
	{ 0x09, 0x01, {0x14} },
	{ 0x0a, 0x01, {0x14} },
	{ 0x0b, 0x01, {0x00} },
	{ 0x0c, 0x01, {0x01} },
	{ 0x0d, 0x01, {0x00} },
	{ 0x0e, 0x01, {0x00} },
	{ 0x0f, 0x01, {0x14} },
	{ 0x10, 0x01, {0x14} },
	{ 0x11, 0x01, {0x00} },
	{ 0x12, 0x01, {0x00} },
	{ 0x13, 0x01, {0x00} },
	{ 0x14, 0x01, {0x00} },
	{ 0x15, 0x01, {0x00} },
	{ 0x16, 0x01, {0x00} },
	{ 0x17, 0x01, {0x00} },
	{ 0x18, 0x01, {0x00} },
	{ 0x19, 0x01, {0x00} },
	{ 0x1a, 0x01, {0x00} },
	{ 0x1b, 0x01, {0x00} },
	{ 0x1c, 0x01, {0x00} },
	{ 0x1d, 0x01, {0x00} },
	{ 0x1e, 0x01, {0x44} },
	{ 0x1f, 0x01, {0x80} },
	{ 0x20, 0x01, {0x02} },
	{ 0x21, 0x01, {0x03} },
	{ 0x22, 0x01, {0x00} },
	{ 0x23, 0x01, {0x00} },
	{ 0x24, 0x01, {0x00} },
	{ 0x25, 0x01, {0x00} },
	{ 0x26, 0x01, {0x00} },
	{ 0x27, 0x01, {0x00} },
	{ 0x28, 0x01, {0x33} },
	{ 0x29, 0x01, {0x03} },
	{ 0x2a, 0x01, {0x00} },
	{ 0x2b, 0x01, {0x00} },
	{ 0x2c, 0x01, {0x00} },
	{ 0x2d, 0x01, {0x00} },
	{ 0x2e, 0x01, {0x00} },
	{ 0x2f, 0x01, {0x00} },
	{ 0x30, 0x01, {0x00} },
	{ 0x31, 0x01, {0x00} },
	{ 0x32, 0x01, {0x00} },
	{ 0x33, 0x01, {0x00} },
	{ 0x34, 0x01, {0x04} },
	{ 0x35, 0x01, {0x00} },
	{ 0x36, 0x01, {0x00} },
	{ 0x37, 0x01, {0x00} },
	{ 0x38, 0x01, {0x3c} },
	{ 0x39, 0x01, {0x00} },
	{ 0x3a, 0x01, {0x40} },
	{ 0x3b, 0x01, {0x40} },
	{ 0x3c, 0x01, {0x00} },
	{ 0x3d, 0x01, {0x00} },
	{ 0x3e, 0x01, {0x00} },
	{ 0x3f, 0x01, {0x00} },
	{ 0x40, 0x01, {0x00} },
	{ 0x41, 0x01, {0x00} },
	{ 0x42, 0x01, {0x00} },
	{ 0x43, 0x01, {0x00} },
	{ 0x44, 0x01, {0x00} },
	{REGFLAG_UDELAY, 100, {} },
	/* GIP_2 */
	{ 0x50, 0x01, {0x01} },
	{ 0x51, 0x01, {0x23} },
	{ 0x52, 0x01, {0x45} },
	{ 0x53, 0x01, {0x67} },
	{ 0x54, 0x01, {0x89} },
	{ 0x55, 0x01, {0xab} },
	{ 0x56, 0x01, {0x01} },
	{ 0x57, 0x01, {0x23} },
	{ 0x58, 0x01, {0x45} },
	{ 0x59, 0x01, {0x67} },
	{ 0x5a, 0x01, {0x89} },
	{ 0x5b, 0x01, {0xab} },
	{ 0x5c, 0x01, {0xcd} },
	{ 0x5d, 0x01, {0xef} },
	{REGFLAG_UDELAY, 100, {} },
	/* GIP_3 */
	{ 0x5e, 0x01, {0x11} },
	{ 0x5f, 0x01, {0x01} },
	{ 0x60, 0x01, {0x00} },
	{ 0x61, 0x01, {0x15} },
	{ 0x62, 0x01, {0x14} },
	{ 0x63, 0x01, {0x0c} },
	{ 0x64, 0x01, {0x0d} },
	{ 0x65, 0x01, {0x0e} },
	{ 0x66, 0x01, {0x0f} },
	{ 0x67, 0x01, {0x06} },
	{ 0x68, 0x01, {0x02} },
	{ 0x69, 0x01, {0x02} },
	{ 0x6a, 0x01, {0x02} },
	{ 0x6b, 0x01, {0x02} },
	{ 0x6c, 0x01, {0x02} },
	{ 0x6d, 0x01, {0x02} },
	{ 0x6e, 0x01, {0x08} },
	{ 0x6f, 0x01, {0x02} },
	{ 0x70, 0x01, {0x02} },
	{ 0x71, 0x01, {0x02} },
	{ 0x72, 0x01, {0x02} },
	{ 0x73, 0x01, {0x02} },
	{ 0x74, 0x01, {0x02} },
	{ 0x75, 0x01, {0x01} },
	{ 0x76, 0x01, {0x00} },
	{ 0x77, 0x01, {0x15} },
	{ 0x78, 0x01, {0x14} },
	{ 0x79, 0x01, {0x0C} },
	{ 0x7a, 0x01, {0x0D} },
	{ 0x7b, 0x01, {0x0e} },
	{ 0x7c, 0x01, {0x0f} },
	{ 0x7d, 0x01, {0x08} },
	{ 0x7e, 0x01, {0x02} },
	{ 0x7f, 0x01, {0x02} },
	{ 0x80, 0x01, {0x02} },
	{ 0x81, 0x01, {0x02} },
	{ 0x82, 0x01, {0x02} },
	{ 0x83, 0x01, {0x02} },
	{ 0x84, 0x01, {0x06} },
	{ 0x85, 0x01, {0x02} },
	{ 0x86, 0x01, {0x02} },
	{ 0x87, 0x01, {0x02} },
	{ 0x88, 0x01, {0x02} },
	{ 0x89, 0x01, {0x02} },
	{ 0x8A, 0x01, {0x02} },
	{REGFLAG_UDELAY, 100, {} },
	/* CMD_Page 4 */
	{ 0xFF, 0x03, {0x98, 0x81, 0x04} },
	{ 0x00, 0x01, {0x00} },
	{ 0x6C, 0x01, {0x15} },
	{ 0x6E, 0x01, {0x2b} },
	{ 0x6F, 0x01, {0x35} },
	{ 0x35, 0x01, {0x1f} },
	{ 0x3A, 0x01, {0x24} },
	{ 0x8D, 0x01, {0x14} },
	{ 0x87, 0x01, {0xBA} },
	{ 0x26, 0x01, {0x76} },
	{ 0xB2, 0x01, {0xD1} },
	{ 0xB5, 0x01, {0x06} },
	{ 0x33, 0x01, {0x14} },
	{ 0x38, 0x01, {0x01} },
	{ 0x39, 0x01, {0x00} },
	{REGFLAG_UDELAY, 100, {} },
	/* CMD_Page 1 */
	{ 0xFF, 0x03, {0x98, 0x81, 0x01} },
	{ 0x22, 0x01, {0x0A} },
	{ 0x31, 0x01, {0x00} },
	{ 0x53, 0x01, {0xAB} },
	{ 0x55, 0x01, {0xB6} },
	{ 0x50, 0x01, {0xB7} },
	{ 0x51, 0x01, {0xB1} },
	{ 0x60, 0x01, {0x25} },
	{ 0x62, 0x01, {0x00} },
	{ 0x63, 0x01, {0x00} },
	{ 0x2E, 0x01, {0xF0} },
	/* ============Gamma START============= */
	/* Pos Register */
	{ 0xA0, 0x01, {0x39} },
	{ 0xA1, 0x01, {0x4D} },
	{ 0xA2, 0x01, {0x58} },
	{ 0xA3, 0x01, {0x14} },
	{ 0xA4, 0x01, {0x17} },
	{ 0xA5, 0x01, {0x2B} },
	{ 0xA6, 0x01, {0x1F} },
	{ 0xA7, 0x01, {0x1F} },
	{ 0xA8, 0x01, {0xCC} },
	{ 0xA9, 0x01, {0x1A} },
	{ 0xAA, 0x01, {0x27} },
	{ 0xAB, 0x01, {0xA9} },
	{ 0xAC, 0x01, {0x1A} },
	{ 0xAD, 0x01, {0x1B} },
	{ 0xAE, 0x01, {0x4F} },
	{ 0xAF, 0x01, {0x24} },
	{ 0xB0, 0x01, {0x2A} },
	{ 0xB1, 0x01, {0x60} },
	{ 0xB2, 0x01, {0x6D} },
	{ 0xB3, 0x01, {0x3E} },

	/* Neg Register */

	{ 0xC0, 0x01, {0x39} },
	{ 0xC1, 0x01, {0x4D} },
	{ 0xC2, 0x01, {0x58} },
	{ 0xC3, 0x01, {0x14} },
	{ 0xC4, 0x01, {0x17} },
	{ 0xC5, 0x01, {0x2B} },
	{ 0xC6, 0x01, {0x1F} },
	{ 0xC7, 0x01, {0x1F} },
	{ 0xC8, 0x01, {0xCC} },
	{ 0xC9, 0x01, {0x1A} },
	{ 0xCA, 0x01, {0x27} },
	{ 0xCB, 0x01, {0xA9} },
	{ 0xCC, 0x01, {0x1A} },
	{ 0xCD, 0x01, {0x1B} },
	{ 0xCE, 0x01, {0x4F} },
	{ 0xCF, 0x01, {0x24} },
	{ 0xD0, 0x01, {0x2A} },
	{ 0xD1, 0x01, {0x60} },
	{ 0xD2, 0x01, {0x6D} },
	{ 0xD3, 0x01, {0x3E} },
	{ 0x34, 0x01, {0x01} },
	/* ============ Gamma END=========== */
	/* CMD_Page 0 */
	{0xFF, 0x03, {0x98,0x81,0x02} },
	{0x03, 0x01, {0x4D} },
	{0x04, 0x01, {0x45} },
	{0x05, 0x01, {0x44} },
	{0x06, 0x01, {0x20} },
	{0x07, 0x01, {0x00} },//pwm 30k
	{ 0xFF, 0x03, {0x98, 0x81, 0x04} },
	{ 0x92, 0x01, {0x8F} },
	{ 0xFF, 0x03, {0x98, 0x81, 0x00} },
	{ 0x51, 0x02, {0x00, 0x00} },
	{ 0x53, 0x01, {0x2C} },
	{ 0x55, 0x01, {0x01} },
	{ 0x35, 0x01, {0x00} },
	{ 0x11, 0x01, {0x00} },
	{REGFLAG_DELAY, 65, {} },
	{ 0x29, 0x01, {0x00} },
	{REGFLAG_DELAY, 1, {} }
};
#ifdef CONFIG_LCT_CABC_MODE_SUPPORT

#define  CABC_MODE_SETTING_DIS 0
#define  CABC_MODE_SETTING_UI 1
#define  CABC_MODE_SETTING_STILL 2
#define  CABC_MODE_SETTING_MV  3

static struct LCM_setting_table lcm_setting_dis[] = {
	{0x55,1,{0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_ui[] = {
	{0x55,1,{0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_still[] = {
	{0x55,1,{0x02}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2C}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_mv[] = {
	{0x55,1,{0x03}},
	{REGFLAG_DELAY, 5, {}},
	{0x53,1,{0x2c}},
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
	{0xFF,3,{0x98,0x81,01}},
	{REGFLAG_DELAY, 5, {}},
	{0x22,1,{0x0A}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,3,{0x98,0x81,00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_reversescan[] = {
	{0xFF,3,{0x98,0x81,01}},
	{REGFLAG_DELAY, 5, {}},
	{0x22,1,{0x09}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,3,{0x98,0x81,00}},
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
	{0xFF,0x03,{0x98,0x81,0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x31,0x01,{0x00}},
	{0x50,0x01,{0xC7}},
	{0x51,0x01,{0xC4}},
	{0x62,0x01,{0x00}},
	{0x63,0x01,{0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,0x03,{0x98,0x81,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_1_dot[] = {
	{0xFF,0x03,{0x98,0x81,0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x50,0x01,{0x42}},
	{0x51,0x01,{0x42}},
	{0x62,0x01,{0x2D}},
	{0x63,0x01,{0x0A}},
	{0x31,0x01,{0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,0x03,{0x98,0x81,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_2_dot[] = {
	{0xFF,0x03,{0x98,0x81,0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x50,0x01,{0x42}},
	{0x51,0x01,{0x42}},
	{0x62,0x01,{0x2D}},
	{0x63,0x01,{0x0A}},
	{0x31,0x01,{0x02}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,0x03,{0x98,0x81,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_3_dot[] = {
	{0xFF,0x03,{0x98,0x81,0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x50,0x01,{0x42}},
	{0x51,0x01,{0x42}},
	{0x62,0x01,{0x2D}},
	{0x63,0x01,{0x0A}},
	{0x31,0x01,{0x03}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,0x03,{0x98,0x81,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setting_4_dot[] = {
	{0xFF,0x03,{0x98,0x81,0x01}},
	{REGFLAG_DELAY, 5, {}},
	{0x50,0x01,{0x42}},
	{0x51,0x01,{0x42}},
	{0x62,0x01,{0x2D}},
	{0x63,0x01,{0x0A}},
	{0x31,0x01,{0x04}},
	{REGFLAG_DELAY, 5, {}},
	{0xFF,0x03,{0x98,0x81,0x00}},
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
	{0x51, 0x02, {0x0F,0xFF}},
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
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 12;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 40;
	params->dsi.horizontal_backporch = 72;
	params->dsi.horizontal_frontporch = 72;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_disable = 0;
	params->dsi.ssc_range = 1;
	params->dsi.PLL_CLOCK = 336;	/* this value must be in MTK suggested table */
	params->dsi.HS_TRAIL = 7;
	params->dsi.CLK_HS_PRPR = 6;
	params->dsi.CLK_HS_POST = 36;
	params->dsi.LPX = 5;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x09;
	params->dsi.lcm_esd_check_table[0].count = 3;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x03;
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

	printk("kernel lcm_init enter\n");

	if(0 == lcdkit_info.panel_infos.enable_PT_test)
		set_gpio_lcd_enp(1);

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("txd_ili9881c----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("txd_ili9881c----nt50358a----cmd=%0x--i2c write success----\n", cmd);

	cmd = 0x01;
	data = 0x0F;

	ret = NT50358A_write_byte(cmd, data);

	if (ret < 0)
		LCM_LOGI("txd_ili9881c----nt50358a----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("txd_ili9881c----nt50358a----cmd=%0x--i2c write success----\n", cmd);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(5);

	SET_RESET_PIN(1);
	MDELAY(15);
	push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
	printk("kernel lcm_init exit\n");
}

static void lcm_suspend(void)
{
	LCM_LOGI("[kernel] lcm_suspend enter\n");
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(10);

	if(0 == lcdkit_info.panel_infos.enable_PT_test)
		set_gpio_lcd_enp(0);

	LCM_LOGI("[kernel] lcm_suspend exit\n");

}


static void lcm_resume(void)
{
	lcm_init();
}



static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

	printk("%s,ili9881c backlight: level = %d\n", __func__, level);

	bl_level[0].para_list[0] = (level >> 4);
	bl_level[0].para_list[1] = (level << 4);
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
           lcdkit_info.panel_infos.model_name = "TXD_ILI9881C 5.45' VIDEO TFT 720x1440";
           lcdkit_info.panel_infos.reg_for_check_lcm = 0x0A;
           lcdkit_info.panel_infos.pram_num_lcm = 1;
           lcdkit_info.panel_infos.pram_expect_lcm[0] = 0x9C;
           lcdkit_info.panel_infos.bl_level_max = 255;
           lcdkit_info.panel_infos.bl_level_min = 2;
           lcdkit_info.panel_infos.bl_max_nit = 0;
           lcdkit_info.panel_infos.panel_status_cmds = 0x0a;
}
LCM_DRIVER txd_ili9881c_5p45_hd_video_lcm_drv = {
	.name = "txd_ili9881c_5p45_hd_video_drv",
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
