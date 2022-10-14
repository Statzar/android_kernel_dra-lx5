/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*/
/* MediaTek Inc. (C) 2015. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*/
#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
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
#else
#include <mt-plat/mtk_gpio.h>
#include <mt-plat/mt6763/include/mach/gpio_const.h>
//#include "mach/gpio_const.h" 
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

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

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

#define set_gpio_lcd_enn(cmd) \
    lcm_util.set_gpio_lcd_enn_bias(cmd)
#ifndef BUILD_LK

extern int NT50358A_write_byte(unsigned char addr, unsigned char value);

#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE                                    0
#define FRAME_WIDTH                                     (720)
#define FRAME_HEIGHT                                    (1440)

#define LCM_PHYSICAL_WIDTH                  (64800)
#define LCM_PHYSICAL_HEIGHT                (115200)
#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef FT8006U_DELAY
#define FT8006U_DELAY 4 
#endif

#ifndef FT8006U_DELAY_BACK
#define FT8006U_DELAY_BACK 6 
#endif

static unsigned int ontim_resume = 0;
struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
    {REGFLAG_DELAY, 20, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
};

static const struct LCM_setting_table init_setting[] = {
    {0x50,2,{0x5a,0x0c}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x80,1,{0xfa}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x50,2,{0x5a,0x0d}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x80,1,{0xf1}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x50,2,{0x5a,0x0e}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x84,1,{0x32}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x50,2,{0x5a,0x19}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0xa0,3,{0x00,0x02,0x00}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x50,2,{0x5a,0x23}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x83,1,{0x05}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x90,2,{0x7f,0x07}},
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x94, 1, {0x2c} },
    {REGFLAG_DELAY, FT8006U_DELAY, {} },
    {0x11,0,{}},                   
    {REGFLAG_DELAY, 120, {} },
    {0x29,0,{}},
    {REGFLAG_DELAY, 20, {} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }

};

static struct LCM_setting_table bl_level[] = {

    {0x50,2,{0x5a,0x0e}},
    {REGFLAG_DELAY, FT8006U_DELAY_BACK, {} },
    {0x84,1,{0x32}},
    {REGFLAG_DELAY, FT8006U_DELAY_BACK, {} },
    {0x50,2,{0x5a,0x23}},
    {REGFLAG_DELAY, FT8006U_DELAY_BACK, {} },
    {0x90, 2, {0x7f,0x07} },
    {REGFLAG_DELAY, FT8006U_DELAY_BACK, {} },
    {0x94, 1, {0x24} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
    LCM_LOGI("%s: txd_ft8006u_5p45_hd_video\n",__func__);
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

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
    lcm_dsi_mode = CMD_MODE;
#else
    params->dsi.mode = SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
    lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
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
    params->dsi.vertical_sync_active = 8; //old is 2,now is 4
    params->dsi.vertical_backporch = 106; //old is 8,now is 100
    params->dsi.vertical_frontporch = 240; //old is 15,now is 24
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active = 14;
    params->dsi.horizontal_backporch = 25;
    params->dsi.horizontal_frontporch = 45;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 0;
    params->dsi.ssc_range = 3;
    //params->dsi.HS_TRAIL = 15;
    params->dsi.PLL_CLOCK = 325;    /* this value must be in MTK suggested table */
    params->dsi.PLL_CK_CMD = 325;    
    params->dsi.PLL_CK_VDO = 325;     
    params->dsi.CLK_HS_POST = 36;

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 0;
    params->dsi.customization_esd_check_enable = 0;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
  
}

#ifdef BUILD_LK
static struct mt_i2c_t NT50358A_i2c;

static int NT50358A_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0] = addr;
    write_data[1] = value;

    NT50358A_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL; /* I2C2; */
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    NT50358A_i2c.addr = LCD_BIAS_ADDR;
    NT50358A_i2c.mode = ST_MODE;
    NT50358A_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&NT50358A_i2c, write_data, len);
    /* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

    return ret_code;
}

#endif

#define CTP_RST_GPIO_PINCTRL_ONTIM 0
extern void tpd_gpio_output(int pin, int level);
static void lcm_reset(void)
{
    //printf("[uboot]:lcm reset start.\n");
    if(ontim_resume){
        ontim_resume = 0;
        MDELAY(1);
        tpd_gpio_output(CTP_RST_GPIO_PINCTRL_ONTIM,0);
        MDELAY(10);
        tpd_gpio_output(CTP_RST_GPIO_PINCTRL_ONTIM,1);
    }
#ifdef GPIO_LCD_RST_PIN
    mt_set_gpio_mode(GPIO_LCD_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(1);
    mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_PIN, GPIO_OUT_ONE);
    MDELAY(50);
#else
    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

#endif
    LCM_LOGI("lcm reset end.\n");
}

static void lcm_init(void)
{
    unsigned char cmd = 0x0;
    unsigned char data = 0x0F;  //up to +/-5.5V
    int ret = 0;
    LCM_LOGI("%s: ontim_ft8006u\n",__func__);

#if defined(GPIO_LCD_BIAS_ENN_PIN)||defined(GPIO_LCD_BIAS_ENP_PIN)
#ifdef GPIO_LCD_BIAS_ENP_PIN 
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif    

#ifdef GPIO_LCD_BIAS_ENN_PIN 
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
#endif

#ifdef GPIO_LCD_BIAS_ENP_PIN 
    MDELAY(20);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#endif    

#ifdef GPIO_LCD_BIAS_ENN_PIN 
    MDELAY(5);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
#endif
#else
    set_gpio_lcd_enp(1);

#endif
    ret = NT50358A_write_byte(cmd, data);
    if (ret < 0)
        LCM_LOGI("ft8006u----nt50358a----cmd=%0x--i2c write error----\n", cmd);
    else
        LCM_LOGI("ft8006u----nt50358a----cmd=%0x--i2c write success----\n", cmd);
    cmd = 0x01;
    data = 0x0F;
    ret = NT50358A_write_byte(cmd, data);
    if (ret < 0)
        LCM_LOGI("ft8006u----nt50358a----cmd=%0x--i2c write error----\n", cmd);
    else
        LCM_LOGI("ft8006u----nt50358a----cmd=%0x--i2c write success----\n", cmd);
    lcm_reset();
    push_table(NULL, init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
#ifndef MACH_FPGA

    push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
#if defined(GPIO_LCD_BIAS_ENN_PIN)||defined(GPIO_LCD_BIAS_ENP_PIN)

#ifdef GPIO_LCD_BIAS_ENN_PIN 
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
#endif

#ifdef GPIO_LCD_BIAS_ENP_PIN 
    mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
#endif    
#else
    set_gpio_lcd_enp(0);
#endif

#endif
    LCM_LOGI("%s,txd_ft8006u_5p45_hd_video lcm_suspend\n", __func__);
}

static void lcm_resume(void)
{
    LCM_LOGI("%s: ontim_ft8006u\n",__func__);
    ontim_resume = 1;
    lcm_init();
}

#define LCM_ID_TCL_ILI9881C (0x9881)

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
    unsigned int ret = 0;
    return ret;
#else
    return 0;
#endif
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{   
    bl_level[6].para_list[0] = level;
    bl_level[6].para_list[1] = (level>>4) & 0x0F;
    printk(" ft8006u  %s,backlight: level = %d, set to 0x%x,0x%x \n", __func__, level,bl_level[0].para_list[0],bl_level[0].para_list[1]);
    push_table(handle, bl_level, sizeof(bl_level) / sizeof(struct LCM_setting_table), 1);
}

/*static void  set_lcdkit_info(void)
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
           lcdkit_info.panel_infos.model_name = "TXD_ft8006u 5.45' VIDEO TFT 720x1440";
           lcdkit_info.panel_infos.reg_for_check_lcm = 0x0A;
           lcdkit_info.panel_infos.pram_num_lcm = 1;
           lcdkit_info.panel_infos.pram_expect_lcm[0] = 0x9C;
           lcdkit_info.panel_infos.bl_level_max = 255;
           lcdkit_info.panel_infos.bl_level_min = 4;
           lcdkit_info.panel_infos.bl_max_nit = 0;
           lcdkit_info.panel_infos.panel_status_cmds = 0x0a;
}*/

LCM_DRIVER txd_ft8006u_5p45_hd_video_lcm_drv = {
    .name = "txd_ft8006u_5p45_hd_video_drv",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .set_backlight_cmdq = lcm_setbacklight_cmdq,
    //.set_lcm_panel_support = set_lcdkit_info,
    //.ata_check = lcm_ata_check,
};
