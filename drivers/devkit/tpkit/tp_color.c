/*
 * Copyright (C) huawei company
 *
 * This	program	is free	software; you can redistribute it and/or modify
 * it under	the	terms of the GNU General Public	License	version	2 as
 * published by	the	Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
//#include <linux/mtd/hisi_nve_interface.h>
#include <linux/module.h>
#include "tpkit_platform_adapter.h"
#include "tp_color.h"
#include <huawei_platform/log/hw_log.h>
#define HWLOG_TAG tp_color
HWLOG_REGIST();

#define WHITE	0xE1
#define WHITE_OLD 0x02
#define BLACK	0xD2
#define BLACK_OLD 0x01
#define PINK	0xC3
#define RED		0xB4
#define YELLOW	0xA5
#define BLUE	0x96
#define GOLD  0x87
#define GOLD_OLD 0x04
#define SILVER  0x3C
#define GRAY  0x78
#define CAFE  0x69
#define CAFE2  0x5A
#define BLACK2  0x4B
#define TP_COLOR_BUF_SIZE		20
#define TP_COLOR_SIZE   15

static char tp_color_buf[TP_COLOR_BUF_SIZE] ={0};

u8 cypress_ts_kit_color[TP_COLOR_SIZE] = { 0 };
#define NV_DATA_SIZE 104 

#define DEC_BASE_DATA   10
/******************************************************************************
Function:	    read_tp_color
******************************************************************************/
static int read_tp_color(void)
{
	int tp_color;
	hwlog_info("tp color is %s\n", tp_color_buf);

	tp_color = (int)simple_strtol(tp_color_buf, NULL, 0);
	return tp_color;
}

static int __init early_parse_tp_color_cmdline(char *arg)
{
	unsigned int len = 0;
	memset(tp_color_buf, 0, sizeof(tp_color_buf));
	if (arg) {
		len = strlen(arg);

		if (len > sizeof(tp_color_buf)) {
			len = sizeof(tp_color_buf);
		}
		memcpy(tp_color_buf, arg, len);
	} else {
		hwlog_info("%s : arg is NULL\n", __func__);
	}

	return 0;
}
/*lint -save -e* */
early_param("TP_COLOR", early_parse_tp_color_cmdline);
/*lint -restore*/
static int is_color_correct(u8 color)
{
	if((color & 0x0f) == ((~(color >> 4)) & 0x0f))
		return true;
	else if ((color == BLACK_OLD) || (color == WHITE_OLD) || (color == GOLD_OLD))
		return true;
	else
		return false;
}

static int read_tp_color_from_nv(char *color_info)
{
	int ret = 0;
	char nv_data[NV_DATA_SIZE]={0};

//	ret = read_tp_color_adapter(nv_data);
	if (ret < 0) {
		hwlog_err("nve_direct_access read error(%d)\n", ret);
		return ret;
	}

	if ((!strncmp(nv_data, "white", strlen("white")))
	    || (!strncmp(nv_data, "black", strlen("black")))
	    || (!strncmp(nv_data, "silver", strlen("silver")))
	    || (!strncmp(nv_data, "pink", strlen("pink")))
	    || (!strncmp(nv_data, "red", strlen("red")))
	    || (!strncmp(nv_data, "yellow", strlen("yellow")))
	    || (!strncmp(nv_data, "blue", strlen("blue")))
	    || (!strncmp(nv_data, "gold", strlen("gold")))
	    || (!strncmp(nv_data, "gray", strlen("gray")))
	    || (!strncmp(nv_data, "cafe", strlen("cafe")))
	    || (!strncmp(nv_data, "pdblack", strlen("pdblack")))) {
		strncpy(color_info, nv_data,
			strlen(nv_data));
		color_info[strlen(nv_data)] = '\0';
		return 0;
	} else {
		hwlog_err("[%s]:read_tp_color_from_nv read unormal value!!\n",
			  __func__);
		return -1;
	}
}

static int write_tp_color_to_nv(void)
{
	int ret = 0;
	u8 lcd_id = 0;
	u8 phone_color = 0;
	char nv_data[NV_DATA_SIZE]={0};

	lcd_id = read_tp_color();
	if (lcd_id != 0xff) {
		hwlog_info("lcd id is %u from read tp color\n", lcd_id);
	}
	if (is_color_correct(cypress_ts_kit_color[0]))
		phone_color = cypress_ts_kit_color[0];
	else if (is_color_correct(lcd_id))
		phone_color = lcd_id;
	else {
		hwlog_err("LCD/TP ID both error!\n");
		return -1;
	}
	switch (phone_color) {
	case WHITE:
		strncpy(nv_data, "white", sizeof(nv_data));
		break;
	case BLACK:
		strncpy(nv_data, "black", sizeof(nv_data));
		break;
	case PINK:
		strncpy(nv_data, "pink", sizeof(nv_data));
		break;
	case RED:
		strncpy(nv_data, "red", sizeof(nv_data));
		break;
	case YELLOW:
		strncpy(nv_data, "yellow", sizeof(nv_data));
		break;
	case BLUE:
		strncpy(nv_data, "blue", sizeof(nv_data));
		break;
	case GOLD:
		strncpy(nv_data, "gold", sizeof(nv_data));
		break;
	case SILVER:
		strncpy(nv_data, "silver", sizeof(nv_data));
		break;
	case GRAY:
		strncpy(nv_data, "gray", sizeof(nv_data));
		break;
	case CAFE:
	case CAFE2:
		strncpy(nv_data, "cafe", sizeof(nv_data));
		break;
	case BLACK2:
		strncpy(nv_data, "pdblack", sizeof(nv_data));
		break;
	default:
		strncpy(nv_data, "", sizeof(nv_data));
		break;
	}
#if 0
	ret = write_tp_color_adapter(nv_data);
	if(ret < 0)
	{
		return ret;
	}
#endif
	hwlog_info("[%s]:(%s)\n", __func__, nv_data);
	return ret;
}

static ssize_t attr_get_tp_color_info(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 lcd_id = 0;
	u8 phone_color = 0;
	lcd_id = read_tp_color();
	if (lcd_id != 0xff) {
		hwlog_info("lcd id is %u from read tp color\n", lcd_id);
	}
	hwlog_info("%s:tp id=%x   lcd id=%x.\n", __func__, cypress_ts_kit_color[0],
		   lcd_id);
	if (is_color_correct(cypress_ts_kit_color[0]))
		phone_color = cypress_ts_kit_color[0];
	else if (is_color_correct(lcd_id))
		phone_color = lcd_id;
	else {
		ret = read_tp_color_from_nv(buf);
		if (ret)
			strncpy(buf, "", sizeof(""));
		return strlen(buf);
	}
	switch (phone_color) {
	case WHITE:
	case WHITE_OLD:
		strncpy(buf, "white", sizeof("white"));
		break;
	case BLACK:
	case BLACK_OLD:
		strncpy(buf, "black", sizeof("black"));
		break;
	case PINK:
		strncpy(buf, "pink", sizeof("pink"));
		break;
	case RED:
		strncpy(buf, "red", sizeof("red"));
		break;
	case YELLOW:
		strncpy(buf, "yellow", sizeof("yellow"));
		break;
	case BLUE:
		strncpy(buf, "blue", sizeof("blue"));
		break;
	case GOLD:
	case GOLD_OLD:
		strncpy(buf, "gold", sizeof("gold"));
		break;
	case SILVER:
		strncpy(buf, "silver", sizeof("silver"));
		break;
	case GRAY:
		strncpy(buf, "gray", sizeof("gray"));
		break;
	case CAFE:
	case CAFE2:
		strncpy(buf, "cafe", sizeof("cafe"));
		break;
	case BLACK2:
		strncpy(buf, "pdblack", sizeof("pdblack"));
		break;
	default:
		strncpy(buf, "", sizeof(""));
		break;
	}
	hwlog_info("%s: TP color is %s\n", __func__, buf);
	return strlen(buf);
}

static ssize_t attr_set_tp_color_info(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, DEC_BASE_DATA);
	hwlog_info("[%s] val=%lu\n", __func__, val);
	if (val == 1) {
		write_tp_color_to_nv();
	}
	return count;
}

static struct device_attribute ts_kit_color_file =
__ATTR(ts_kit_color_info, 0664, attr_get_tp_color_info, attr_set_tp_color_info);

static struct platform_device huawei_ts_kit_color = {
	.name = "huawei_ts_kit_color",
	.id = -1,
};

int ts_get_tp_module_info(struct ts_module_info *info)
{
    int ret = 0;
    u8 phone_color = 0;

    // get tp module name
    ret = ts_get_tp_module_name(info->module_name, TP_MAX_STR_LEN);
    if (ret) {
        hwlog_info("%s: get tp module name error\n", __func__);
        return ret;
    }

    // get tp panel color
    if (is_color_correct(cypress_ts_kit_color[0])) {
        phone_color = cypress_ts_kit_color[0];
    } else {
        ret = read_tp_color_from_nv(info->panel_color);
        if (ret) {
            strncpy(info->panel_color, "", sizeof(""));
        }
        return ret;
    }

    switch (phone_color) {
    case WHITE:
    case WHITE_OLD:
        strncpy(info->panel_color, WHITE_COLOR, sizeof(WHITE_COLOR));
        break;
    case BLACK:
    case BLACK_OLD:
        strncpy(info->panel_color, BLACK_COLOR, sizeof(BLACK_COLOR));
        break;
    case PINK:
        strncpy(info->panel_color, PINK_COLOR, sizeof(PINK_COLOR));
        break;
    case RED:
        strncpy(info->panel_color, RED_COLOR, sizeof(RED_COLOR));
        break;
    case YELLOW:
        strncpy(info->panel_color, YELLOW_COLOR, sizeof(YELLOW_COLOR));
        break;
    case BLUE:
        strncpy(info->panel_color, BLUE_COLOR, sizeof(BLUE_COLOR));
        break;
    case GOLD:
    case GOLD_OLD:
        strncpy(info->panel_color, GOLD_COLOR, sizeof(GOLD_COLOR));
        break;
    case SILVER:
        strncpy(info->panel_color, SILVER_COLOR, sizeof(SILVER_COLOR));
        break;
    case GRAY:
        strncpy(info->panel_color, GRAY_COLOR, sizeof(GRAY_COLOR));
        break;
    case CAFE:
    case CAFE2:
        strncpy(info->panel_color, CAFE_COLOR, sizeof(CAFE_COLOR));
        break;
    case BLACK2:
        strncpy(info->panel_color, PDBLACK_COLOR, sizeof(PDBLACK_COLOR));
        break;
    default:
        strncpy(info->panel_color, "", sizeof(""));
        break;
    }
    hwlog_info("%s: TP color is %s\n", __func__, info->panel_color);
    return ret;
}

static int __init ts_color_info_init(void)
{
	int ret = 0;
	hwlog_info("[%s] ++", __func__);

	ret = platform_device_register(&huawei_ts_kit_color);
	if (ret) {
		hwlog_err("%s: platform_device_register failed, ret:%d.\n",
			  __func__, ret);
		goto REGISTER_ERR;
	}
	if (device_create_file(&huawei_ts_kit_color.dev, &ts_kit_color_file)) {
		hwlog_err("%s:Unable to create interface\n", __func__);
		goto SYSFS_CREATE_FILE_ERR;
	}
	hwlog_info("[%s] --", __func__);
	return 0;

SYSFS_CREATE_FILE_ERR:
	platform_device_unregister(&huawei_ts_kit_color);
REGISTER_ERR:
	return ret;

}
/*lint -save -e* */
device_initcall(ts_color_info_init);
/*lint -restore*/
MODULE_DESCRIPTION("ts color info");
MODULE_AUTHOR("huawei driver group of k3");
MODULE_LICENSE("GPL");
