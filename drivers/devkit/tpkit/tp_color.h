/*
 * Huawei Touchpanel Color Driver
 *
 * Copyright (C) 2013 Huawei Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */

#ifndef __TP_COLOR_H_
#define __TP_COLOR_H_

#define TP_MAX_STR_LEN    32
#define WHITE_COLOR       "white"
#define BLACK_COLOR       "black"
#define PINK_COLOR        "pink"
#define RED_COLOR         "red"
#define YELLOW_COLOR      "yellow"
#define BLUE_COLOR        "blue"
#define GOLD_COLOR        "gold"
#define SILVER_COLOR      "silver"
#define GRAY_COLOR        "gray"
#define CAFE_COLOR        "cafe"
#define PDBLACK_COLOR     "pdblack"

struct ts_module_info {
    char module_name[TP_MAX_STR_LEN];
    char panel_color[TP_MAX_STR_LEN];
};

extern int ts_get_tp_module_name(char *name, u8 size);
extern int ts_get_tp_module_info(struct ts_module_info *info);

#endif