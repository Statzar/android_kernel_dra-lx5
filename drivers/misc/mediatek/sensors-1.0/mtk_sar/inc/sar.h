/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/



#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include "sensor_attr.h"
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/atomic.h>
#include <linux/ioctl.h>

#include <sensors_io.h>
#include <hwmsen_helper.h>
#include <hwmsensor.h>
#include "sensor_event.h"



#define SAR_TAG						"<sar_sensor> "
#define SAR_PR_ERR(fmt, args...)		pr_err(SAR_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define SAR_LOG(fmt, args...)		pr_err(SAR_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define SAR_VER(fmt, args...)		pr_err(SAR_TAG"%s %d : "fmt, __func__, __LINE__, ##args)


#define MAX_CHOOSE_SAR_NUM 5
#define SAR_INVALID_VALUE -1

struct sar_init_info {
	char *name;
	int (*init)(void);
	int (*uninit)(void);
	struct platform_driver *platform_diver_addr;
};

struct sar_control_path {
	int (*open_report_data)(int open);/* open data rerport to HAL */
	int (*enable_nodata)(int en);/* only enable not report event to HAL */
	int (*set_delay)(u64 delay);
	int (*batch)(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs);
	int (*flush)(void);/* open data rerport to HAL */
	int (*access_data_fifo)(void);/* version2.used for flush operate */
	bool is_support_batch;/* version2.used for batch mode support flag */
	bool is_polling_mode;

};

struct sar_data_path {
	int (*get_data)(int *sar_value, int *status);
	int (*sar_get_raw_data)(int *sar_value);
	int vender_div;
};


struct sar_data {
	struct hwm_sensor_data sar_data;
	int data_updata;
};

struct sar_context {
	struct input_dev   *idev;
	struct sensor_attr_t sar_mdev;
	struct work_struct  report_sar;
	struct mutex sar_op_mutex;
	struct timer_list		timer_sar;  /* sar polling timer */
	atomic_t            delay_sar; /*polling period for reporting input event*/
	atomic_t            wake;  /*user-space request to wake-up, used with stop*/
	struct hrtimer		hrTimer;
	ktime_t			target_ktime;
	atomic_t            trace;
	atomic_t                early_suspend;
	
	struct sar_data	drv_data;
	struct sar_data_path sar_data;
	struct sar_control_path sar_ctl;
	
	
	bool			is_sar_active_nodata;
	bool			is_sar_active_data;		/* Active and HAL need data . */
	bool is_sar_first_data_after_enable;
	bool is_get_valid_sar_data_after_enable;
	bool is_sar_polling_run;
	bool is_sar_batch_enable;	/* version2.this is used for judging whether sensor is in batch mode */
	int sar_power;
	int sar_enable;
	int64_t sar_delay_ns;
	int64_t sar_latency_ns;
};


/* for auto detect */
extern int sar_driver_add(struct sar_init_info *obj);
extern int sar_report_interrupt_data(int value);
extern int sar_flush_report(void);
extern int sar_data_report(int value, int status);
extern int sar_register_data_path(struct sar_data_path *data);
extern int sar_register_control_path(struct sar_control_path *ctl);
extern struct platform_device *get_sar_platformdev(void);


