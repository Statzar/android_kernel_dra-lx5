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

/*
 *
 * Filename:
 * ---------
 *    mtk_switch_charging.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of Battery charging
 *
 * Author:
 * -------
 * Wy Chuang
 *
 */
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>

#include <mt-plat/mtk_boot.h>
#include <musb_core.h>
#include "mtk_charger_intf.h"
#include "mtk_switch_charging.h"
#define DPM_CHANGE_VOLT 50
#define TO_T1_INPUT_CURRENT  100000
#define TO_T1_CHARGING_CURRENT 500000
#define T1_T2_CHARGING_CURRENT 900000
#define FIRST_STAGE_CV_VOL 4400000
#define SECOND_STAGE_CV_VOL 4380000
#define CV_SWITCH_LIMEN 4395
extern int g_basp_aging_cv;
extern int g_basp_aging_max_current;

typedef struct VBAT_AREA
{
   int min,max;
   int dpm;
   unsigned char dpm_byte;
}vbat_area;

static vbat_area vbatArea[] =
{
       {0, 3900, 4280, 0x04}, {3900, 4000, 4440, 0x04},
       {4000, 4250, 4520, 0x04}, {4250, 4590, 4600, 0x05}
};

unsigned int global_dpm = 0;
#define VBAT_AREA_MEMBER (sizeof(vbatArea)/sizeof(vbat_area))
int get_vbat_area_index(unsigned int vbat)
{
   int i = 0;
   for(; i < VBAT_AREA_MEMBER; i++)
   {
       if (vbat >= vbatArea[i].min && vbat <= vbatArea[i].max)
           return i;
   }
   return -1;
}
void __set_dpm(struct charger_device *charger_dev, int indx, int *last_indx)
{
       global_dpm = vbatArea[indx].dpm;
       *last_indx = indx;
	   charger_dev_set_dpm(charger_dev, vbatArea[indx].dpm_byte);
       return;
}
void set_dpm_proc(struct charger_device *charger_dev, unsigned int vbat, bool charger_enable)
{
   int indx;
   static unsigned int last_indx = -1;
   if(false == charger_enable)
   {
       global_dpm = 0;
       charger_dev_set_dpm(charger_dev, vbatArea[0].dpm_byte);
       return;
   }
   if ((indx = get_vbat_area_index(vbat)) == -1)
   {
       printk(KERN_ERR  "invalid vbat: %d\n", vbat);
       return;
   }
   if (0 == global_dpm || abs(last_indx - indx) > 1)
   {
       __set_dpm(charger_dev, indx, &last_indx);
       return;
   }
   if (vbatArea[indx].dpm < global_dpm)
   {
       if (vbatArea[indx].max - vbat >= DPM_CHANGE_VOLT)
       {
           __set_dpm(charger_dev, indx, &last_indx);
           return;
       }
   }
   else if (vbatArea[indx].dpm > global_dpm)
   {
       if (vbat - vbatArea[indx].min >= DPM_CHANGE_VOLT)
       {
           __set_dpm(charger_dev, indx, &last_indx);
           return;
       }
   }

}
static void _disable_all_charging(struct charger_manager *info)
{
	charger_dev_enable(info->chg1_dev, false);

	if (mtk_is_pe30_running(info)) {
		mtk_pe30_end(info);
		mtk_pe30_set_is_enable(info, true);
	}

	if (mtk_pe20_get_is_enable(info)) {
		mtk_pe20_set_is_enable(info, false);
		if (mtk_pe20_get_is_connect(info))
			mtk_pe20_reset_ta_vchr(info);
	}

	if (mtk_pe_get_is_enable(info)) {
		mtk_pe_set_is_enable(info, false);
		if (mtk_pe_get_is_connect(info))
			mtk_pe_reset_ta_vchr(info);
	}
}

extern int ontim_set_charging_current;
static void swchg_select_charging_current_limit(struct charger_manager *info)
{
	struct charger_data *pdata;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	u32 ichg1_min = 0, aicr1_min = 0;
	int ret = 0;
	int sm_current = 0;

	pdata = &info->chg1_data;
	mutex_lock(&swchgalg->ichg_aicr_access_mutex);

	/* AICL */
	if (!mtk_is_pe30_running(info) && !mtk_pe20_get_is_connect(info) &&
		!mtk_pe_get_is_connect(info))
		charger_dev_run_aicl(info->chg1_dev, &pdata->input_current_limit_by_aicl);

	if (pdata->force_charging_current > 0) {

		pdata->charging_current_limit = pdata->force_charging_current;
		if (pdata->force_charging_current <= 450000) {
			pdata->input_current_limit = 500000;
		} else {
			pdata->input_current_limit = info->data.ac_charger_input_current;
			pdata->charging_current_limit = info->data.ac_charger_current;
		}
		goto done;
	}

	if (info->usb_unlimited) {
		pdata->input_current_limit = info->data.ac_charger_input_current;
		pdata->charging_current_limit = info->data.ac_charger_current;
		goto done;
	}

	if ((get_boot_mode() == META_BOOT) || ((get_boot_mode() == ADVMETA_BOOT))) {
		pdata->input_current_limit = 200000; /* 200mA */
		goto done;
	}

	if (mtk_is_TA_support_pd_pps(info)) {
		pdata->input_current_limit = info->data.pe40_single_charger_input_current;
		pdata->charging_current_limit = info->data.pe40_single_charger_current;
	} else if (is_typec_adapter(info)) {

		if (tcpm_inquire_typec_remote_rp_curr(info->tcpc) == 3000) {
			pdata->input_current_limit = 3000000;
			pdata->charging_current_limit = 3000000;
		} else if (tcpm_inquire_typec_remote_rp_curr(info->tcpc) == 1500) {
			pdata->input_current_limit = 1500000;
			pdata->charging_current_limit = 2000000;
		} else {
			/* for TYPEC_CC_VOLT_SNK_DFT */
			pdata->input_current_limit = 500000;
			pdata->charging_current_limit = 500000;
		}

		chr_err("type-C:%d current:%d\n",
			info->pd_type, tcpm_inquire_typec_remote_rp_curr(info->tcpc));
	} else if (mtk_pdc_check_charger(info) == true) {
		int vbus = 0, cur = 0, idx = 0;

		mtk_pdc_get_setting(info, &vbus, &cur, &idx);
		if (idx != -1) {
		pdata->input_current_limit = cur * 1000;
		pdata->charging_current_limit = info->data.pd_charger_current;
			mtk_pdc_setup(info, idx);
		} else {
			pdata->input_current_limit = info->data.usb_charger_current_configured;
			pdata->charging_current_limit = info->data.usb_charger_current_configured;
		}
		chr_err("[%s]vbus:%d input_cur:%d idx:%d current:%d\n", __func__,
			vbus, cur, idx, info->data.pd_charger_current);

	} else if (info->chr_type == STANDARD_HOST) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE)) {
			if (info->usb_state == USB_SUSPEND)
				pdata->input_current_limit = info->data.usb_charger_current_suspend;
			else if (info->usb_state == USB_UNCONFIGURED)
				pdata->input_current_limit = info->data.usb_charger_current_unconfigured;
			else if (info->usb_state == USB_CONFIGURED)
				pdata->input_current_limit = info->data.usb_charger_current_configured;
			else
				pdata->input_current_limit = info->data.usb_charger_current_unconfigured;

			pdata->charging_current_limit = pdata->input_current_limit;
		} else {
			pdata->input_current_limit = info->data.usb_charger_current;
			pdata->charging_current_limit = info->data.usb_charger_current;	/* it can be larger */
		}
	} else if (info->chr_type == NONSTANDARD_CHARGER) {
		pdata->input_current_limit = info->data.non_std_ac_charger_current;
		pdata->charging_current_limit = info->data.non_std_ac_charger_current;
	} else if (info->chr_type == STANDARD_CHARGER) {
		pdata->input_current_limit = info->data.ac_charger_input_current;
		pdata->charging_current_limit = info->data.ac_charger_current;
		mtk_pe20_set_charging_current(info, &pdata->charging_current_limit, &pdata->input_current_limit);
		mtk_pe_set_charging_current(info, &pdata->charging_current_limit, &pdata->input_current_limit);
	} else if (info->chr_type == CHARGING_HOST) {
		pdata->input_current_limit = info->data.charging_host_charger_current;
		pdata->charging_current_limit = info->data.charging_host_charger_current;
	} else if (info->chr_type == APPLE_1_0A_CHARGER) {
		pdata->input_current_limit = info->data.apple_1_0a_charger_current;
		pdata->charging_current_limit = info->data.apple_1_0a_charger_current;
	} else if (info->chr_type == APPLE_2_1A_CHARGER) {
		pdata->input_current_limit = info->data.apple_2_1a_charger_current;
		pdata->charging_current_limit = info->data.apple_2_1a_charger_current;
	}

	if (info->enable_sw_jeita) {
		if (IS_ENABLED(CONFIG_USBIF_COMPLIANCE) && info->chr_type == STANDARD_HOST)
			pr_debug("USBIF & STAND_HOST skip current check\n");
		else {
			if (info->sw_jeita.sm == TEMP_T0_TO_T1)
			{
				pdata->input_current_limit = TO_T1_INPUT_CURRENT;
				pdata->charging_current_limit = TO_T1_CHARGING_CURRENT;
			}
			else if (info->sw_jeita.sm == TEMP_T1_TO_T2 || info->sw_jeita.sm == TEMP_T3_TO_T4)
			{
			       sm_current = T1_T2_CHARGING_CURRENT;
				pdata->input_current_limit = min(pdata->input_current_limit,sm_current);
				pdata->charging_current_limit = min(pdata->charging_current_limit,sm_current);
			}
		}
	}

	if (pdata->thermal_charging_current_limit != -1)
		if (pdata->thermal_charging_current_limit < pdata->charging_current_limit)
			pdata->charging_current_limit = pdata->thermal_charging_current_limit;

	if (pdata->thermal_input_current_limit != -1)
		if (pdata->thermal_input_current_limit < pdata->input_current_limit)
			pdata->input_current_limit = pdata->thermal_input_current_limit;
	if (ontim_set_charging_current != -1 && ontim_set_charging_current != 0 && ontim_set_charging_current != 1)
	{
		if (ontim_set_charging_current < pdata->input_current_limit)
			pdata->input_current_limit = ontim_set_charging_current;
	}
		
	if (pdata->input_current_limit_by_aicl != -1 && !mtk_is_pe30_running(info) &&
		!mtk_pe20_get_is_connect(info) && !mtk_pe_get_is_connect(info))
		if (pdata->input_current_limit_by_aicl < pdata->input_current_limit)
			pdata->input_current_limit = pdata->input_current_limit_by_aicl;
done:
	ret = charger_dev_get_min_charging_current(info->chg1_dev, &ichg1_min);
	if (ret != -ENOTSUPP && pdata->charging_current_limit < ichg1_min)
		pdata->charging_current_limit = 0;

	ret = charger_dev_get_min_input_current(info->chg1_dev, &aicr1_min);
	if (ret != -ENOTSUPP && pdata->input_current_limit < aicr1_min)
		pdata->input_current_limit = 0;

	chr_err("jeita current force:%d thermal:%d %d ontim_current:%d setting:%d %d type:%d usb_unlimited:%d usbif:%d usbsm:%d aicl:%d\n",
		pdata->force_charging_current,
		pdata->thermal_input_current_limit, pdata->thermal_charging_current_limit, ontim_set_charging_current,
		pdata->input_current_limit, pdata->charging_current_limit,
		info->chr_type, info->usb_unlimited,
		IS_ENABLED(CONFIG_USBIF_COMPLIANCE), info->usb_state,
		pdata->input_current_limit_by_aicl);

	charger_dev_set_input_current(info->chg1_dev, pdata->input_current_limit);
#if 0
	if(g_basp_aging_max_current > 0){
		pdata->charging_current_limit = min(pdata->charging_current_limit,g_basp_aging_max_current);
	}
#endif
	charger_dev_set_charging_current(info->chg1_dev, pdata->charging_current_limit);

	/* If AICR < 300mA, stop PE+/PE+20 */
	if (pdata->input_current_limit < 300000) {
		if (mtk_pe20_get_is_enable(info)) {
			mtk_pe20_set_is_enable(info, false);
			if (mtk_pe20_get_is_connect(info))
				mtk_pe20_reset_ta_vchr(info);
		}

		if (mtk_pe_get_is_enable(info)) {
			mtk_pe_set_is_enable(info, false);
			if (mtk_pe_get_is_connect(info))
				mtk_pe_reset_ta_vchr(info);
		}
	}

	/*
	 * If thermal current limit is larger than charging IC's minimum
	 * current setting, enable the charger immediately
	 */
	if (pdata->input_current_limit > aicr1_min && pdata->charging_current_limit > ichg1_min
	    && info->can_charging)
		charger_dev_enable(info->chg1_dev, true);
	mutex_unlock(&swchgalg->ichg_aicr_access_mutex);
}

bool switch_cv_flag = false;
static void swchg_select_cv(struct charger_manager *info)
{
	int constant_voltage;

	if (info->enable_sw_jeita)
		if (info->sw_jeita.cv != 0)
		{
			if (info->bat_vol < CV_SWITCH_LIMEN && switch_cv_flag == false)
				constant_voltage = FIRST_STAGE_CV_VOL;
			else
			{
				constant_voltage = SECOND_STAGE_CV_VOL;
				switch_cv_flag = true;
			}
			constant_voltage = min(constant_voltage, info->sw_jeita.cv);
#if 0		
			if((g_basp_aging_cv != -1) && (g_basp_aging_cv < constant_voltage))
			{
				 constant_voltage = g_basp_aging_cv;
			}
#endif
			charger_dev_set_constant_voltage(info->chg1_dev, constant_voltage);
			return;
		}

	/* dynamic cv*/
	constant_voltage = info->data.battery_cv;
	mtk_get_dynamic_cv(info, &constant_voltage);

	charger_dev_set_constant_voltage(info->chg1_dev, constant_voltage);
}
static void swchg_charger_init(struct charger_manager *info)
{
	charger_dev_init(info->chg1_dev);
}
static void swchg_turn_on_charging(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	bool charging_enable = true;
	unsigned int bat_vol;
	if (swchgalg->state == CHR_ERROR) {
		charging_enable = false;
		chr_err("[charger]Charger Error, turn OFF charging !\n");
	} else if ((get_boot_mode() == META_BOOT) || ((get_boot_mode() == ADVMETA_BOOT))) {
		charging_enable = false;
		info->chg1_data.input_current_limit = 200000; /* 200mA */
		charger_dev_set_input_current(info->chg1_dev, info->chg1_data.input_current_limit);
		chr_err("[charger]In meta or advanced meta mode, disable charging and set input current limit to 200mA\n");
	} else {
		swchg_charger_init(info);
		mtk_pe20_start_algorithm(info);
		mtk_pe_start_algorithm(info);
		printk(KERN_ERR "%s, %d\n", __func__, __LINE__);
		swchg_select_charging_current_limit(info);
		if (info->chg1_data.input_current_limit == 0 || info->chg1_data.charging_current_limit == 0) {
			charging_enable = false;
			chr_err("[charger]charging current is set 0mA, turn off charging !\r\n");
		} else {
			swchg_select_cv(info);
		}
	}
	bat_vol = battery_get_bat_voltage();
	set_dpm_proc(info->chg1_dev, bat_vol, charging_enable);
	charger_dev_enable(info->chg1_dev, charging_enable);
}

static int mtk_switch_charging_plug_in(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	swchgalg->state = CHR_CC;
	info->polling_interval = CHARGING_INTERVAL;
	swchgalg->disable_charging = false;
	get_monotonic_boottime(&swchgalg->charging_begin_time);
	charger_manager_notifier(info, CHARGER_NOTIFY_START_CHARGING);

	return 0;
}

static int mtk_switch_charging_plug_out(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	swchgalg->total_charging_time = 0;

	mtk_pe20_set_is_cable_out_occur(info, true);
	mtk_pe_set_is_cable_out_occur(info, true);
	mtk_pe30_plugout_reset(info);
	mtk_pdc_plugout(info);
	charger_manager_notifier(info, CHARGER_NOTIFY_STOP_CHARGING);
	return 0;
}

static int mtk_switch_charging_do_charging(struct charger_manager *info, bool en)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	chr_err("mtk_switch_charging_do_charging en:%d %s\n", en, info->algorithm_name);
	if (en) {
		swchgalg->disable_charging = false;
		swchgalg->state = CHR_CC;
		get_monotonic_boottime(&swchgalg->charging_begin_time);
		charger_manager_notifier(info, CHARGER_NOTIFY_NORMAL);
	} else {
		/* disable might change state , so first */
		_disable_all_charging(info);
		swchgalg->disable_charging = true;
		swchgalg->state = CHR_ERROR;
		charger_manager_notifier(info, CHARGER_NOTIFY_ERROR);
	}

	return 0;
}
#if 0
/* return false if total charging time exceeds max_charging_time */
static bool mtk_switch_check_charging_time(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	struct timespec time_now;

	if (info->enable_sw_safety_timer) {
		get_monotonic_boottime(&time_now);
		chr_debug("%s: begin: %ld, now: %ld\n", __func__,
			swchgalg->charging_begin_time.tv_sec, time_now.tv_sec);

		if (swchgalg->total_charging_time >= info->data.max_charging_time) {
			chr_err("%s: SW safety timeout: %d sec > %d sec\n",
				__func__, swchgalg->total_charging_time,
				info->data.max_charging_time);
			charger_dev_notify(info->chg1_dev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
			return false;
		}
	}

	return true;
}
#endif
static int mtk_switch_chr_cc(struct charger_manager *info)
{
	bool chg_done = false;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	struct timespec time_now, charging_time;

	/* check bif */
	if (IS_ENABLED(CONFIG_MTK_BIF_SUPPORT)) {
		if (pmic_is_bif_exist() != 1) {
			chr_err("CONFIG_MTK_BIF_SUPPORT but no bif , stop charging\n");
			swchgalg->state = CHR_ERROR;
			charger_manager_notifier(info, CHARGER_NOTIFY_ERROR);
		}
	}

	get_monotonic_boottime(&time_now);
	charging_time = timespec_sub(time_now, swchgalg->charging_begin_time);

	swchgalg->total_charging_time = charging_time.tv_sec;

	/* turn on LED */

	swchg_turn_on_charging(info);

	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	if (chg_done) {
		if (info->bat_vol < 4300 && info->sw_jeita.sm != TEMP_T2_TO_T3)
			swchgalg->state = CHR_CC;
		else
		{
			swchgalg->state = CHR_BATFULL;
			charger_dev_do_event(info->chg1_dev, EVENT_EOC, 0);
			chr_err("battery full!\n");
		}
	}
	/* If it is not disabled by throttling,
	 * enable PE+/PE+20, if it is disabled
	 */
	if (info->chg1_data.thermal_input_current_limit != -1 &&
		info->chg1_data.thermal_input_current_limit < 300)
		return 0;

	if (!mtk_pe20_get_is_enable(info)) {
		mtk_pe20_set_is_enable(info, true);
		mtk_pe20_set_to_check_chr_type(info, true);
	}

	if (!mtk_pe_get_is_enable(info)) {
		mtk_pe_set_is_enable(info, true);
		mtk_pe_set_to_check_chr_type(info, true);
	}
	return 0;
}

int mtk_switch_chr_err(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	if (info->enable_sw_jeita) {
		if ((info->sw_jeita.sm == TEMP_BELOW_T0) || (info->sw_jeita.sm == TEMP_ABOVE_T4))
			info->sw_jeita.error_recovery_flag = false;

		if ((info->sw_jeita.error_recovery_flag == false) &&
			(info->sw_jeita.sm != TEMP_BELOW_T0) && (info->sw_jeita.sm != TEMP_ABOVE_T4)) {
			info->sw_jeita.error_recovery_flag = true;
			swchgalg->state = CHR_CC;
			get_monotonic_boottime(&swchgalg->charging_begin_time);
		}
	}

	swchgalg->total_charging_time = 0;

	_disable_all_charging(info);
	return 0;
}

int mtk_switch_chr_full(struct charger_manager *info)
{
	bool chg_done = false;
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	swchgalg->total_charging_time = 0;

	/* turn off LED */

	/*
	 * If CV is set to lower value by JEITA,
	 * Reset CV to normal value if temperture is in normal zone
	 */
	swchg_select_cv(info);
	info->polling_interval = CHARGING_FULL_INTERVAL;
	charger_dev_is_charging_done(info->chg1_dev, &chg_done);
	if (!chg_done) {
		swchgalg->state = CHR_CC;
		charger_dev_do_event(info->chg1_dev, EVENT_RECHARGE, 0);
		mtk_pe20_set_to_check_chr_type(info, true);
		mtk_pe_set_to_check_chr_type(info, true);
		info->enable_dynamic_cv = true;
		get_monotonic_boottime(&swchgalg->charging_begin_time);
		chr_err("battery recharging!\n");
		info->polling_interval = CHARGING_INTERVAL;
		charger_dev_enable(info->chg1_dev, false);
		charger_dev_set_hz_mode(info->chg1_dev, true);
		charger_dev_enable(info->chg1_dev, true);
		charger_dev_dump_registers(info->chg1_dev);
	}

	return 0;
}

int mtk_switch_pe30(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;

	if (mtk_is_pe30_running(info) == false)
		swchgalg->state = CHR_CC;

	return 0;
}

static int mtk_switch_charging_current(struct charger_manager *info)
{
	swchg_select_charging_current_limit(info);
	return 0;
}

static int mtk_switch_charging_run(struct charger_manager *info)
{
	struct switch_charging_alg_data *swchgalg = info->algorithm_data;
	int ret = 0;

	chr_err("%s [%d], timer=%d\n", __func__, swchgalg->state,
		swchgalg->total_charging_time);

	if (mtk_pe30_check_charger(info) == true)
		swchgalg->state = CHR_PE30;

	if (mtk_is_TA_support_pe30(info) == false) {
		mtk_pe20_check_charger(info);
		mtk_pe_check_charger(info);
	}

	switch (swchgalg->state) {
	case CHR_CC:
		ret = mtk_switch_chr_cc(info);
		break;

	case CHR_BATFULL:
		ret = mtk_switch_chr_full(info);
		break;

	case CHR_ERROR:
		ret = mtk_switch_chr_err(info);
		break;

	case CHR_PE30:
		ret = mtk_switch_pe30(info);
		break;
	}

	//mtk_switch_check_charging_time(info);

	charger_dev_dump_registers(info->chg1_dev);
	return 0;
}

int charger_dev_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct charger_manager *info = container_of(nb, struct charger_manager, chg1_nb);
	struct chgdev_notify *data = v;

	pr_err_ratelimited("%s %ld", __func__, event);

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		charger_manager_notifier(info, CHARGER_NOTIFY_EOC);
		pr_info("%s: end of charge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		charger_manager_notifier(info, CHARGER_NOTIFY_START_CHARGING);
		pr_info("%s: recharge\n", __func__);
		break;
	case CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT:
		info->safety_timeout = true;
		chr_err("%s: safety timer timeout\n", __func__);

		/* If sw safety timer timeout, do not wake up charger thread */
		if (info->enable_sw_safety_timer)
			return NOTIFY_DONE;
		break;
	case CHARGER_DEV_NOTIFY_VBUS_OVP:
		info->vbusov_stat = data->vbusov_stat;
		chr_err("%s: vbus ovp = %d\n", __func__, info->vbusov_stat);
		break;
	default:
		return NOTIFY_DONE;
	}

	if (info->chg1_dev->is_polling_mode == false)
		_wake_up_charger(info);

	return NOTIFY_DONE;
}


int mtk_switch_charging_init(struct charger_manager *info)
{
	struct switch_charging_alg_data *swch_alg;

	swch_alg = devm_kzalloc(&info->pdev->dev, sizeof(struct switch_charging_alg_data), GFP_KERNEL);
	if (!swch_alg)
		return -ENOMEM;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("Found primary charger [%s]\n", info->chg1_dev->props.alias_name);
	else
		chr_err("*** Error : can't find primary charger [%s]***\n", "primary_chg");

	mutex_init(&swch_alg->ichg_aicr_access_mutex);

	info->algorithm_data = swch_alg;
	info->do_algorithm = mtk_switch_charging_run;
	info->plug_in = mtk_switch_charging_plug_in;
	info->plug_out = mtk_switch_charging_plug_out;
	info->do_charging = mtk_switch_charging_do_charging;
	info->do_event = charger_dev_event;
	info->change_current_setting = mtk_switch_charging_current;

	return 0;
}




