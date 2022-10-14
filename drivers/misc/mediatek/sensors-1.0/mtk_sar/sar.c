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

#include <linux/vmalloc.h>
#include "inc/sar.h"
#include "sensor_performance.h"

struct sar_context *sar_context_obj/* = NULL*/;

static struct sar_init_info *sar_sensor_init_list[MAX_CHOOSE_SAR_NUM] = { 0 };


int sar_data_report(int value, int status)
{
	int err = 0;
	struct sensor_event event;
	memset(&event, 0, sizeof(struct sensor_event));

	SAR_LOG("[sar]sar_data_report! %d, %d\n", value, status);
	event.flush_action = DATA_ACTION;
	event.word[0] = value + 1;
	event.status = status;
	err = sensor_input_event(sar_context_obj->sar_mdev.minor, &event);
	if (err < 0)
		SAR_PR_ERR("event buffer full, so drop this data\n");
	return err;
}

int sar_flush_report(void)
{
	struct sensor_event event;
	int err = 0;
	memset(&event, 0, sizeof(struct sensor_event));

	event.flush_action = FLUSH_ACTION;
	err = sensor_input_event(sar_context_obj->sar_mdev.minor, &event);
	if (err < 0)
		SAR_PR_ERR("event buffer full, so drop this data\n");
	else
		SAR_LOG("flush\n");
	return err;
}


static void sar_work_func(struct work_struct *work)
{

	struct sar_context *cxt = NULL;
	int value, status;
	int64_t  nt;
	struct timespec time;
	int err = 0;

	cxt  = sar_context_obj;
	if (cxt->sar_data.get_data == NULL) {
		SAR_PR_ERR("sar driver not register data path\n");
		return;
	}

	time.tv_sec = time.tv_nsec = 0;
	time = get_monotonic_coarse();
	nt = time.tv_sec*1000000000LL+time.tv_nsec;

	/* add wake lock to make sure data can be read before system suspend */
	err = cxt->sar_data.get_data(&value, &status);
	if (err) {
		SAR_PR_ERR("get sar data fails!!\n");
		return;
	} else {
		cxt->drv_data.sar_data.values[0] = value;
		cxt->drv_data.sar_data.status = status;
		cxt->drv_data.sar_data.time = nt;
	}

	if (true ==  cxt->is_sar_first_data_after_enable) {
		cxt->is_sar_first_data_after_enable = false;
		/* filter -1 value */
		if (cxt->drv_data.sar_data.values[0] == SAR_INVALID_VALUE) {
			SAR_LOG(" read invalid data\n");
			goto sar_loop;
		}
	}

	if (cxt->is_get_valid_sar_data_after_enable == false) {
		if (cxt->drv_data.sar_data.values[0] != SAR_INVALID_VALUE)
			cxt->is_get_valid_sar_data_after_enable = true;
	}

	sar_data_report(cxt->drv_data.sar_data.values[0],
	cxt->drv_data.sar_data.status);
	
sar_loop:
	if (true == cxt->is_sar_polling_run) {
		if (cxt->sar_ctl.is_polling_mode || (cxt->is_get_valid_sar_data_after_enable == false))
			mod_timer(&cxt->timer_sar, jiffies + atomic_read(&cxt->delay_sar)/(1000/HZ));
	}
}

static void sarsensor_poll(unsigned long data)
{
	struct sar_context *obj = (struct sar_context *)data;

	if (obj != NULL)
		schedule_work(&obj->report_sar);
}

static struct sar_context *sar_context_alloc_object(void)
{
	struct sar_context *obj = kzalloc(sizeof(*obj), GFP_KERNEL);

	SAR_LOG("sar_context_alloc_object++++\n");
	if (!obj) {
		SAR_PR_ERR("Alloc sar object error!\n");
		return NULL;
	}
	atomic_set(&obj->delay_sar, 200);	/*5Hz ,  set work queue delay time 200ms */
	atomic_set(&obj->wake, 0);

	INIT_WORK(&obj->report_sar, sar_work_func);
	init_timer(&obj->timer_sar);
	obj->timer_sar.expires	= jiffies + atomic_read(&obj->delay_sar)/(1000/HZ);
	obj->timer_sar.function	= sarsensor_poll;
	obj->timer_sar.data	= (unsigned long)obj;

	obj->is_sar_active_nodata = false;
	obj->is_sar_active_data = false;
	obj->is_sar_first_data_after_enable = false;
	obj->is_sar_polling_run = false;
	mutex_init(&obj->sar_op_mutex);
	obj->is_sar_batch_enable = false;/* for batch mode init */
	obj->sar_power = 0;
	obj->sar_enable = 0;
	obj->sar_delay_ns = -1;
	obj->sar_latency_ns = -1;
	SAR_LOG("acc_context_alloc_object----\n");
	return obj;
}


/*----------------------------------------------------------------------------*/

static int sar_enable_and_batch(void)
{
	struct sar_context *cxt = sar_context_obj;
	int err;

	/* sar_power on -> power off */
	if (cxt->sar_power == 1 && cxt->sar_enable == 0) {
		SAR_LOG("sar disable\n");
		err = cxt->sar_ctl.enable_nodata(0);
		if (err) {
			SAR_PR_ERR("sar turn off sar_power err = %d\n", err);
			return -1;
		}
		SAR_LOG("sar turn off sar_power done\n");

		cxt->sar_power = 0;
		cxt->sar_delay_ns = -1;
		SAR_LOG("sar disable done\n");
		return 0;
	}
	/* sar_power off -> power on */
	if (cxt->sar_power == 0 && cxt->sar_enable == 1) {
		SAR_LOG("sar_power on\n");
		err = cxt->sar_ctl.enable_nodata(1);
		if (err) {
			SAR_PR_ERR("sar turn on sar_power err = %d\n", err);
			return -1;
		}
		SAR_LOG("sar turn on sar_power done\n");

		cxt->sar_power = 1;
		SAR_LOG("sar_power on done\n");
	}
	/* rate change */
/*	if (cxt->sar_power == 1 && cxt->sar_delay_ns >= 0) {
		SAR_LOG("sar set batch\n");
		// set ODR, fifo timeout latency
		err = cxt->sar_ctl.batch(0, cxt->sar_delay_ns, 0);
		if (err) {
			SAR_PR_ERR("sar set batch(ODR) err %d\n", err);
			return -1;
		}
		SAR_LOG("ps set ODR, fifo latency done\n");
		// start polling, if needed

		sar_data_report(0,3);
		SAR_LOG("sar batch done\n");
	}
*/
	return 0;
}

static ssize_t sar_store_active(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sar_context *cxt = sar_context_obj;
	int err = 0;

	SAR_LOG("sar_store_active buf=%s\n", buf);
	mutex_lock(&sar_context_obj->sar_op_mutex);

	if (!strncmp(buf, "1", 1))
		cxt->sar_enable = 1;
	else if (!strncmp(buf, "0", 1))
		cxt->sar_enable = 0;
	else {
		SAR_PR_ERR(" sar_store_active error !!\n");
		err = -1;
		goto err_out;
	}
	err = sar_enable_and_batch();

err_out:
	mutex_unlock(&sar_context_obj->sar_op_mutex);
	SAR_LOG(" sar_store_active done\n");
	return count;
}

static ssize_t sar_show_active(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sar_context *cxt = sar_context_obj;
	int div = 0;

	div = cxt->sar_data.vender_div;
	SAR_LOG("sar vender_div value: %d\n", div);
	return snprintf(buf, PAGE_SIZE, "%d\n", div);
}

static ssize_t sar_store_batch(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sar_context *cxt = sar_context_obj;
	int handle = 0, flag = 0, err = 0;

	SAR_LOG("sar_store_batch %s\n", buf);
	err = sscanf(buf, "%d,%d,%lld,%lld", &handle, &flag,
			&cxt->sar_delay_ns, &cxt->sar_latency_ns);
	if (err != 4)
		SAR_PR_ERR("sar_store_batch param error: err = %d\n", err);

	mutex_lock(&sar_context_obj->sar_op_mutex);
	err = cxt->sar_ctl.batch(0, cxt->sar_delay_ns, 0);
	
	err = sar_enable_and_batch(); ////////////////////////////////
	
	mutex_unlock(&sar_context_obj->sar_op_mutex);
	SAR_LOG("sar_store_batch done: %d\n", cxt->is_sar_batch_enable);
	return count;
}

static ssize_t sar_show_batch(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t sar_store_flush(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sar_context *cxt = NULL;
	int handle = 0, err = 0;

	err = kstrtoint(buf, 10, &handle);
	if (err != 0)
		SAR_PR_ERR("sar_store_flush param error: err = %d\n", err);

	SAR_PR_ERR("ps_store_flush param: handle %d\n", handle);

	mutex_lock(&sar_context_obj->sar_op_mutex);
	cxt = sar_context_obj;
	if (cxt->sar_ctl.flush != NULL)
		err = cxt->sar_ctl.flush();
	else
		SAR_PR_ERR(" SAR DRIVER OLD ARCHITECTURE DON'T SUPPORT SAR COMMON VERSION FLUSH\n");
	if (err < 0)
		SAR_PR_ERR("sar enable flush err %d\n", err);
	mutex_unlock(&sar_context_obj->sar_op_mutex);
	if (err)
		return err;
	else
		return count;
}


static ssize_t sar_show_flush(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}
/* need work around again */
static ssize_t sar_show_devnum(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}


static int sar_sensor_remove(struct platform_device *pdev)
{
	SAR_LOG("sar_sensor_remove\n");
	return 0;
}

static int sar_sensor_probe(struct platform_device *pdev)
{
	SAR_LOG("sar_sensor_probe\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sar_sensor_of_match[] = {
	{ .compatible = "mediatek,sar", },
	{},
};
#endif
static struct platform_driver sar_sensor_driver = {
	.probe = sar_sensor_probe,
	.remove = sar_sensor_remove,
	.driver = {
		   .name = "sar",
	#ifdef CONFIG_OF
		.of_match_table = sar_sensor_of_match,
		#endif
		   }
};

static int sar_real_driver_init(void)
{
	int i = 0;
	int err = 0;

	SAR_LOG(" sar_real_driver_init +\n");
	for (i = 0; i < MAX_CHOOSE_SAR_NUM; i++) {
		SAR_LOG(" i=%d\n", i);
		if (sar_sensor_init_list[i] != 0) {
			SAR_LOG(" sar try to init driver %s\n", sar_sensor_init_list[i]->name);
			err = sar_sensor_init_list[i]->init();
			if (err == 0) {
				SAR_LOG(" sar real driver %s probe ok\n",
					sar_sensor_init_list[i]->name);
				break;
			}
		}
	}

	if (i == MAX_CHOOSE_SAR_NUM) {
		SAR_LOG(" sar_real_driver_init fail\n");
		err = -1;
	}
	return err;
}

int sar_driver_add(struct sar_init_info *obj)
{
	int err = 0;
	int i = 0;

	if (!obj) {
		SAR_PR_ERR("SAR driver add fail, sar_init_info is NULL\n");
		return -1;
	}
	for (i = 0; i < MAX_CHOOSE_SAR_NUM; i++) {
		if ((i == 0) && (sar_sensor_init_list[0] == NULL)) {
			SAR_LOG("register sar_sensor driver for the first time\n");
			if (platform_driver_register(&sar_sensor_driver))
				SAR_PR_ERR("failed to register gensor driver already exist\n");
		}

		if (sar_sensor_init_list[i] == NULL) {
			obj->platform_diver_addr = &sar_sensor_driver;
			sar_sensor_init_list[i] = obj;
			break;
		}
	}
	if (i >= MAX_CHOOSE_SAR_NUM) {
		SAR_PR_ERR("SAR driver add err\n");
		err = -1;
	}

	return err;
}
EXPORT_SYMBOL_GPL(sar_driver_add);


int sar_report_interrupt_data(int value)
{
	struct sar_context *cxt = NULL;
	/* int err =0; */
	cxt = sar_context_obj;
	SAR_LOG("[%s]:value=%d\n", __func__, value);
	if (cxt->is_get_valid_sar_data_after_enable == false) {
		if (value != SAR_INVALID_VALUE) {
			cxt->is_get_valid_sar_data_after_enable = true;
			smp_mb();/*for memory barriier*/
			del_timer_sync(&cxt->timer_sar);
			smp_mb();/*for memory barriier*/
			cancel_work_sync(&cxt->report_sar);
		}
	}

	if (cxt->is_sar_batch_enable == false)
		sar_data_report(value, 3);

	return 0;
}
EXPORT_SYMBOL_GPL(sar_report_interrupt_data);

DEVICE_ATTR(saractive, S_IWUSR | S_IRUGO, sar_show_active, sar_store_active);
DEVICE_ATTR(sarbatch,		S_IWUSR | S_IRUGO, sar_show_batch, sar_store_batch);
DEVICE_ATTR(sarflush,		S_IWUSR | S_IRUGO, sar_show_flush, sar_store_flush);
DEVICE_ATTR(sardevnum,		S_IWUSR | S_IRUGO, sar_show_devnum,  NULL);

static struct attribute *sar_attributes[] = {
	&dev_attr_saractive.attr,
	&dev_attr_sarbatch.attr,
	&dev_attr_sarflush.attr,
	&dev_attr_sardevnum.attr,
	NULL
};

static struct attribute_group sar_attribute_group = {
	.attrs = sar_attributes
};


static int sar_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	return 0;
}

static ssize_t sar_read(struct file *file, char __user *buffer,
			  size_t count, loff_t *ppos)
{
	ssize_t read_cnt = 0;

	read_cnt = sensor_event_read(sar_context_obj->sar_mdev.minor, file, buffer, count, ppos);

	return read_cnt;
}

static unsigned int sar_poll(struct file *file, poll_table *wait)
{
	return sensor_event_poll(sar_context_obj->sar_mdev.minor, file, wait);
}

static const struct file_operations sar_fops = {
	.owner = THIS_MODULE,
	.open = sar_open,
	.read = sar_read,
	.poll = sar_poll,
};


static int sar_misc_init(struct sar_context *cxt)
{
	int err = 0;

	cxt->sar_mdev.minor = ID_CAP_PROX;
	cxt->sar_mdev.name  = CAP_MISC_DEV_NAME;
	cxt->sar_mdev.fops = &sar_fops;
	err = sensor_attr_register(&cxt->sar_mdev);
	if (err)
		SAR_PR_ERR("unable to register sar misc device!!\n");

	return err;
}


int sar_register_data_path(struct sar_data_path *data)
{
	struct sar_context *cxt = NULL;
/* int err =0; */
	cxt = sar_context_obj;
	cxt->sar_data.get_data = data->get_data;
	cxt->sar_data.vender_div = data->vender_div;
	cxt->sar_data.sar_get_raw_data = data->sar_get_raw_data;
	SAR_LOG("alsps register data path vender_div: %d\n", cxt->sar_data.vender_div);
	if (cxt->sar_data.get_data == NULL) {
		SAR_LOG("sar register data path fail\n");
		return -1;
	}
	return 0;
}


int sar_register_control_path(struct sar_control_path *ctl)
{
	struct sar_context *cxt = NULL;
	int err = 0;

	cxt = sar_context_obj;
	cxt->sar_ctl.set_delay = ctl->set_delay;
	cxt->sar_ctl.open_report_data = ctl->open_report_data;
	cxt->sar_ctl.enable_nodata = ctl->enable_nodata;
	cxt->sar_ctl.batch = ctl->batch;
	cxt->sar_ctl.flush = ctl->flush;
	cxt->sar_ctl.is_support_batch = ctl->is_support_batch;
	cxt->sar_ctl.is_polling_mode = ctl->is_polling_mode;

	if (cxt->sar_ctl.enable_nodata == NULL || cxt->sar_ctl.batch == NULL ||
			cxt->sar_ctl.flush == NULL) {
		SAR_LOG("sar register control path fail\n");
		return -1;
	}

	err = sar_misc_init(sar_context_obj);
	if (err) {
		SAR_PR_ERR("unable to register sar misc device!!\n");
		return -2;
	}
	err = sysfs_create_group(&sar_context_obj->sar_mdev.this_device->kobj,
			&sar_attribute_group);
	if (err < 0) {
		SAR_PR_ERR("unable to create sar attribute file\n");
		return -3;
	}
	kobject_uevent(&sar_context_obj->sar_mdev.this_device->kobj, KOBJ_ADD);
	return 0;
}



static int sar_probe(void)
{

	int err;

	SAR_LOG("+++++++++++++sar_sensor_probe!!\n");

	sar_context_obj = sar_context_alloc_object();
	if (!sar_context_obj) {
		err = -ENOMEM;
		SAR_PR_ERR("unable to allocate devobj!\n");
		goto exit_alloc_data_failed;
	}
	/* init real acceleration driver */
	err = sar_real_driver_init();
	if (err) {
		SAR_PR_ERR("sat real driver init fail\n");
		goto real_driver_init_fail;
	}

	SAR_LOG("----sar_probe OK !!\n");
	return 0;


 real_driver_init_fail:
	kfree(sar_context_obj);
	sar_context_obj = NULL;
 exit_alloc_data_failed:

	SAR_PR_ERR("----sar_probe fail !!!\n");
	return err;
}

static int sar_remove(void)
{
	int err = 0;

	sysfs_remove_group(&sar_context_obj->sar_mdev.this_device->kobj, &sar_attribute_group);

	err = sensor_attr_deregister(&sar_context_obj->sar_mdev);
	if (err)
		SAR_PR_ERR("misc_deregister fail: %d\n", err);
	kfree(sar_context_obj);

	return 0;
}

static int __init sar_init(void)
{
	SAR_LOG("acc_init\n");

	if (sar_probe()) {
		SAR_PR_ERR("failed to register sar driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit sar_exit(void)
{
	sar_remove();
	platform_driver_unregister(&sar_sensor_driver);
}
late_initcall(sar_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SAR device driver");
MODULE_AUTHOR("Mediatek");
