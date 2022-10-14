/*
 * Huawei Touchscreen Driver
 *
 * Copyright (C) 2018 Huawei Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */
#include "huawei_ts_kit.h"
#include "alsps.h"
#include "cust_alsps.h"

extern struct ts_kit_platform_data g_ts_kit_platform_data;

// only enabled but not report inputEvent to HAL
static int ts_kit_ps_enable_nodata(int en)
{
    int ret = 0;
    struct ts_proximity_info *info = &g_ts_kit_platform_data.feature_info.proximity_info;

    TS_LOG_INFO("ts kit proximity enable value = %d\n", en);

    if (g_ts_kit_platform_data.chip_data->ops->chip_proximity_enable) {
        ret = g_ts_kit_platform_data.chip_data->ops->chip_proximity_enable(en, info);
    }

    return ret;
}

static int ts_kit_ps_open_report_data(int open)
{
    return 0;
}

static int ts_kit_ps_set_delay(u64 ns)
{
    return 0;
}

static int ts_kit_ps_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
    return 0;
}

static int ts_kit_ps_flush(void)
{
    int err = 0;

    err = ps_flush_report();
    TS_LOG_INFO("add for modify: flush complete \n");
    return err;
}

static int ts_kit_ps_open(struct inode *inode, struct file *file)
{
    return nonseekable_open(inode, file);
}

static int ts_kit_ps_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static long ts_kit_ps_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    struct ts_proximity_info *info = &g_ts_kit_platform_data.feature_info.proximity_info;
    TS_LOG_INFO("cmd= %08X\n", cmd);

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if (g_ts_kit_platform_data.chip_data->ops->chip_proximity_enable) {
                err = g_ts_kit_platform_data.chip_data->ops->chip_proximity_enable(enable, info);
            }
            break;

        case ALSPS_GET_PS_MODE:
            enable = info->proximity_enable;
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
            TS_LOG_DEBUG("ALSPS_GET_PS_DATA\n");
            dat = info->proximity_status;
            if (copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            TS_LOG_DEBUG("ALSPS_GET_PS_RAW_DATA, ts kit do nothing\n");
            break;

// ----------------- factory mode test ------------------------//
        case ALSPS_GET_PS_TEST_RESULT:
            if (!info->proximity_enable) {
                goto err_out;
            }
            dat = info->proximity_status;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_IOCTL_CLR_CALI:
            break;

        case ALSPS_IOCTL_GET_CALI:
            break;

        case ALSPS_IOCTL_SET_CALI:
            break;

        case ALSPS_SET_PS_THRESHOLD:
            break;

        case ALSPS_GET_PS_THRESHOLD_HIGH:
            break;

        case ALSPS_GET_PS_THRESHOLD_LOW:
            break;
// ----------------- factory mode test ------------------------//

        default:
            err = -ENOIOCTLCMD;
            break;
    }

    err_out:
    return err;
}

#ifdef CONFIG_COMPAT
static long ts_kit_ps_compat_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    int err = 0;
    void __user *ptr64 = compat_ptr(arg);
    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd)
    {
        case COMPAT_ALSPS_SET_PS_MODE:
        case COMPAT_ALSPS_GET_PS_MODE:
        case COMPAT_ALSPS_GET_PS_DATA:
        case COMPAT_ALSPS_GET_PS_RAW_DATA:
        case COMPAT_ALSPS_SET_ALS_MODE:
        case COMPAT_ALSPS_GET_ALS_MODE:
        case COMPAT_ALSPS_GET_ALS_DATA:
        case COMPAT_ALSPS_GET_ALS_RAW_DATA:
        case COMPAT_ALSPS_GET_PS_TEST_RESULT:
        case COMPAT_ALSPS_IOCTL_CLR_CALI:
        case COMPAT_ALSPS_IOCTL_GET_CALI:
        case COMPAT_ALSPS_IOCTL_SET_CALI:
        case COMPAT_ALSPS_SET_PS_THRESHOLD:
        case COMPAT_ALSPS_GET_PS_THRESHOLD_HIGH:
        case COMPAT_ALSPS_GET_PS_THRESHOLD_LOW:
            if(ptr64 == NULL){
                err = -EINVAL;
                break;
            }
            err = file->f_op->unlocked_ioctl(file,cmd,(unsigned long)ptr64);
            if(err <0){
                TS_LOG_DEBUG("cmd %08X is failed !\n",cmd);
            }
            break;
        default:
            TS_LOG_DEBUG("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;
    }
    return err;
}
#endif

static struct file_operations ts_kit_ps_fops = {
    .owner = THIS_MODULE,
    .open = ts_kit_ps_open,
    .release = ts_kit_ps_release,
    .unlocked_ioctl = ts_kit_ps_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = ts_kit_ps_compat_ioctl,
#endif
};

static struct miscdevice ts_kit_ps_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &ts_kit_ps_fops,
};

int ts_kit_proximity_init(void)
{
    int retval = 0;
    struct ps_control_path ps_ctl = {0};

    ps_ctl.is_use_common_factory = false;
    ps_ctl.open_report_data= ts_kit_ps_open_report_data;
    ps_ctl.enable_nodata = ts_kit_ps_enable_nodata;
    ps_ctl.set_delay  = ts_kit_ps_set_delay;
    ps_ctl.is_report_input_direct = false;
    ps_ctl.batch = ts_kit_ps_batch;
    ps_ctl.flush = ts_kit_ps_flush;
#ifdef CUSTOM_KERNEL_SENSORHUB
    ps_ctl.is_support_batch = false;
#else
    ps_ctl.is_support_batch = false;
#endif

    retval = ps_register_control_path(&ps_ctl);
    if(retval)
    {
        TS_LOG_ERR("ts kit ps_register_control_path fail = %d\n", retval);
    }
    retval = misc_register(&ts_kit_ps_device);
    if(retval)
    {
        TS_LOG_ERR("ts kit ps device register failed\n");
    }

    TS_LOG_INFO("ts kit proximity init done\n");

    return retval;
}

