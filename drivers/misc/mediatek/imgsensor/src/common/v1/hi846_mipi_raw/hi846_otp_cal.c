/*
 * Driver for CAM_CAL
 *
 *
 */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include <linux/videodev2.h>
#include <asm/atomic.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor_errcode.h"
#include <linux/slab.h>
#include "hi846_mipi_raw_Sensor.h"
#include <linux/module.h>

#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB pr_err
#else
#define CAM_CALDB(x,...)
#endif

#define PFX "hi846_otp_cal"
#define LOG_INF(format, args...)    printk(PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
extern struct HI846_otp_struct HI846_otp;

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRIVER_CLASS_NAME "CAM_CALdrv"
#define CAM_CAL_DEV_MAJOR_NUMBER 226

#define CAM_CAL_DRVNAME "CAM_CAL_DRV1"
/*******************************************************************************
*
********************************************************************************/

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    struct stCAM_CAL_INFO_STRUCT *ptempbuf;

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof( struct stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[HI846_OTP] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof( struct stCAM_CAL_INFO_STRUCT)))
            {
                //get input structure address
                kfree(pBuff);
                CAM_CALDB("[HI846_OTP] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = ( struct stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[HI846_OTP] ioctl allocate mem failed\n");
        return -ENOMEM;
    }

    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[HI846_OTP] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            //CAM_CALDB("[HI846_OTP] Write CMD \n");

            break;
        case CAM_CALIOC_G_READ:
            //CAM_CALDB("[HI846_OTP] Read CMD \n");

            //CAM_CALDB("chenqiang cpy otp data,offset:%x,length %d\n",ptempbuf->u4Offset,ptempbuf->u4Length);
            if(ptempbuf->u4Length == 1)
                memcpy(pWorkingBuff, &HI846_otp.module_integrator_id, ptempbuf->u4Length);
            else if((ptempbuf->u4Length == 8))
                memcpy(pWorkingBuff, HI846_otp.wb_data, ptempbuf->u4Length);
            else if(ptempbuf->u4Length == 4)
                memcpy(pWorkingBuff, HI846_otp.af_data, ptempbuf->u4Length);
            else if(ptempbuf->u4Length == 1868)
                memcpy(pWorkingBuff, HI846_otp.lsc_data, ptempbuf->u4Length);
            else{
                //pr_err("hi846_otp_cal unknow ptempbuf->u4Length = %d\n", ptempbuf->u4Length);
                return -EFAULT;
             }
             break;
        default :
             //CAM_CALDB("[HI846_OTP] No CMD \n");
             i4RetValue = -EPERM;
             break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        //CAM_CALDB("[HI846_OTP] to user length %d \n", ptempbuf->u4Length);
        //CAM_CALDB("[HI846_OTP] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[HI846_OTP] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}

//Gionee <litao> <2014-01-07> add imx214 otp begin
#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            struct stCAM_CAL_INFO_STRUCT __user *data)
{
    //compat_uptr_t p;
    compat_uint_t i;
    int err;
    if(data == NULL)
    {
        return -EFAULT;
    }
    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not unchange */

    return err;
}
static int compat_get_cal_info_struct(
            struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            struct stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;
    if(data == NULL)
    {
        return -EFAULT;
    }
    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);
    return 0;
}

static long CAM_CAL_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret=0;
    struct COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    struct stCAM_CAL_INFO_STRUCT __user *data;
    int err;
    //CAM_CALDB("[CAMERA SENSOR] cat24c16_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );
    if(filp == NULL)
    {
    return -EFAULT;
    }
    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALDB("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}

#endif
//Gionee <litao> <2014-01-07> add imx214 otp end

static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[HI846_OTP] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        CAM_CALDB("[HI846_OTP] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl,
    //Gionee <litao> <2014-01-07> add imx214 otp begin
#ifdef CONFIG_COMPAT
    .compat_ioctl = CAM_CAL_Ioctl_Compat,
#endif
    //Gionee <litao> <2014-01-07> add imx214 otp end
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

    #if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[HI846_OTP] Allocate device no failed\n");

        return -EAGAIN;
    }
    #else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[HI846_OTP] Register device no failed\n");

        return -EAGAIN;
    }
    #endif

    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[HI846_OTP] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }
    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[HI846_OTP] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, CAM_CAL_DRIVER_CLASS_NAME);
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    //LOG_INF("CAM_CAL_probe enter zhaocq\n");
    int i4RetValue = 0;

    i4RetValue = RegisterCAM_CALCharDrv();
    return i4RetValue;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    UnregisterCAM_CALCharDrv();

    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe      = CAM_CAL_probe,
    .remove = CAM_CAL_remove,
    .driver     = {
        .name   = CAM_CAL_DRVNAME,
        .owner  = THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};


static int __init OFILM_CAM_CAL_i2C_init(void)
{
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register HI846_OTP driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register HI846_OTP driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit OFILM_CAM_CAL_i2C_exit(void)
{
    platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(OFILM_CAM_CAL_i2C_init);
module_exit(OFILM_CAM_CAL_i2C_exit);
