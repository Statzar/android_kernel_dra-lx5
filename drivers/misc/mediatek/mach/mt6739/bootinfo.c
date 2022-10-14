
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <video/mmp_disp.h>
#include "lcm_drv.h"
#include "kd_imgsensor.h"
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
//+add by hzb
//#include <mt_gpio.h>
//#include "mach/gpio_const.h"
#include "upmu_common.h"
#include <mach/upmu_sw.h>
#include <linux/delay.h>
//-add by hzb

/* < AR0009LUDL luohao lwx479740 20171026 begin */
#include "sec_boot_lib.h"
/* AR0009LUDL luohao lwx479740 20171026 end > */

extern LCM_DRIVER *lcm_kernel_detect_drv;//add by liuwei
//extern ssize_t modem_show(struct kobject *kobj, struct kobj_attribute *attr, char* buf);
#define XTOA(c)    L"0123456789ABCDEF"[c]
#define BOARD_ID_ONE  1
#define BOARD_ID_TWO  2
#define BOARD_ID_THR  3
#define BOARD_ID_FOR  4
#define BOARD_ID_FIV  5

#define SBOARD_ID_ONE  1
#define SBOARD_ID_TWO  2

char g_board_id = 0x00;
char g_sboard_id = 0x00;
char g_sdtray_val = 0x00;
static struct kobject *bootinfo_kobj = NULL;
static u8 lcd_array[]="lcd not found!";
/***add by shuxinlei  for camera info*****start*****/
static u8 back_camera_array[]="back camera not found!";
static u8 back_camera_array1[]="XinLi_HI1333!";//back_camera_id = 1
static u8 back_camera_array2[]="QiuTai_S5K3L6!";//back_camera_id = 2
static u8 back_camera_array3[]="SanYingXing_OV13855!";//back_camera_id = 3
static u8 back_camera_array4[]="XinLi_HI846!";//back_camera_id = 4
static u8 back_camera_array5[]="HeLiTai_OV8856!";//back_camera_id = 5
static u8 back_camera_array6[]="SangLaiShi_GC8034A!";//back_camera_id = 6
static u8 back_camera_array7[]="SanYingXing_GC8034H!";//back_camera_id = 7
static u8 front_camera_array[]="front camera not found!";
static u8 front_camera_array1[]="SiJiChun_HI556!";//front_camera_id = 1
static u8 front_camera_array2[]="HeLiTai_OV5675!";//front_camera_id = 2
static u8 front_camera_array3[]="SangLaiShi_GC5025A!";//front_camera_id = 3
static u8 front_camera_array4[]="SanYingXing_GC5025H!";//front_camera_id = 4
u8 back_checksum = 0xff;
u8 front_checksum = 0xff;
#define FLASHLIGHT_SIZE (3)
#define CHECKSUM_SIZE (3)
u8 back_camera_id = 0;
u8 front_camera_id = 0;
/***add by shuxinlei  for camera info*****end*****/
int touch_flash_enable=0;

int lcd_find_success=0;
bool tp_probe_ok;//bit0
bool back_camera_probe_ok;//bit1
bool front_camera_probe_ok;//bit2
bool gsensor_probe_ok;//bit3
bool proximity_probe_ok;//bit4
bool charger_probe_ok;//bit5
bool sarsensor_probe_ok;//bit6

struct pinctrl *idctrl = NULL;
struct pinctrl_state *board_control_id1 = NULL;
struct pinctrl_state *board_control_id2 = NULL;
struct pinctrl_state *board_control_id3 = NULL;
struct pinctrl_state *board_control_id4 = NULL;
struct pinctrl_state *board_control_id5 = NULL;

static ssize_t lcd_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;

	if(lcd_find_success)
		s += sprintf(s, "%s\n",lcm_kernel_detect_drv->name);
	else
		s += sprintf(s, "%s\n",lcd_array);

	return (s - buf);
}

static ssize_t lcd_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	return n;
}
static struct kobj_attribute lcd_info_attr = {
	.attr = {
		.name = "lcd_info",
		.mode = 0644,
	},
	.show =&lcd_info_show,
	.store= &lcd_info_store,
};

int retVal = 0;
static char get_sdtray_info(void)
{
	g_sdtray_val = __gpio_get_value(retVal);
	printk("g_sdtray_val = %d \n",g_sdtray_val);
	return g_sdtray_val;
}

static ssize_t hw_sd_tray_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return sprintf(buf, "%d\n", get_sdtray_info()? 1:0);
}


static struct kobj_attribute hw_sd_tray_attr = {
	.attr = {
		.name = "hw_sd_tray",
		.mode = 0444,
	},
	.show =&hw_sd_tray_show,
};

static ssize_t lcd_driving_mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
        unsigned int val;
        int res=0;

        res = kstrtouint(buf, 10, &val);
        if(res){
            printk(KERN_ERR "[kernel]:lcd_set_driving_mode kstrtouint error!\n");
            return -1;
        }

        return n;
}

static struct kobj_attribute lcd_driving_mode_set_attr = {
	.attr = {
		.name = "lcd_driving_mode_set_info",
		.mode = 0664,
	},
	.store = &lcd_driving_mode_store,
};

static ssize_t efuse_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	return sprintf(buf, "%d\n", sec_schip_enabled()? 1:0);
}

static struct kobj_attribute sboot_efuse_info_attr = {
	.attr = {
		.name = "hw_efuse_info",
		.mode = 0444,
	},
	.show =&efuse_info_show,
};

static ssize_t back_camera_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    char *s = buf;
    u8 back_sum[CHECKSUM_SIZE]={0};
    back_sum[0] = XTOA((back_checksum >> 4) & 0xf);
    back_sum[1] = XTOA(back_checksum & 0xf);
    switch(back_camera_id){
        case 1:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array1,back_sum);
            break;
        case 2:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array2,back_sum);
            break;
        case 3:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array3,back_sum);
            break;
        case 4:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array4,back_sum);
            break;
        case 5:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array5,back_sum);
            break;
        case 6:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array6,back_sum);
            break;
        case 7:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array7,back_sum);
            break;
        default:
            s += sprintf(s, "%s checksum:0x%s\n",back_camera_array,back_sum);
            break;
    }

 	return (s - buf);
}

static ssize_t back_camera_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
 	return n;
}


static struct kobj_attribute back_camera_info_attr = {
	.attr = {
        .name = "back_camera",
		.mode = 0644,
	},
	.show =&back_camera_info_show,
	.store= &back_camera_info_store,
};

static ssize_t front_camera_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
    u8 front_sum[CHECKSUM_SIZE]={0};
    front_sum[0] = XTOA((front_checksum >> 4) & 0xf);
    front_sum[1] = XTOA(front_checksum & 0xf);
	switch(front_camera_id){
		case 1:
            s += sprintf(s, "%s checksum:0x%s\n",front_camera_array1,front_sum);
			break;
		case 2:
            s += sprintf(s, "%s checksum:0x%s\n",front_camera_array2,front_sum);
			break;
		case 3:
            s += sprintf(s, "%s checksum:0x%s\n",front_camera_array3,front_sum);
			break;
		case 4:
            s += sprintf(s, "%s checksum:0x%s\n",front_camera_array4,front_sum);
			break;
		default:
            s += sprintf(s, "%s checksum:0x%s\n",front_camera_array,front_sum);
			break;
	}

 	return (s - buf);
}

static ssize_t front_camera_info_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
 	return n;
}

static struct kobj_attribute front_camera_info_attr = {
	.attr = {
        .name = "front_camera",
		.mode = 0644,
	},
	.show =&front_camera_info_show,
	.store= &front_camera_info_store,
};

static ssize_t i2c_devices_info_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char *s = buf;
//	u8 string[5]={'\0'};
	int tmp=0;
	tmp|= (tp_probe_ok<<0);
	tmp|= (front_camera_probe_ok<<1);
	tmp|= (back_camera_probe_ok<<2);
	tmp|= (gsensor_probe_ok<<3);
	tmp|= (proximity_probe_ok<<4);
	tmp|= (charger_probe_ok<<5);
	tmp|= (sarsensor_probe_ok<<6);

	//itoa((int)tmp,string);
	s += sprintf(s, "%d\n",tmp);

	return (s - buf);
}
static struct kobj_attribute i2c_devices_info_attr = {
	.attr = {
		.name = "i2c_devices_probe_info",
		.mode = 0444,
	},
	.show =&i2c_devices_info_show,
};

//+add by hzb

static struct attribute * g[] = {
	&lcd_info_attr.attr,//+add by liuwei
	&lcd_driving_mode_set_attr.attr,//+add by liuwei
	&i2c_devices_info_attr.attr,//+add by liuwei
	&back_camera_info_attr.attr,//add by shuxinlei
	&front_camera_info_attr.attr,//add by shuxinlei
	&sboot_efuse_info_attr.attr,
	&hw_sd_tray_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};



int lcm_id=0x83;
static int __init lcm_id_setup(char *str)
{
        int en;
        if(!get_option(&str, &en))
                return 0;
        lcm_id = en;
        return 1;
}
int get_lcm_id(void)
{
        printk("[kernel]:get_lcm_id=%x.\n",lcm_id);
        return lcm_id;
}
__setup("lcm_id=", lcm_id_setup);

static int mt_test_pinctrl_init(struct platform_device *pdev)
{
    int ret = 0;
    idctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(idctrl)) {
     ret = PTR_ERR(idctrl);
        printk("Cannot find testctrl!\n");
        return ret;
    }

    board_control_id1 = pinctrl_lookup_state(idctrl, "board_id1");
    if (IS_ERR(board_control_id1)) {
     ret = PTR_ERR(board_control_id1);
        printk("pinctrl err, board_control_id1 zhaohuan\n");
        return ret;
    }

    board_control_id2 = pinctrl_lookup_state(idctrl, "board_id2");
    if (IS_ERR(board_control_id2)) {
     ret = PTR_ERR(board_control_id2);
        printk("pinctrl err, board_id2 zhaohuan\n");
        return ret;
    }

    board_control_id3 = pinctrl_lookup_state(idctrl, "board_id3");
    if (IS_ERR(board_control_id3)) {
     ret = PTR_ERR(board_control_id3);
        printk("pinctrl err, board_id3 zhaohuan\n");
        return ret;
    }

    board_control_id4 = pinctrl_lookup_state(idctrl, "board_id4");
    if (IS_ERR(board_control_id4)) {
    ret = PTR_ERR(board_control_id4);
        printk("pinctrl err, board_id4 zhaohuan\n");
        return ret;
    }

    board_control_id5 = pinctrl_lookup_state(idctrl, "board_id5");
    if (IS_ERR(board_control_id5)) {
    ret = PTR_ERR(board_control_id5);
        printk("pinctrl err, board_id5 zhaohuan\n");
        return ret;
    }
    return ret;
}

void board_gpio_set(int pin)
{
   switch (pin) {
     case 1:
         pinctrl_select_state(idctrl, board_control_id1);
         break;
     case 2:
         pinctrl_select_state(idctrl, board_control_id2);
         break;
     case 3:
         pinctrl_select_state(idctrl, board_control_id3);
         break;
     case 4:
         pinctrl_select_state(idctrl, board_control_id4);
         break;
     case 5:
         pinctrl_select_state(idctrl, board_control_id5);
         break;
     default:
         printk("wrong pin number zhaohuan\n");
         break;
   }
   printk("set gpio state end zhaohuan\n");
}

int board_id_get_gpio_info(struct platform_device *pdev)
{
    struct  device_node *np = pdev->dev.of_node;
    int ret;
    char gpio_val1 = 0;
    char gpio_val2 = 0;
    char gpio_val3 = 0;
    char gpio_val4 = 0;
    char gpio_val5 = 0;
    np = of_find_node_by_name(NULL,"board_id");
    if(!np)
    {
        printk(KERN_ERR"%s find node board_id fail zhaohuan\n",__func__);
    }
    ret = of_get_named_gpio(np,"id5,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no id5 boardid5 info zhaohuan\n",__func__);
    }
    gpio_val5 = __gpio_get_value(ret);
    g_board_id = gpio_val5;
    ret = of_get_named_gpio(np,"id4,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no id4 boardid4 info\n",__func__);
    }
    gpio_val4 = __gpio_get_value(ret);
    g_board_id = g_board_id | (gpio_val4<<1);
    ret = of_get_named_gpio(np,"id3,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no id3 boardid3 info\n",__func__);
    }
    gpio_val3 = __gpio_get_value(ret);
    g_board_id = g_board_id | (gpio_val3<<2);
    ret = of_get_named_gpio(np,"id2,intr_gpio",0);
    if(ret<0)
    {
        printk("%s no id2 boardid2 info\n",__func__);
    }
    gpio_val2 = __gpio_get_value(ret);
    g_board_id = g_board_id | (gpio_val2<<3);
    ret = of_get_named_gpio(np,"id1,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no id1 boardid1 info\n",__func__);
    }
    gpio_val1 = __gpio_get_value(ret);
    g_board_id = g_board_id | (gpio_val1<<4);

    printk(KERN_ERR "g_board_id = %d, Board ID[1-5] = %1d%1d%1d%1d%1d\n",g_board_id,gpio_val1,gpio_val2,gpio_val3,gpio_val4,gpio_val5);
    return 0;
}

static ssize_t show_board_id(struct device *dev, struct device_attribute *attr,char *buf)
{
    char ret_value;
    ret_value = g_board_id;
    return sprintf(buf,"%u\n",ret_value);
}
static ssize_t store_board_id(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    return size;
}

void get_board_ID(int *board_ID)
{
    *board_ID = g_board_id;
}
static DEVICE_ATTR(board_id, 0664, show_board_id, store_board_id);

static int board_id_probe(struct platform_device *pdev)
{
    int  ret_device_file = 0;
    mt_test_pinctrl_init(pdev);
    board_gpio_set(BOARD_ID_ONE);
    board_gpio_set(BOARD_ID_TWO);
    board_gpio_set(BOARD_ID_THR);
    board_gpio_set(BOARD_ID_FOR);
    board_gpio_set(BOARD_ID_FIV);

    board_id_get_gpio_info(pdev);

    ret_device_file =device_create_file( &pdev->dev, &dev_attr_board_id);

    return 0;
}

static int board_id_remove(struct platform_device *pdev)
{
    return 0;
}

static void board_id_shutdown(struct platform_device *pdev)
{
    return ;
}

static const struct of_device_id dura_boardid[]  = {
{ .compatible = "dura_boardid", },
{}
};

static struct platform_driver board_id_driver = {
    .driver = {
           .name = "boardid",
           .owner = THIS_MODULE,
           .of_match_table = dura_boardid,
          },
    .probe = board_id_probe,
    .remove = board_id_remove,
    .shutdown = board_id_shutdown,
};
struct pinctrl *sidctrl = NULL;
struct pinctrl_state *sboard_control_id1 = NULL;
struct pinctrl_state *sboard_control_id2 = NULL;

int sboard_pinctrl_init(struct platform_device *pdev)
{
    int ret = 0;
    sidctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(sidctrl)) {
     ret = PTR_ERR(sidctrl);
        printk("Cannot find testctrl!\n");
        return ret;
    }

    sboard_control_id1 = pinctrl_lookup_state(sidctrl, "sboard_id1");
    if (IS_ERR(sboard_control_id1)) {
     ret = PTR_ERR(sboard_control_id1);
        printk("pinctrl err, sboard_id1 zhaohuan\n");
        return ret;
    }

    sboard_control_id2 = pinctrl_lookup_state(sidctrl, "sboard_id2");
    if (IS_ERR(sboard_control_id2)) {
     ret = PTR_ERR(sboard_control_id2);
        printk("pinctrl err, sboard_id2 zhaohuan\n");
        return ret;
    }
    return ret;
}
void sboard_gpio_set(int pin)
{
     switch (pin) {
     case 1:
         pinctrl_select_state(sidctrl, sboard_control_id1);
         break;
     case 2:
         pinctrl_select_state(sidctrl, sboard_control_id2);
         break;
     default:
         printk("wrong spin number zhaohuan\n");
         break;
     }
     printk("set sboard gpio state end zhaohuan\n");
}

int sboard_id_get_gpio_info(struct platform_device *pdev)
{
    struct  device_node *np = pdev->dev.of_node;
    int ret;
    char gpio_vals1 = 0;
    char gpio_vals2 = 0;
    np = of_find_node_by_name(NULL,"sboard_id");
    if(!np)
    {
        printk(KERN_ERR"%s find node sboard_id fail zhaohuan\n",__func__);
    }
    ret = of_get_named_gpio(np,"sub1,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no sub1 boardid1 info zhaohuan\n",__func__);
    }
    gpio_vals1 = __gpio_get_value(ret);
    printk("gpio_vals1 = %d zhaohuan\n",gpio_vals1);
    g_sboard_id = gpio_vals1;
    ret = of_get_named_gpio(np,"sub2,intr_gpio",0);
    if(ret<0)
    {
        printk(KERN_ERR"%s no sub2 boardid2 info\n",__func__);
    }
    gpio_vals2 = __gpio_get_value(ret);
    printk("gpio_vals2 = %d zhaohuan\n",gpio_vals2);
    g_sboard_id = g_sboard_id | (gpio_vals2<<1);

    printk("g_sboard_id = %d zhaohuan\n",g_sboard_id);
    return 0;
}
static ssize_t show_sboard_id(struct device *dev, struct device_attribute *attr,char *buf)
{
    char ret_value;
    ret_value = g_board_id;
    return sprintf(buf,"%u\n",ret_value);
}
static ssize_t store_sboard_id(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    return size;
}

static DEVICE_ATTR(sboard_id, 0664, show_sboard_id, store_sboard_id);

static int sboard_id_probe(struct platform_device *pdev)
{
    int  ret_device_file = 0;
    sboard_pinctrl_init(pdev);
    sboard_gpio_set(SBOARD_ID_ONE);
    sboard_gpio_set(SBOARD_ID_TWO);

    sboard_id_get_gpio_info(pdev);

    ret_device_file =device_create_file(&pdev->dev, &dev_attr_sboard_id);

    return 0;
}

static int sboard_id_remove(struct platform_device *pdev)
{
    return 0;
}

static void sboard_id_shutdown(struct platform_device *pdev)
{
    return ;
}


static const struct of_device_id dura_sboardid[]  = {
{ .compatible = "dura_sboardid", },
{}
};

static struct platform_driver sboard_id_driver = {
    .driver = {
          .name = "sboardid",
          .owner = THIS_MODULE,
          .of_match_table = dura_sboardid,
          },
    .probe = sboard_id_probe,
    .remove = sboard_id_remove,
    .shutdown = sboard_id_shutdown,
};


struct pinctrl *sdctrl = NULL;
struct pinctrl_state *sdtray_control_id1 = NULL;

int sdtray_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	struct  device_node *np = pdev->dev.of_node;
	sdctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sdctrl)) {
		ret = PTR_ERR(sdctrl);
		printk("Cannot find hw_sdtray!\n");
		return ret;
	}

	sdtray_control_id1 = pinctrl_lookup_state(sdctrl, "sdtray_id1");
	if (IS_ERR(sdtray_control_id1)) {
		ret = PTR_ERR(sdtray_control_id1);
		printk("pinctrl err, sdtray_control_id\n");
		return ret;
	}

	pinctrl_select_state(sdctrl, sdtray_control_id1);

	np = of_find_node_by_name(NULL,"sdtray_id");
	if(!np)
	{
		printk(KERN_ERR"%s find node sdtray_id fail\n",__func__);
		return ret;
	}
	ret = of_get_named_gpio(np,"sdtray1,intr_gpio",0);
	if(ret<0)
	{
		printk(KERN_ERR"%s no sdtray info\n",__func__);
		return ret;
	}
	retVal = ret;
	return ret;
}

static int sdtray_probe(struct platform_device *pdev)
{
	if(pdev == NULL)
	{
		return 0;
	}
	sdtray_pinctrl_init(pdev);
	return 0;
}
static int sdtray_remove(struct platform_device *pdev)
{
	return 0;
}
static void sdtray_shutdown(struct platform_device *pdev)
{
	return ;
}
static const struct of_device_id dura_sdtrayid[]  = {
{ .compatible = "dura_sdtrayid", },
{}
};

static struct platform_driver sdtray_id_driver = {
    .driver = {
          .name = "sdtray_id",
          .owner = THIS_MODULE,
          .of_match_table = dura_sdtrayid,
          },
    .probe = sdtray_probe,
    .remove = sdtray_remove,
    .shutdown = sdtray_shutdown,
};

static int __init bootinfo_init(void)
{
	int ret = -ENOMEM;

	//printk("%s,line=%d\n",__func__,__LINE__);

	bootinfo_kobj = kobject_create_and_add("ontim_bootinfo", NULL);

	if (bootinfo_kobj == NULL) {
		printk("bootinfo_init: kobject_create_and_add failed\n");
		goto fail;
	}

	ret = sysfs_create_group(bootinfo_kobj, &attr_group);
	if (ret) {
		printk("bootinfo_init: sysfs_create_group failed\n");
		goto sys_fail;
	}

	return ret;
sys_fail:
	kobject_del(bootinfo_kobj);
fail:
	return ret;

}

static int __init prjinfo_init(void)
{
    int ret;
    printk("bootinfo_init: check_hw_ver Start!\n");

    ret = platform_driver_register(&sdtray_id_driver);
    if(ret)
    {
        printk("platform_driver_register sdtray_id_driver error\n");
    }
    ret = platform_driver_register(&board_id_driver);
    if (ret)
    {
        printk("platform_driver_register error zhaohuan\n");
    }
    ret = platform_driver_register(&sboard_id_driver);
    if (ret)
    {
        printk("platform_driver_register sboard_id_driver error zhaohuan\n");
    }
    return ret;
}

static void __exit bootinfo_exit(void)
{
	platform_driver_unregister(&sdtray_id_driver);
    platform_driver_unregister(&board_id_driver);
    platform_driver_unregister(&sboard_id_driver);
    if (bootinfo_kobj) {
        sysfs_remove_group(bootinfo_kobj, &attr_group);
        kobject_del(bootinfo_kobj);
    }
}

arch_initcall(bootinfo_init);
device_initcall(prjinfo_init);
module_exit(bootinfo_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Boot information collector");
