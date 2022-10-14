/*
 * Copyright (C) 2006-2017 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Johnson Yeh
 * Maintain: Luca Hsu, Tigers Huang, Dicky Chiang
 */

/**
 *
 * @file    ilitek_drv_main.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */
#include "mstar_apknode.h"
#include "mstar_common.h"
#include "mstar_dts.h"
#include "mp_test/mp_common.h"
#include <huawei_platform/log/log_jank.h>

// ontim debug node
#include <ontim/ontim_dev_dgb.h>
static char mstp_version[]="msg28xxa ver 1.0";
char g_mstp_vendor[50]="hlt-msg28xxa";
DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version", mstp_version)
DEV_ATTR_DEFINE("vendor",g_mstp_vendor)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);


struct i2c_client *g_i2c_client = NULL;
struct ts_kit_device_data *g_mstar_dev_data;
bool mstar_suspend_flag = false;
char mstar_project_id[MSTAR_PROJECT_ID_LEN] = {0};
u8 mstar_golden_sample_version[2] = {0};
u16 mstar_vendor_id = 0;
int mstar_support_get_tp_color = 0;
extern u8 cypress_ts_kit_color[TP_COLOR_SIZE];

int MS_TS_MSG_IC_GPIO_RST = GTP_RST_PORT;   // Must set a value other than 1
int MS_TS_MSG_IC_GPIO_INT = GTP_INT_PORT;   // Must set value as 1

#ifdef CONFIG_ENABLE_ITO_MP_TEST
u32 g_mp_test = 0;
#endif

#if defined(CONFIG_ENABLE_GESTURE_DEBUG_MODE)
u8 g_gesture_wakeup_packet[GESTURE_DEBUG_MODE_PACKET_LENGTH] = { 0 };
#elif defined(CONFIG_ENABLE_GESTURE_INFORMATION_MODE)
u8 g_gesture_wakeup_packet[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH] = { 0 };
#else
u8 g_gesture_wakeup_packet[GESTURE_WAKEUP_PACKET_LENGTH] = { 0 };   // for MSG22xx/MSG28xx
#endif

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
u8 g_gesture_debug_flag = 0x00;
u8 g_gesture_debug_mode = 0x00;
u8 g_gesture_log_debug[GESTURE_DEBUG_MODE_PACKET_LENGTH] = { 0 };

struct kset *g_gesture_kset = NULL;
struct kobject *g_GestureKObj = NULL;
void mstar_gesture_open_debug(u8 gesture_flag);
void mstar_gesture_close_debug(void);
#endif

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
u32 g_gesture_log_info[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH] = { 0 };

static void mstar_gesture_convert_coordinate(u8 * pRawData, u32 * pTranX, u32 * pTranY);
static void mstar_easy_wakeup_gesture_report_coordinate(u32 * buf, u32 count);
#endif

u8 g_gesture_wakeup_flag = 0;

#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE  // support at most 64 types of gesture wakeup mode
u32 g_gesture_wakeup_mode[2] = { 0xFFFFFFFF, 0xFFFFFFFF };
#else // support at most 16 types of gesture wakeup mode
u32 g_gesture_wakeup_mode[2] = { 0x0000FFFF, 0x00000000 };
#endif

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA // for MSG28xx/MSG58xxA/ILI21xx
static u8 g_touch_packet_flag[2] = { 0 };

u16 g_fw_packet_data_addr = 0;
u16 g_fw_packet_flag_addr = 0;
u8 g_fw_support_seg = 0;
#endif

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
static u8 g_charge_plug_in = 0;
u8 g_force_update = 0;
int g_charge_enable = 1;
struct delayed_work g_charge_delayed_work;
struct workqueue_struct *g_charge_workqueue = NULL;
#endif

static spinlock_t g_irq_lock;

static int g_int_flag = 0;
static int g_int_esd_flag = 0;

u8 *g_platform_fw_inter_ver = NULL; // internal use firmware version for MStar
u32 g_platform_fw_ver[3] = { 0 };

u8 g_fw_ver_flag = 0;       // 0: old 1:new ,platform FW version V01.010.03
u8 *g_fw_cust_ver = NULL;   // customer firmware version
u8 *g_chip_num = NULL;

MpTestInfo_t mp_test_info = { 0 };

int mstar_schedule_normal(void);
DECLARE_WORK(mstar_resume_normal_work, mstar_schedule_normal);
int mstar_schedule_gesture(void);
DECLARE_WORK(mstar_resume_gesture_work, mstar_schedule_gesture);

u32 SLAVE_I2C_ID_DBBUS = (0xC4 >> 1);   //0x62
u32 SLAVE_I2C_ID_DWI2C = (0x4C >> 1);   //0x26

u16 FIRMWARE_MODE_UNKNOWN_MODE = 0xFFFF;
u16 FIRMWARE_MODE_DEMO_MODE = 0xFFFF;
u16 FIRMWARE_MODE_DEBUG_MODE = 0xFFFF;
u16 FIRMWARE_MODE_RAW_DATA_MODE = 0xFFFF;

u16 DEMO_MODE_PACKET_LENGTH = 0;
u16 DEBUG_MODE_PACKET_LENGTH = 0;
u16 MAX_TOUCH_NUM = 0;

extern u8 g_switch_mode_apk;

u8 IS_FIRMWARE_DATA_LOG_ENABLED = CONFIG_ENABLE_FIRMWARE_DATA_LOG;
u8 IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED = CONFIG_ENABLE_APK_PRINT_FIRMWARE_SPECIFIC_LOG;
u8 IS_SELF_FREQ_SCAN_ENABLED = 0;
u8 IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = 0;
u8 IS_DISABLE_ESD_PROTECTION_CHECK = 0;

int g_esd_enable = 1;
u8 g_hw_reset_drv = 0;

struct input_dev *g_input_dev = NULL;
struct mutex g_mutex;
struct mutex g_mutex_protect;
struct mutex wrong_gesture_lock;

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
extern u32 g_report_rate_enable;
extern u32 g_int_cont;
extern u32 g_vaild_touch_cont;
extern u32 g_int_report_rate;
extern u32 g_vaild_touch_report_rate;
extern struct timeval g_start_time;
#endif

u16 g_chip_type = 0;
u16 g_chip_type_ori = 0;

u8 g_demo_packet[ILI21XX_DEMO_MODE_PACKET_LENGTH] = { 0 };

MutualFirmwareInfo_t g_mutual_fw_info;

u8 g_debug_packet[MUTUAL_DEBUG_MODE_PACKET_LENGTH] = { 0 };
u8 g_log_packet[MUTUAL_DEBUG_MODE_PACKET_LENGTH] = { 0 };

u16 g_fw_mode;

u8 g_fw_update = 0x00;
u8 g_finger_touch_disable = 0;

struct kset *g_touch_kset = NULL;
struct kobject *g_touch_kobj = NULL;

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
extern int mstar_fw_update_swid_entry(void);
#endif

static s32 mstar_mutual_parse_packet(u8 * pPacket, u16 nLength, MutualTouchInfo_t * pInfo, struct ts_fingers *info);
extern s32 mstar_update_fw_cash(u8 ** szFwData, EmemType_e eEmemType);

extern u16 mstar_get_chip_type(void);
extern s32 mstar_fw_update_sdcard(const char *pFilePath, u8 mode);
extern void mstar_optimize_current_consump(void);
extern u16 mstar_get_swid(EmemType_e eEmemType);

void mstar_exit_sleep_mode(void);
void mstar_enter_sleep_mode(void);
void mstar_dev_hw_reset(void);
void mstar_esd_check(void);


u16 mstar_get_reg_16bit(u16 nAddr)
{
    u8 szTxData[3] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF };
    u8 szRxData[2] = { 0 };

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 3);
    mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &szRxData[0], 2);

    return (szRxData[1] << 8 | szRxData[0]);
}

u8 mstar_get_reg_low_byte(u16 nAddr)
{
    u8 szTxData[3] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF };
    u8 szRxData = { 0 };

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 3);
    mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &szRxData, 1);

    return (szRxData);
}

u8 mstar_get_reg_high_byte(u16 nAddr)
{
    u8 szTxData[3] = { 0x10, (nAddr >> 8) & 0xFF, (nAddr & 0xFF) + 1 };
    u8 szRxData = { 0 };

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 3);
    mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &szRxData, 1);

    return (szRxData);
}

void mstar_get_reg_xbit(u16 nAddr, u8 * pRxData, u16 nLength, u16 nMaxI2cLengthLimit)
{
    u16 nReadAddr = nAddr;
    u16 nReadSize = 0;
    u16 nLeft = nLength;
    u16 nOffset = 0;
    u8 szTxData[3] = { 0 };

    szTxData[0] = 0x10;

    mutex_lock(&g_mutex);

    while (nLeft > 0) {
        if (nLeft >= nMaxI2cLengthLimit) {
            nReadSize = nMaxI2cLengthLimit;

            szTxData[1] = (nReadAddr >> 8) & 0xFF;
            szTxData[2] = nReadAddr & 0xFF;

            mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 3);
            mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &pRxData[nOffset], nReadSize);

            nReadAddr = nReadAddr + nReadSize;  //set next read address
            nLeft = nLeft - nReadSize;
            nOffset = nOffset + nReadSize;
        } else {
            nReadSize = nLeft;

            szTxData[1] = (nReadAddr >> 8) & 0xFF;
            szTxData[2] = nReadAddr & 0xFF;

            mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 3);
            mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &pRxData[nOffset], nReadSize);

            nLeft = 0;
            nOffset = nOffset + nReadSize;
        }
    }

    mutex_unlock(&g_mutex);
}

void mstar_reg_get_xbit_write_4byte_value(u16 nAddr, u8 * pRxData, u16 nLength, u16 nMaxI2cLengthLimit)
{
    u16 nReadAddr = nAddr;
    u16 nReadSize = 0;
    u16 nLeft = nLength;
    u16 nOffset = 0;
    u8 szTxData[4] = { 0 };

    szTxData[0] = 0x10;

    mutex_lock(&g_mutex);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaReset();
#endif
#endif

    while (nLeft > 0) {
        if (nLeft >= nMaxI2cLengthLimit) {
            nReadSize = nMaxI2cLengthLimit;
            TS_LOG_DEBUG(
                "*** RegGetXBitValue# Length >= I2cMax   nReadAddr=%x, nReadSize=%d ***\n", nReadAddr,
                nReadSize);

            szTxData[2] = (nReadAddr >> 8) & 0xFF;
            szTxData[3] = nReadAddr & 0xFF;

            mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 4);
            mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &pRxData[nOffset], nReadSize);

            nReadAddr = nReadAddr + nReadSize;  //set next read address
            nLeft = nLeft - nReadSize;
            nOffset = nOffset + nReadSize;
            TS_LOG_DEBUG( "*** RegGetXBitValue# Length >= I2cMax   nLeft=%d, nOffset=%d ***\n",
                nLeft, nOffset);
        } else {
            nReadSize = nLeft;
            TS_LOG_DEBUG(
                "*** RegGetXBitValue# Length < I2cMax   nReadAddr=%x, nReadSize=%d ***\n", nReadAddr,
                nReadSize);

            szTxData[2] = (nReadAddr >> 8) & 0xFF;
            szTxData[3] = nReadAddr & 0xFF;

            mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 4);
            mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &pRxData[nOffset], nReadSize);

            nLeft = 0;
            nOffset = nOffset + nReadSize;
            TS_LOG_DEBUG( "*** RegGetXBitValue# Length < I2cMax   nLeft=%d, nOffset=%d ***\n",
                nLeft, nOffset);
        }
    }

    mutex_unlock(&g_mutex);
}

s32 mstar_set_reg_16bit(u16 nAddr, u16 nData)
{
    s32 rc = 0;
    u8 szTxData[5] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF, nData & 0xFF, nData >> 8 };

    rc = mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 5);

    return rc;
}

s32 mstar_set_reg_16bit_retry(u16 nAddr, u16 nData)
{
    s32 rc = 0;
    u32 nRetryCount = 0;

    while (nRetryCount < 5) {
        mdelay(5);
        rc = mstar_set_reg_16bit(nAddr, nData);
        if (rc > 0) {
            TS_LOG_INFO("mstar_set_reg_16bit(0x%x, 0x%x) success, rc = %d\n", nAddr, nData, rc);
            break;
        }

        nRetryCount++;
    }
    if (nRetryCount == 5) {
        TS_LOG_INFO("mstar_set_reg_16bit(0x%x, 0x%x) failed, rc = %d\n", nAddr, nData, rc);
    }

    return rc;
}

void mstar_set_reg_low_byte(u16 nAddr, u8 nData)
{
    u8 szTxData[4] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF, nData };
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 4);
}

void mstar_set_reg_hi_byte(u16 nAddr, u8 nData)
{
    u8 szTxData[4] = { 0x10, (nAddr >> 8) & 0xFF, (nAddr & 0xFF) + 1, nData };
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szTxData[0], 4);
}

void mstar_set_reg_16bit_on(u16 nAddr, u16 nData)   //set bit on nData from 0 to 1
{
    u16 rData = mstar_get_reg_16bit(nAddr);
    rData |= nData;
    mstar_set_reg_16bit(nAddr, rData);
}

void mstar_set_reg_16bit_off(u16 nAddr, u16 nData)  //set bit on nData from 1 to 0
{
    u16 rData = mstar_get_reg_16bit(nAddr);
    rData &= (~nData);
    mstar_set_reg_16bit(nAddr, rData);
}

u16 mstar_get_reg_16bit_by_addr(u16 nAddr, AddressMode_e eAddressMode)
{
    u16 nData = 0;

    if (eAddressMode == ADDRESS_MODE_16BIT) {
        nAddr = nAddr - (nAddr & 0xFF) + ((nAddr & 0xFF) << 1);
    }

    nData = mstar_get_reg_16bit(nAddr);

    return nData;
}

void mstar_set_reg_16bit_by_addr(u16 nAddr, u16 nData, AddressMode_e eAddressMode)
{
    if (eAddressMode == ADDRESS_MODE_16BIT) {
        nAddr = nAddr - (nAddr & 0xFF) + ((nAddr & 0xFF) << 1);
    }

    mstar_set_reg_16bit(nAddr, nData);
}

void mstar_reg_mask_16bit(u16 nAddr, u16 nMask, u16 nData, AddressMode_e eAddressMode)
{
    u16 nTmpData = 0;

    if (nData > nMask) {
        return;
    }

    nTmpData = mstar_get_reg_16bit_by_addr(nAddr, eAddressMode);
    nTmpData = (nTmpData & (~nMask));
    nTmpData = (nTmpData | nData);
    mstar_set_reg_16bit_by_addr(nAddr, nTmpData, eAddressMode);
}

s32 mstar_dbbus_enter_serial_debug(void)
{
    s32 rc = 0;
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    TS_LOG_INFO("mstar_dbbus_enter_serial_debug\n");
    rc = mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 5);

    return rc;
}

void mstar_dbbus_exit_serial_debug(void)
{
    u8 data[1];

    // i2c response ack
    mstar_reg_mask_16bit(0x1E4F, BIT15, BIT15, ADDRESS_MODE_16BIT);

    // Exit the Serial Debug Mode
    data[0] = 0x45;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
    TS_LOG_INFO("mstar_dbbus_exit_serial_debug\n");

    // Delay some interval to guard the next transaction
    // udelay(200);        // delay about 0.2ms
}

void mstar_dbbus_i2c_response_ack(void)
{
    // i2c response ack
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA) {
        mstar_reg_mask_16bit(0x1E4F, BIT15, BIT15, ADDRESS_MODE_16BIT);
    }
}

void mstar_dbbus_iic_use_bus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_iic_not_use_bus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_iic_reshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_stop_mcu(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_not_stop_mcu(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_reset_slave(void)
{
    u8 data[1];

    // IIC Reset Slave
    data[0] = 0x00;

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_dbbus_wait_mcu(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);

    data[0] = 0x61;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, data, 1);
}

void mstar_set_cfb(u8 Cfb)
{
    /// Setting Cfb
    switch (Cfb) {
    case _50p:      /// Cfb = 50p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0000, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0000, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x0, Rfb: 180kohm
        break;

    case _40p:      /// Cfb = 40p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0020, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0100, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x1, Rfb: 225kohm
        break;

    case _30p:      /// Cfb = 30p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0040, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0200, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;

    case _20p:      /// Cfb = 20p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0060, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0200, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;

    case _10p:      /// Cfb = 10p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0070, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0200, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;

    default:
        break;
    }
}

int tpkit_i2c_read(u8 * values, u16 values_size)
{
    int ret = 0;
    struct ts_bus_info *bops = NULL;

    bops = g_mstar_dev_data->ts_platform_data->bops;
    ret = bops->bus_read(NULL, 0, values, values_size, 3);
    if (ret) {
        TS_LOG_ERR("%s:fail, addrs:", __func__);

    }
    if (ret == 0) {
        ret = values_size;
    }
    return ret;
}

int tpkit_i2c_write(u8 * values, u16 values_size)
{
    int ret = 0;
    struct ts_bus_info *bops = NULL;
    u8 default_zero = 0;

    bops = g_mstar_dev_data->ts_platform_data->bops;
    if (values_size == 1) {
        ret = bops->bus_write(values, 1, &default_zero, 0, 3);
    } else {
        ret = bops->bus_write(values, 1, values + 1, values_size - 1, 3);
    }

    if (ret) {
        TS_LOG_ERR("%s:fail, addrs: 0x%x", __func__, g_mstar_dev_data->ts_platform_data->client->addr);
    }
    if (ret == 0) {
        ret = values_size;
    }
    return ret;
}

s32 mstar_iic_write_data(u8 nSlaveId, u8 * pBuf, u16 nSize)
{
    s32 rc = 0;
    if (g_i2c_client != NULL) {
        if ((g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
             || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A)
            && nSlaveId == SLAVE_I2C_ID_DWI2C && g_fw_update != 0) {
            TS_LOG_ERR("Not allow to execute SmBus command while update firmware.\n");
        } else {
            g_mstar_dev_data->ts_platform_data->client->addr = nSlaveId;
            rc = tpkit_i2c_write(pBuf, nSize);

            // no return error if command is for serialDebug mode
            if (nSize == 5) {
                if (pBuf[0] == 0x53 && pBuf[1] == 0x45 && pBuf[2] == 0x52
                    && pBuf[3] == 0x44 && pBuf[4] == 0x42) {
                    rc = nSize;
                    goto out;
                }
            }

            if (nSize == 1) {
                if (pBuf[0] == 0x45) {
                    rc = nSize;
                    goto out;
                }
            }
            if (rc < 0) {
                TS_LOG_ERR("mstar_iic_write_data() error %d, nSlaveId=%d, nSize=%d\n", rc, nSlaveId,
                       nSize);
                mp_test_info.i2c_status = ERROR_I2C_WRITE;
            }
        }
    } else {
        TS_LOG_ERR("i2c client is NULL\n");
    }

out:
    return rc;
}

s32 mstar_iic_read_data(u8 nSlaveId, u8 * pBuf, u16 nSize)
{
    s32 rc = 0;
    if (g_i2c_client != NULL) {
        if ((g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
             || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A)
            && nSlaveId == SLAVE_I2C_ID_DWI2C && g_fw_update != 0) {
            TS_LOG_ERR("Not allow to execute SmBus command while update firmware.\n");
        } else {
            g_mstar_dev_data->ts_platform_data->client->addr = nSlaveId;
            rc = tpkit_i2c_read(pBuf, nSize);

            if (rc < 0) {
                TS_LOG_ERR("mstar_iic_read_data() error %d, nSlaveId=%d, nSize=%d\n", rc, nSlaveId,
                       nSize);
                mp_test_info.i2c_status = ERROR_I2C_READ;
            }
        }
    } else {
        TS_LOG_ERR("i2c client is NULL\n");
    }

    return rc;
}

s32 mstar_ii2c_segment_read_dbbus(u8 nRegBank, u8 nRegAddr, u8 * pBuf, u16 nSize, u16 nMaxI2cLengthLimit)
{
    s32 rc = 0;
    u16 nLeft = nSize;
    u16 nOffset = 0;
    u16 nSegmentLength = 0;
    u16 nReadSize = 0;
    u16 nOver = 0;
    u8 szWriteBuf[3] = { 0 };
    u8 nNextRegBank = nRegBank;
    u8 nNextRegAddr = nRegAddr;

    if (g_i2c_client != NULL) {
        u8 *pReadBuf = NULL;
        u16 nLength = 0;
        u8 nAddrBefore = g_i2c_client->addr;

        g_i2c_client->addr = SLAVE_I2C_ID_DBBUS;

        if (nMaxI2cLengthLimit >= 256) {
            nSegmentLength = 256;
        } else {
            nSegmentLength = 128;
        }

        TS_LOG_DEBUG("nSegmentLength = %d\n", nSegmentLength);

        while (nLeft > 0) {
            szWriteBuf[0] = 0x10;
            nRegBank = nNextRegBank;
            szWriteBuf[1] = nRegBank;
            nRegAddr = nNextRegAddr;
            szWriteBuf[2] = nRegAddr;

            TS_LOG_DEBUG("nRegBank = 0x%x, nRegAddr = 0x%x\n", nRegBank, nRegAddr);

            pReadBuf = &pBuf[nOffset];

            if (nLeft > nSegmentLength) {
                if ((nRegAddr + nSegmentLength) < MAX_TOUCH_IC_REGISTER_BANK_SIZE) {
                    nNextRegAddr = nRegAddr + nSegmentLength;

                    TS_LOG_DEBUG("nNextRegAddr = 0x%x\n", nNextRegAddr);

                    nLength = nSegmentLength;
                    nLeft -= nSegmentLength;
                    nOffset += nLength;
                } else if ((nRegAddr + nSegmentLength) == MAX_TOUCH_IC_REGISTER_BANK_SIZE) {
                    nNextRegAddr = 0x00;
                    nNextRegBank = nRegBank + 1;    // shift to read data from next register bank

                    TS_LOG_DEBUG("nNextRegBank = 0x%x\n", nNextRegBank);

                    nLength = nSegmentLength;
                    nLeft -= nSegmentLength;
                    nOffset += nLength;
                } else {
                    nNextRegAddr = 0x00;
                    nNextRegBank = nRegBank + 1;    // shift to read data from next register bank

                    TS_LOG_DEBUG("nNextRegBank = 0x%x\n", nNextRegBank);

                    nOver = (nRegAddr + nSegmentLength) - MAX_TOUCH_IC_REGISTER_BANK_SIZE;

                    TS_LOG_DEBUG("nOver = 0x%x\n", nOver);

                    nLength = nSegmentLength - nOver;
                    nLeft -= nLength;
                    nOffset += nLength;
                }
            } else {
                if ((nRegAddr + nLeft) < MAX_TOUCH_IC_REGISTER_BANK_SIZE) {
                    nNextRegAddr = nRegAddr + nLeft;

                    TS_LOG_DEBUG("nNextRegAddr = 0x%x\n", nNextRegAddr);

                    nLength = nLeft;
                    nLeft = 0;
                } else if ((nRegAddr + nLeft) == MAX_TOUCH_IC_REGISTER_BANK_SIZE) {
                    nNextRegAddr = 0x00;
                    nNextRegBank = nRegBank + 1;    // shift to read data from next register bank

                    TS_LOG_DEBUG("nNextRegBank = 0x%x\n", nNextRegBank);

                    nLength = nLeft;
                    nLeft = 0;
                } else {
                    nNextRegAddr = 0x00;
                    nNextRegBank = nRegBank + 1;    // shift to read data from next register bank

                    TS_LOG_DEBUG("nNextRegBank = 0x%x\n", nNextRegBank);

                    nOver = (nRegAddr + nLeft) - MAX_TOUCH_IC_REGISTER_BANK_SIZE;

                    TS_LOG_DEBUG("nOver = 0x%x\n", nOver);

                    nLength = nLeft - nOver;
                    nLeft -= nLength;
                    nOffset += nLength;
                }
            }

            rc = mstar_iic_write_data(g_i2c_client->addr, &szWriteBuf[0], 3);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_dbbus() -> i2c_master_send() error %d\n", rc);

                return rc;
            }

            rc = mstar_iic_read_data(g_i2c_client->addr, pReadBuf, nLength);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_dbbus() -> i2c_master_recv() error %d\n", rc);

                return rc;
            } else {
                nReadSize = nReadSize + nLength;
            }
        }
        g_i2c_client->addr = nAddrBefore;
    } else {
        TS_LOG_ERR("i2c client is NULL\n");
    }

    return nReadSize;
}

s32 mstar_ii2c_segment_read_smbus(u16 nAddr, u8 * pBuf, u16 nSize, u16 nMaxI2cLengthLimit)
{
    s32 rc = 0;
    u16 nLeft = nSize;
    u16 nOffset = 0;
    u16 nReadSize = 0;
    u8 szWriteBuf[3] = { 0 };
    if (g_i2c_client != NULL) {
#ifndef CONFIG_ENABLE_DMA_IIC
        u8 *pReadBuf = NULL;
#endif
        u16 nLength = 0;
        u8 nAddrBefore = g_i2c_client->addr;

        g_i2c_client->addr = SLAVE_I2C_ID_DWI2C;

#ifdef CONFIG_ENABLE_DMA_IIC
        while (nLeft > 0) {
            s32 i = 0;

            szWriteBuf[0] = 0x53;
            szWriteBuf[1] = ((nAddr + nOffset) >> 8) & 0xFF;
            szWriteBuf[2] = (nAddr + nOffset) & 0xFF;

            if (nLeft > nMaxI2cLengthLimit) {
                nLength = nMaxI2cLengthLimit;
                nLeft -= nMaxI2cLengthLimit;
            } else {
                nLength = nLeft;
                nLeft = 0;
            }

            rc = mstar_iic_write_data(g_i2c_client->addr, &szWriteBuf[0], 3);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_smbus() -> i2c_master_send() error %d\n", rc);

                return rc;
            }

            rc = mstar_iic_read_data(g_i2c_client->addr, pBuf, nLength);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_smbus() -> i2c_master_recv() error %d\n", rc);

                return rc;
            } else {
                nOffset += nLength;
                nReadSize = nReadSize + nLength;
            }
        }
#else
        while (nLeft > 0) {
            szWriteBuf[0] = 0x53;
            szWriteBuf[1] = ((nAddr + nOffset) >> 8) & 0xFF;
            szWriteBuf[2] = (nAddr + nOffset) & 0xFF;

            pReadBuf = &pBuf[nOffset];

            if (nLeft > nMaxI2cLengthLimit) {
                nLength = nMaxI2cLengthLimit;
                nLeft -= nMaxI2cLengthLimit;
                nOffset += nLength;
            } else {
                nLength = nLeft;
                nLeft = 0;
            }

            rc = mstar_iic_write_data(g_i2c_client->addr, &szWriteBuf[0], 3);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_smbus() -> i2c_master_send() error %d\n", rc);

                return rc;
            }

            rc = mstar_iic_read_data(g_i2c_client->addr, pReadBuf, nLength);
            if (rc < 0) {
                TS_LOG_ERR("mstar_ii2c_segment_read_smbus() -> i2c_master_recv() error %d\n", rc);

                return rc;
            } else {
                nReadSize = nReadSize + nLength;
            }
        }
#endif
        g_i2c_client->addr = nAddrBefore;
    } else {
        TS_LOG_ERR("i2c client is NULL\n");
    }

    return nReadSize;
}

static u8 mstar_calculate_checksum(u8 * pMsg, u32 nLength)
{
    s32 nCheckSum = 0;
    u32 i;

    for (i = 0; i < nLength; i++) {
        nCheckSum += pMsg[i];
    }

    return (u8) ((-nCheckSum) & 0xFF);
}

u32 mstar_convert_char_to_hex_digit(char *pCh, u32 nLength)
{
    u32 nRetVal = 0;
    u32 i;

    TS_LOG_INFO("nLength = %d\n", nLength);

    for (i = 0; i < nLength; i++) {
        char ch = *pCh++;
        u32 n = 0;
        u8 nIsValidDigit = 0;

        if ((i == 0 && ch == '0') || (i == 1 && ch == 'x')) {
            continue;
        }

        if ('0' <= ch && ch <= '9') {
            n = ch - '0';
            nIsValidDigit = 1;
        } else if ('a' <= ch && ch <= 'f') {
            n = 10 + ch - 'a';
            nIsValidDigit = 1;
        } else if ('A' <= ch && ch <= 'F') {
            n = 10 + ch - 'A';
            nIsValidDigit = 1;
        }

        if (1 == nIsValidDigit) {
            nRetVal = n + nRetVal * 16;
        }
    }

    return nRetVal;
}

void mstar_read_file(char *pFilePath, u8 * pBuf, u16 nLength)
{
    struct file *pFile = NULL;
    mm_segment_t old_fs;
    ssize_t nReadBytes = 0;

    old_fs = get_fs();
    set_fs(get_ds());

    pFile = filp_open(pFilePath, O_RDONLY, 0);
    if (IS_ERR(pFile)) {
        TS_LOG_INFO("Open file failed: %s\n", pFilePath);
        return;
    }

    pFile->f_op->llseek(pFile, 0, SEEK_SET);
    nReadBytes = pFile->f_op->read(pFile, pBuf, nLength, &pFile->f_pos);
    TS_LOG_INFO("Read %d bytes!\n", (int)nReadBytes);

    set_fs(old_fs);
    filp_close(pFile, NULL);
}

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
int mstar_proximity_enable(int mode)
{
    u8 data[4] = { 0 };
    u8 i = 0;
    int rc = 0;

    TS_LOG_INFO("%s called\n", __func__);

    data[0] = 0x52;
    data[1] = 0x00;
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        data[2] = 0x47;
    } else {
        TS_LOG_ERR("unrecognized chip type = 0x%x\n", g_chip_type);
        return -EIO;
    }

    if (mode) {
        data[3] = 0xa0;
    } else {
        data[3] = 0xa1;
    }

    while (i < 5) {
        if (mode && mstar_suspend_flag) {
            rc = 4;
            TS_LOG_INFO("tp suspend not write proximity enable cmd\n");
        }

        if (rc <= 0) {
            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &data[0], 4);
        }
        if (rc > 0) {
            if (mode) {
                TS_LOG_INFO("enable proximity detection success\n");
            } else {
                TS_LOG_INFO("disable proximity detection success\n");
            }
            break;
        }
        i++;
    }

    if (i >= 5) {
        TS_LOG_ERR("failed to enable proximity detection, rc = %d\n", rc);
    }

    return rc;
}

// only enabled but not report inputEvent to HAL
int mstar_ps_enable_nodata(int en, struct ts_proximity_info *info)
{
    int ret = 0;

    ret = mstar_proximity_enable(en);
    if (ret > 0) {
        info->proximity_enable = en;
    }

    return (ret > 0) ? 0 : -1;
}
#endif

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void mstar_regulator_power_switch(bool flag)
{
    s32 nRetVal = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_mstar_dev_data->ts_platform_data->vdd != NULL) {
        if (flag == true) {
            nRetVal = regulator_enable(g_mstar_dev_data->ts_platform_data->vdd);
            if (nRetVal) {
                TS_LOG_INFO("regulator_enable failed. nRetVal=%d\n", nRetVal);
            }
            mdelay(20);
        } else {
            nRetVal = regulator_disable(g_mstar_dev_data->ts_platform_data->vdd);
            if (nRetVal) {
                TS_LOG_INFO("regulator_disable failed. nRetVal=%d\n", nRetVal);
            }
            mdelay(20);
        }
    }
}
#endif

void mstar_exit_sleep_mode(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (MS_TS_MSG_IC_GPIO_RST >= 0 && MS_TS_MSG_IC_GPIO_RST != 1)   // MS_TS_MSG_IC_GPIO_RST must be a value other than 1
    {
        tpd_gpio_output(MS_TS_MSG_IC_GPIO_RST, 1);
        mdelay(10);
        tpd_gpio_output(MS_TS_MSG_IC_GPIO_RST, 0);
        mdelay(10);
        tpd_gpio_output(MS_TS_MSG_IC_GPIO_RST, 1);
        mdelay(25);
    }
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    g_charge_enable = 1;
#endif
    if (g_mstar_dev_data->need_wd_check_status) {
        g_esd_enable = 1;
        g_hw_reset_drv = 1;  // hw reset is triggered by driver, not fw or ic
    }
}

void mstar_enter_sleep_mode(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_optimize_current_consump();

    if (MS_TS_MSG_IC_GPIO_RST >= 0 && MS_TS_MSG_IC_GPIO_RST != 1)   // MS_TS_MSG_IC_GPIO_RST must be a value other than 1
    {
        tpd_gpio_output(MS_TS_MSG_IC_GPIO_RST, 0);
    }
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    g_charge_enable = 0;
#endif
    if (g_mstar_dev_data->need_wd_check_status)
        g_esd_enable = 0;
}

void mstar_dev_hw_reset(void)
{
    TS_LOG_INFO("MSTP RESET!\n");
    tpd_gpio_output(GTP_RST_PORT, 1);
    msleep(10);
    tpd_gpio_output(GTP_RST_PORT, 0);
    msleep(10);
    tpd_gpio_output(GTP_RST_PORT, 1);
    msleep(25);
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    g_charge_enable = 1;
#endif

    if (g_mstar_dev_data->need_wd_check_status) {
        g_esd_enable = 1;
        g_hw_reset_drv = 1; // Indicate HwReset is triggered by Device Driver instead of Firmware or Touch IC
    }
}

void mstar_finger_touch_report_disable(void)
{
    unsigned long nIrqFlag;

    TS_LOG_INFO("%s() g_int_flag = %d\n", __func__, g_int_flag);

    spin_lock_irqsave(&g_irq_lock, nIrqFlag);

    if (g_int_flag == 1) {
        disable_irq_nosync(g_mstar_dev_data->ts_platform_data->irq_id);

        g_int_flag = 0;
    }

    spin_unlock_irqrestore(&g_irq_lock, nIrqFlag);
}

void mstar_finger_touch_report_enable(void)
{
    unsigned long nIrqFlag;

    TS_LOG_INFO("*** %s() g_int_flag = %d ***\n", __func__, g_int_flag);

    spin_lock_irqsave(&g_irq_lock, nIrqFlag);

    if (g_int_flag == 0) {
        enable_irq(g_mstar_dev_data->ts_platform_data->irq_id);

        g_int_flag = 1;
    }

    spin_unlock_irqrestore(&g_irq_lock, nIrqFlag);
}

void mstar_finger_touch_release(void)
{
    TS_LOG_INFO("point touch released\n");
    if (g_mstar_dev_data->ts_platform_data->input_dev) {
        input_mt_sync(g_mstar_dev_data->ts_platform_data->input_dev);
        input_report_key(g_mstar_dev_data->ts_platform_data->input_dev, BTN_TOUCH, 0);
        input_sync(g_mstar_dev_data->ts_platform_data->input_dev);
        TS_LOG_DEBUG("%s: input_dev->name = %s\n", __func__,
                 g_mstar_dev_data->ts_platform_data->input_dev->name);
    }
}

void mstar_set_ii2c_data_rate(struct i2c_client *pClient, u32 nIicDataRate)
{
    TS_LOG_INFO("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);
}


void mstar_esd_check(void)
{
    u8 szData[1] = { 0x00 };
    u32 i = 0;
    s32 rc = 0;

    TS_LOG_DEBUG("*** %s() g_esd_enable = %d ***\n", __func__, g_esd_enable);
    mutex_lock(&g_mutex);
    if (g_esd_enable == 0) {
        goto out;
    }

    if (g_int_flag == 0)    // Skip ESD check while finger touch
    {
        TS_LOG_DEBUG("Not allow to do ESD check while finger touch.\n");
        goto out;
    }

    if (g_fw_update != 0)   // Check whether update frimware is finished
    {
        TS_LOG_DEBUG("Not allow to do ESD check while update firmware is proceeding.\n");
        goto out;
    }

    if (g_mp_test == 1) // Check whether mp test is proceeding
    {
        TS_LOG_DEBUG("Not allow to do ESD check while mp test is proceeding.\n");
        goto out;
    }

    if (g_int_esd_flag == 1)
    {
        TS_LOG_DEBUG("Not allow to do ESD check while screen is being touched\n");
        goto out;
    }

    if (IS_FIRMWARE_DATA_LOG_ENABLED) {
        if (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE)  // Skip ESD check while firmware mode is DEBUG MODE
        {
            TS_LOG_DEBUG("Not allow to do ESD check while firmware mode is DEBUG MODE.\n");
            goto out;
        }
    }

    if (IS_DISABLE_ESD_PROTECTION_CHECK)    // Skip ESD check while mp test is triggered by MTPTool APK through JNI interface
    {
        TS_LOG_DEBUG
            ("Not allow to do ESD check while mp test is triggered by MTPTool APK through JNI interface.\n");
        goto out;
    }
    // Method 2. Use I2C write command for checking whether I2C connection is still available under ESD testing

    szData[0] = 0x00;   // Dummy command for ESD check

    while (i < 3) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX
            || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
            || g_chip_type == CHIP_TYPE_ILI2117A) {
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szData[0], 1);
        } else {
            TS_LOG_ERR("Un-recognized chip type = 0x%x\n", g_chip_type);
            break;
        }

        if (rc > 0) {
            TS_LOG_DEBUG("ESD check success\n");
            break;
        }

        i++;
    }

    if (i >= 3) {
        TS_LOG_ERR("ESD check failed, rc = %d\n", rc);
        mstar_dev_hw_reset();
#if defined (CONFIG_HUAWEI_DSM)
        if (!dsm_client_ocuppy(ts_dclient)) {
            dsm_client_record(ts_dclient, "Mstar ESD check failed\n");
            dsm_client_notify(ts_dclient,DSM_TP_ESD_ERROR_NO);
        }
#endif
    }
out:
    mutex_unlock(&g_mutex);
    g_int_esd_flag = 0;
}

static void mstar_var_init(void)
{
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        FIRMWARE_MODE_UNKNOWN_MODE = MSG28XX_FIRMWARE_MODE_UNKNOWN_MODE;
        FIRMWARE_MODE_DEMO_MODE = MSG28XX_FIRMWARE_MODE_DEMO_MODE;
        FIRMWARE_MODE_DEBUG_MODE = MSG28XX_FIRMWARE_MODE_DEBUG_MODE;

        g_fw_mode = FIRMWARE_MODE_DEMO_MODE;

        DEMO_MODE_PACKET_LENGTH = MUTUAL_DEMO_MODE_PACKET_LENGTH;
        DEBUG_MODE_PACKET_LENGTH = MUTUAL_DEBUG_MODE_PACKET_LENGTH;
        MAX_TOUCH_NUM = MUTUAL_MAX_TOUCH_NUM;
    }
}

void mstar_get_customer_fw_ver(u16 * pMajor, u16 * pMinor, u8 ** ppVersion)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        u8 szTxData[3] = { 0 };
        u8 szRxData[4] = { 0 };

        szTxData[0] = 0x03;

        mutex_lock(&g_mutex);

        //mstar_dev_hw_reset();

        mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 1);
        mdelay(I2C_SMBUS_READ_COMMAND_DELAY_FOR_PLATFORM);
        mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 4);

        mutex_unlock(&g_mutex);

        TS_LOG_DEBUG("szRxData[0] = 0x%x, szRxData[1] = 0x%x, szRxData[2] = 0x%x, szRxData[3] = 0x%x\n",
                szRxData[0], szRxData[1], szRxData[2], szRxData[3]);

        *pMajor = (szRxData[1] << 8) + szRxData[0];
        *pMinor = (szRxData[3] << 8) + szRxData[2];
    }
    if (*ppVersion == NULL) {
        *ppVersion = kzalloc(sizeof(u8) * 16, GFP_KERNEL);
    }

    TS_LOG_INFO("*** Major = %d ***\n", *pMajor);
    if (g_fw_ver_flag) {
        TS_LOG_INFO("*** Minor = %d.%d ***\n", (*pMinor & 0xFF), ((*pMinor >> 8) & 0xFF));
    } else {
        TS_LOG_INFO("*** Minor = %d ***\n", *pMinor);
    }
    sprintf(*ppVersion, "%05d.%05d", *pMajor, *pMinor);
}

void mstar_get_platform_fw_ver(u8 ** ppVersion)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        u8 szTxData[1] = { 0 };
        u8 szRxData[10] = { 0 };

        szTxData[0] = 0x0C;

        mutex_lock(&g_mutex);

        //mstar_dev_hw_reset();

#ifdef CONFIG_ENABLE_DMA_IIC
        DmaReset();
#endif
        mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 1);
        mdelay(I2C_SMBUS_READ_COMMAND_DELAY_FOR_PLATFORM);
        mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 10);

        mutex_unlock(&g_mutex);

        TS_LOG_DEBUG("szRxData[0] = 0x%x , %c \n", szRxData[0], szRxData[0]);
        TS_LOG_DEBUG("szRxData[1] = 0x%x , %c \n", szRxData[1], szRxData[1]);
        TS_LOG_DEBUG("szRxData[2] = 0x%x , %c \n", szRxData[2], szRxData[2]);
        TS_LOG_DEBUG("szRxData[3] = 0x%x , %c \n", szRxData[3], szRxData[3]);
        TS_LOG_DEBUG("szRxData[4] = 0x%x , %c \n", szRxData[4], szRxData[4]);
        TS_LOG_DEBUG("szRxData[5] = 0x%x , %c \n", szRxData[5], szRxData[5]);
        TS_LOG_DEBUG("szRxData[6] = 0x%x , %c \n", szRxData[6], szRxData[6]);
        TS_LOG_DEBUG("szRxData[7] = 0x%x , %c \n", szRxData[7], szRxData[7]);
        TS_LOG_DEBUG("szRxData[8] = 0x%x , %c \n", szRxData[8], szRxData[8]);
        TS_LOG_DEBUG("szRxData[9] = 0x%x , %c \n", szRxData[9], szRxData[9]);

        if (*ppVersion == NULL) {
            *ppVersion = kzalloc(sizeof(u8) * 16, GFP_KERNEL);
        }

        sprintf(*ppVersion, "%.10s", szRxData);
        sscanf(*ppVersion, "V%u.%u.%u\n", &g_platform_fw_ver[0], &g_platform_fw_ver[1], &g_platform_fw_ver[2]);

        if (g_platform_fw_ver[0] * 100000 + (g_platform_fw_ver[1]) * 100 + g_platform_fw_ver[2] >= 101003) {
            g_fw_ver_flag = 1;
        }
    } else {
        if (*ppVersion == NULL) {
            *ppVersion = kzalloc(sizeof(u8) * 16, GFP_KERNEL);
        }

        sprintf(*ppVersion, "%s", "N/A");
    }

    TS_LOG_INFO("*** platform firmware version = %s ***\n", *ppVersion);
}

s32 mstar_update_fw(u8 ** szFwData, EmemType_e eEmemType)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    return mstar_update_fw_cash(szFwData, eEmemType);
}

s32 mstar_update_fw_sdcard(const char *pFilePath)
{
    s32 nRetVal = -1;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
        || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
        nRetVal = mstar_fw_update_sdcard(pFilePath, 1);
    } else {
        TS_LOG_INFO("This chip type (0x%x) does not support update firmware by sd card\n", g_chip_type);
    }

    return nRetVal;
}

u16 mstar_change_fw_mode(u16 nMode)
{
    TS_LOG_INFO("*** %s() *** nMode = 0x%x\n", __func__, nMode);

    if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
        || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2120 || g_chip_type == CHIP_TYPE_ILI2121) {
        u8 szTxData[2] = { 0 };
        u32 i = 0;
        s32 rc;

        g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send change firmware mode i2c command to firmware.
        {
            szTxData[0] = 0x02;
        }
        szTxData[1] = (u8) nMode;

        mutex_lock(&g_mutex);
        TS_LOG_DEBUG("*** %s() *** mutex_lock(&g_mutex)\n", __func__);

        while (i < 5) {
            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 2);
            if (rc > 0) {
                TS_LOG_INFO("Change firmware mode success\n");
                break;
            }

            i++;
        }
        if (i == 5) {
            TS_LOG_ERR("Change firmware mode failed, rc = %d\n", rc);
        }

        TS_LOG_DEBUG("*** %s() *** mutex_unlock(&g_mutex)\n", __func__);    // add for debug
        mutex_unlock(&g_mutex);

        g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
    }

    return nMode;
}

void mstar_mutual_get_fw_info(MutualFirmwareInfo_t * pInfo)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        u8 szTxData[1] = { 0 };
        u8 szRxData[10] = { 0 };
        u32 i = 0;
        s32 rc;
        szTxData[0] = 0x01;
        g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send get firmware info i2c command to firmware.

        mutex_lock(&g_mutex);
        TS_LOG_DEBUG("*** %s() *** mutex_lock(&g_mutex)\n", __func__);

#ifdef CONFIG_ENABLE_DMA_IIC
        DmaReset();
#endif

        while (i < 5) {
            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 1);
            if (rc > 0) {
                TS_LOG_INFO("Get firmware info mstar_iic_write_data() success\n");
            }

            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 10);
            if (rc > 0) {
                TS_LOG_INFO("Get firmware info mstar_iic_read_data() success\n");
                TS_LOG_INFO("FW mode: 0x%x\n", szRxData[1]);
                if (szRxData[1] == FIRMWARE_MODE_DEMO_MODE || szRxData[1] == FIRMWARE_MODE_DEBUG_MODE) {

                    break;
                } else {
                    i = 0;
                }
            }

            i++;
        }
        if (i == 5) {
            TS_LOG_ERR("Get firmware info failed, rc = %d\n", rc);
        }

        TS_LOG_INFO("*** %s() *** mutex_unlock(&g_mutex)\n", __func__);
        mutex_unlock(&g_mutex);

        // add protection for incorrect firmware info check
        if ((szRxData[1] == FIRMWARE_MODE_DEBUG_MODE && szRxData[2] == 0xA7
             && szRxData[5] == PACKET_TYPE_TOOTH_PATTERN) || (szRxData[1] == FIRMWARE_MODE_DEMO_MODE
                                      && szRxData[2] == 0x5A)) {
            pInfo->nFirmwareMode = szRxData[1];
            TS_LOG_INFO("pInfo->nFirmwareMode = 0x%x\n", pInfo->nFirmwareMode);

            pInfo->nLogModePacketHeader = szRxData[2];
            pInfo->nLogModePacketLength = (szRxData[3] << 8) + szRxData[4];
            pInfo->nType = szRxData[5];
            pInfo->nMy = szRxData[6];
            pInfo->nMx = szRxData[7];
            pInfo->nSd = szRxData[8];
            pInfo->nSs = szRxData[9];

            TS_LOG_DEBUG("pInfo->nLogModePacketHeader = 0x%x\n", pInfo->nLogModePacketHeader);
            TS_LOG_DEBUG("pInfo->nLogModePacketLength = %d\n", pInfo->nLogModePacketLength);
            TS_LOG_DEBUG("pInfo->nType = 0x%x\n", pInfo->nType);
            TS_LOG_DEBUG("pInfo->nMy = %d\n", pInfo->nMy);
            TS_LOG_DEBUG("pInfo->nMx = %d\n", pInfo->nMx);
            TS_LOG_DEBUG("pInfo->nSd = %d\n", pInfo->nSd);
            TS_LOG_DEBUG("pInfo->nSs = %d\n", pInfo->nSs);
        } else {
            TS_LOG_INFO("Firmware info before correcting :\n");

            TS_LOG_DEBUG("FirmwareMode = 0x%x\n", szRxData[1]);
            TS_LOG_DEBUG("LogModePacketHeader = 0x%x\n", szRxData[2]);
            TS_LOG_DEBUG("LogModePacketLength = %d\n", (szRxData[3] << 8) + szRxData[4]);
            TS_LOG_DEBUG("Type = 0x%x\n", szRxData[5]);
            TS_LOG_DEBUG("My = %d\n", szRxData[6]);
            TS_LOG_DEBUG("Mx = %d\n", szRxData[7]);
            TS_LOG_DEBUG("Sd = %d\n", szRxData[8]);
            TS_LOG_DEBUG("Ss = %d\n", szRxData[9]);

            // Set firmware mode to demo mode(default)
            pInfo->nFirmwareMode = FIRMWARE_MODE_DEMO_MODE;
            pInfo->nLogModePacketHeader = 0x5A;
            pInfo->nLogModePacketLength = DEMO_MODE_PACKET_LENGTH;
            pInfo->nType = 0;
            pInfo->nMy = 0;
            pInfo->nMx = 0;
            pInfo->nSd = 0;
            pInfo->nSs = 0;

            TS_LOG_INFO("Firmware info after correcting :\n");

            TS_LOG_DEBUG("pInfo->nFirmwareMode = 0x%x\n", pInfo->nFirmwareMode);
            TS_LOG_DEBUG("pInfo->nLogModePacketHeader = 0x%x\n", pInfo->nLogModePacketHeader);
            TS_LOG_DEBUG("pInfo->nLogModePacketLength = %d\n", pInfo->nLogModePacketLength);
            TS_LOG_DEBUG("pInfo->nType = 0x%x\n", pInfo->nType);
            TS_LOG_DEBUG("pInfo->nMy = %d\n", pInfo->nMy);
            TS_LOG_DEBUG("pInfo->nMx = %d\n", pInfo->nMx);
            TS_LOG_DEBUG("pInfo->nSd = %d\n", pInfo->nSd);
            TS_LOG_DEBUG("pInfo->nSs = %d\n", pInfo->nSs);
        }

        g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
    }
}

void mstar_restore_fw_to_log_data(void)
{
    TS_LOG_INFO("*** %s() g_switch_mode_apk = %d ***\n", __func__, g_switch_mode_apk);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        if (g_switch_mode_apk == 1) {
            MutualFirmwareInfo_t tInfo;

            memset(&tInfo, 0x0, sizeof(MutualFirmwareInfo_t));

            mstar_mutual_get_fw_info(&tInfo);

            TS_LOG_INFO("g_fw_mode = 0x%x, tInfo.nFirmwareMode = 0x%x\n", g_fw_mode, tInfo.nFirmwareMode);

            // since reset_hw() will reset the firmware mode to demo mode, we must reset the firmware mode again after reset_hw().
            if (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE && FIRMWARE_MODE_DEBUG_MODE != tInfo.nFirmwareMode) {
                g_fw_mode = mstar_change_fw_mode(FIRMWARE_MODE_DEBUG_MODE);
            } else {
                TS_LOG_INFO("firmware mode is not restored\n");
            }
        }
    }
}

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
void mstar_get_touch_packet_addr(u16 * pDataAddress, u16 * pFlagAddress)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120 || g_chip_type == CHIP_TYPE_ILI2121
        || (g_chip_type == CHIP_TYPE_MSG22XX && IS_SELF_FREQ_SCAN_ENABLED)) {
        s32 rc = 0;
        u32 i = 0;
        u8 szTxData[1] = { 0 };
        u8 szRxData[4] = { 0 };
        szTxData[0] = 0x05;
        mutex_lock(&g_mutex);
        TS_LOG_INFO("*** %s() *** mutex_lock(&g_mutex)\n", __func__);

        while (i < 5) {
            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 1);
            if (rc > 0) {
                TS_LOG_INFO("Get touch packet address mstar_iic_write_data() success\n");
            }

            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 4);
            if (rc > 0) {
                TS_LOG_INFO("Get touch packet address mstar_iic_read_data() success\n");
                break;
            }

            i++;
        }
        if (i == 5) {
            TS_LOG_ERR("Get touch packet address failed, rc = %d\n", rc);
        }

        if (rc < 0) {
            g_fw_support_seg = 0;
        } else {
            {
                *pDataAddress = (szRxData[0] << 8) + szRxData[1];
                *pFlagAddress = (szRxData[2] << 8) + szRxData[3];
            }

            g_fw_support_seg = 1;

            TS_LOG_DEBUG("*** *pDataAddress = 0x%2X ***\n", *pDataAddress); // add for debug
            TS_LOG_DEBUG("*** *pFlagAddress = 0x%2X ***\n", *pFlagAddress); // add for debug
        }

        TS_LOG_DEBUG("*** %s() *** mutex_unlock(&g_mutex)\n", __func__);    // add for debug
        mutex_unlock(&g_mutex);
    }
}

static int mstar_finger_touch_packet_flag_bit1(void)
{
    u8 szTxData[3] = { 0 };
    s32 nRetVal;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    szTxData[0] = 0x53;
    szTxData[1] = (g_fw_packet_flag_addr >> 8) & 0xFF;
    szTxData[2] = g_fw_packet_flag_addr & 0xFF;

    mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
    mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &g_touch_packet_flag[0], 2);

    if ((g_touch_packet_flag[0] & BIT1) == 0x00) {
        nRetVal = 0;    // Bit1 is 0
    } else {
        nRetVal = 1;    // Bit1 is 1
    }
    TS_LOG_INFO("Bit1 = %d\n", nRetVal);

    return nRetVal;
}

static void mstar_reset_finger_touch_packet_flag_bit1(void)
{
    u8 szTxData[4] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    szTxData[0] = 0x52;
    szTxData[1] = (g_fw_packet_flag_addr >> 8) & 0xFF;
    szTxData[2] = g_fw_packet_flag_addr & 0xFF;
    szTxData[3] = g_touch_packet_flag[0] | BIT1;

    mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 4);
}
#endif

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
void mstar_charge_detect(u8 nChargerStatus)
{
    u32 i = 0;
    u8 szTxData[2] = { 0 };
    s32 rc = 0;

    TS_LOG_INFO("g_charge_plug_in = %d, nChargerStatus = %d, g_force_update = %d\n", g_charge_plug_in,
            nChargerStatus, g_force_update);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        szTxData[0] = 0x09;
    }

    if (nChargerStatus) // charger plug in
    {
        if (g_charge_plug_in == 0 || g_force_update == 1) {
            szTxData[1] = 0xA5;

            while (i < 5) {
                mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
                mutex_lock(&g_mutex);
                rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 2);
                mutex_unlock(&g_mutex);
                if (rc > 0) {
                    g_charge_plug_in = 1;

                    TS_LOG_INFO("Update status for charger plug in success.\n");
                    break;
                }

                i++;
            }
            if (i == 5) {
                TS_LOG_ERR("Update status for charger plug in failed, rc = %d\n", rc);
            }

            g_force_update = 0; // Clear flag after force update charger status
        }
    } else          // charger plug out
    {
        if (g_charge_plug_in == 1 || g_force_update == 1) {
            szTxData[1] = 0x5A;

            while (i < 5) {
                mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
                mutex_lock(&g_mutex);
                rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 2);
                mutex_unlock(&g_mutex);
                if (rc > 0) {
                    g_charge_plug_in = 0;

                    TS_LOG_INFO("Update status for charger plug out success.\n");
                    break;
                }

                i++;
            }
            if (i == 5) {
                TS_LOG_ERR("Update status for charger plug out failed, rc = %d\n", rc);
            }

            g_force_update = 0; // Clear flag after force update charger status
        }
    }
}

static void mstar_charge_check(struct work_struct *pWork)
{
    u8 szChargerStatus[20] = { 0 };

    TS_LOG_INFO("*** %s() g_charge_enable = %d ***\n", __func__, g_charge_enable);

    if (g_charge_enable == 0) {
        return;
    }
/*
    if (g_int_flag == 0) // Skip charger plug in/out check while finger touch
    {
        TS_LOG_INFO( "Not allow to do charger plug in/out check while finger touch.\n");
        goto ChargerPlugInOutCheckEnd;
    }
*/
    if (g_fw_update != 0)   // Check whether update frimware is finished
    {
        TS_LOG_INFO("Not allow to do charger plug in/out check while update firmware is proceeding.\n");
        goto ChargerPlugInOutCheckEnd;
    }
#ifdef CONFIG_ENABLE_ITO_MP_TEST
    if (g_mp_test == 1) // Check whether mp test is proceeding
    {
        TS_LOG_INFO("Not allow to do charger plug in/out check while mp test is proceeding.\n");
        goto ChargerPlugInOutCheckEnd;
    }
#endif

    mstar_read_file(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

    TS_LOG_INFO("*** Battery Status : %s ***\n", szChargerStatus);

    if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL
        || strstr(szChargerStatus, "Fully charged") != NULL) {
        mstar_charge_detect(1); // charger plug-in
    } else {
        mstar_charge_detect(0); // charger plug-out
    }

ChargerPlugInOutCheckEnd:

    if (g_charge_enable == 1) {
        queue_delayed_work(g_charge_workqueue, &g_charge_delayed_work, CHARGER_DETECT_CHECK_PERIOD);
    }
}
#endif

static s32 mstar_mutual_read_ftd(u8 * pPacket, u16 nReportPacketLength)
{
    s32 rc;

    if (IS_FIRMWARE_DATA_LOG_ENABLED) {
        if (g_fw_mode == FIRMWARE_MODE_DEMO_MODE) {
            rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &pPacket[0], nReportPacketLength);
            if (rc < 0) {
                TS_LOG_ERR("I2C read packet data failed, rc = %d\n", rc);
                return -1;
            }
        } else if (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE) {
#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
            TS_LOG_INFO("*** g_fw_packet_data_addr = 0x%2X ***\n", g_fw_packet_data_addr);  // add for debug
            TS_LOG_INFO("*** g_fw_packet_flag_addr = 0x%2X ***\n", g_fw_packet_flag_addr);  // add for debug

            if (g_fw_support_seg == 0) {
                TS_LOG_INFO("g_fw_packet_data_addr & g_fw_packet_flag_addr is un-initialized\n");
                return -1;
            }

            if (g_finger_touch_disable == 1) {
                TS_LOG_INFO
                    ("Skip finger touch for handling get firmware info or change firmware mode\n");
                return -1;
            }

            if (0 != mstar_finger_touch_packet_flag_bit1()) {
                TS_LOG_INFO
                    ("Bit1 is not 0. FW is not ready for providing debug mode packet to Device Driver\n");
                return -1;
            }

            rc = mstar_ii2c_segment_read_smbus(g_fw_packet_data_addr, &pPacket[0], nReportPacketLength,
                               MAX_I2C_TRANSACTION_LENGTH_LIMIT);

            mstar_reset_finger_touch_packet_flag_bit1();

            if (rc < 0) {
                TS_LOG_INFO("I2C read debug mode packet data failed, rc = %d\n", rc);
                return -1;
            }
#else

            rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &pPacket[0], nReportPacketLength);
            if (rc < 0) {
                TS_LOG_INFO("I2C read packet data failed, rc = %d\n", rc);
                return -1;
            }
#endif
        } else {
            TS_LOG_INFO("WRONG FIRMWARE MODE : 0x%x\n", g_fw_mode);
            return -1;
        }
    } else {
        rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &pPacket[0], nReportPacketLength);
        if (rc < 0) {
            TS_LOG_INFO("I2C read packet data failed, rc = %d\n", rc);
            return -1;
        }
    }

    return 0;
}

static void mstar_read_touch_data(struct ts_fingers *info)
{
    MutualTouchInfo_t tInfo;
    static u32 nLastCount = 0;
    u8 *pPacket = NULL;
    u16 nReportPacketLength = 0;

    if (g_finger_touch_disable == 1) {
        TS_LOG_INFO("Skip finger touch for handling get firmware info or change firmware mode\n");
        return;
    }

    mutex_lock(&g_mutex);
    mutex_lock(&g_mutex_protect);
    memset(&tInfo, 0x0, sizeof(MutualTouchInfo_t));

    if (IS_FIRMWARE_DATA_LOG_ENABLED) {
        if (g_fw_mode == FIRMWARE_MODE_DEMO_MODE) {
            TS_LOG_DEBUG("FIRMWARE_MODE_DEMO_MODE\n");

            nReportPacketLength = DEMO_MODE_PACKET_LENGTH;
            pPacket = g_demo_packet;
        } else if (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE) {
            TS_LOG_DEBUG("FIRMWARE_MODE_DEBUG_MODE\n");

            if (g_mutual_fw_info.nLogModePacketHeader != 0xA7) {
                TS_LOG_INFO("WRONG DEBUG MODE HEADER : 0x%x\n", g_mutual_fw_info.nLogModePacketHeader);
                goto TouchHandleEnd;
            }

            nReportPacketLength = g_mutual_fw_info.nLogModePacketLength;
            pPacket = g_debug_packet;
        } else {
            TS_LOG_INFO("WRONG FIRMWARE MODE : 0x%x\n", g_fw_mode);
            goto TouchHandleEnd;
        }
    } else {
        TS_LOG_DEBUG("FIRMWARE_MODE_DEMO_MODE\n");

        nReportPacketLength = DEMO_MODE_PACKET_LENGTH;
        pPacket = g_demo_packet;
    }

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
    if (g_gesture_debug_mode == 1 && g_gesture_wakeup_flag == 1) {
        TS_LOG_INFO("Set gesture debug mode packet length, g_chip_type=0x%x\n", g_chip_type);

        if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
            || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
            nReportPacketLength = GESTURE_DEBUG_MODE_PACKET_LENGTH;
            pPacket = g_gesture_wakeup_packet;
        } else {
            TS_LOG_INFO("This chip type does not support gesture debug mode.\n");
            goto TouchHandleEnd;
        }
    } else if (g_gesture_wakeup_flag == 1) {
        TS_LOG_INFO("Set gesture wakeup packet length\n");

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
        nReportPacketLength = GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH;
#else
        nReportPacketLength = GESTURE_WAKEUP_PACKET_LENGTH;
#endif
        pPacket = g_gesture_wakeup_packet;
    }
#else

    if (g_gesture_wakeup_flag == 1) {
        TS_LOG_INFO("Set gesture wakeup packet length\n");

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
        nReportPacketLength = GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH;
#else
        nReportPacketLength = GESTURE_WAKEUP_PACKET_LENGTH;
#endif
        pPacket = g_gesture_wakeup_packet;
    }
#endif

    if (0 != mstar_mutual_read_ftd(&pPacket[0], nReportPacketLength)) {
        goto TouchHandleEnd;
    }

    if (0 == mstar_mutual_parse_packet(pPacket, nReportPacketLength, &tInfo, info)) {

        TS_LOG_DEBUG("tInfo.nCount = %d, nLastCount = %d\n", tInfo.nCount, nLastCount);

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
        if (g_report_rate_enable == 1) {
            if (g_vaild_touch_cont == 4294967295UL) {
                g_vaild_touch_cont = 0; // Reset count if overflow
                TS_LOG_INFO("g_vaild_touch_cont reset to 0\n");
            }

            g_vaild_touch_cont++;

            TS_LOG_INFO("g_vaild_touch_cont = %d\n", g_vaild_touch_cont);
        }
#endif
    }

TouchHandleEnd:
    mutex_unlock(&g_mutex_protect);
    mutex_unlock(&g_mutex);
}

s32 mstar_chip_detect_init(void)
{
    s32 nRetVal = 0;
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    u8 szChargerStatus[20] = { 0 };
#endif

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_apknode_create_procfs();

#ifdef CONFIG_ENABLE_JNI_INTERFACE
    mstar_apknode_create_jni_msg();
#endif

    g_chip_type = mstar_get_chip_type();    // Try to get chip type by SLAVE_I2C_ID_DBBUS(0x62) firstly.

    if (g_chip_type == 0xbf) {
        sprintf(g_chip_num, "MSG%xA", 0x2836);
        TS_LOG_INFO("*** chip: %s ***\n", g_chip_num);
    }

    if (g_chip_type == 0)   // If failed, try to get chip type by SLAVE_I2C_ID_DBBUS(0x59) again.
    {
        SLAVE_I2C_ID_DBBUS = (0xB2 >> 1);   //0x59
        g_chip_type = mstar_get_chip_type();
    }

    mstar_dev_hw_reset();

    if (g_chip_type != 0)   // To make sure TP is attached on cell phone.
    {
        if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
            || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
            memset(&g_mutual_fw_info, 0x0, sizeof(MutualFirmwareInfo_t));
        }

        mstar_var_init();

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
        mstar_read_file(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

        TS_LOG_INFO("*** Battery Status : %s ***\n", szChargerStatus);

        if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL) // Charging
        {
            mstar_charge_detect(1); // charger plug-in
        }
        elses {
            mstar_charge_detect(0); // charger plug-out
        }
#endif

    } else {
        nRetVal = -ENODEV;
    }

    return nRetVal;
}

void mstar_mutex_init(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    mutex_init(&g_mutex);
    mutex_init(&g_mutex_protect);
    mutex_init(&wrong_gesture_lock);
    spin_lock_init(&g_irq_lock);
}

s32 mstar_input_device_init(void)
{
    s32 nRetVal = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_input_dev = g_mstar_dev_data->ts_platform_data->input_dev;

    return nRetVal;
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 mstar_unregister_device(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
    mstar_regulator_power_switch(false);
#endif

    if (g_touch_kset) {
        kset_unregister(g_touch_kset);
        g_touch_kset = NULL;
    }

    if (g_touch_kobj) {
        kobject_put(g_touch_kobj);
        g_touch_kobj = NULL;
    }
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
    if (g_gesture_kset) {
        kset_unregister(g_gesture_kset);
        g_gesture_kset = NULL;
    }

    if (g_GestureKObj) {
        kobject_put(g_GestureKObj);
        g_GestureKObj = NULL;
    }
#endif

    mstar_apknode_remove_procfs();

#ifdef CONFIG_ENABLE_JNI_INTERFACE
    mstar_apknode_delete_jni_msg();
#endif

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    if (g_charge_workqueue) {
        destroy_workqueue(g_charge_workqueue);
        g_charge_workqueue = NULL;
    }
#endif

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaFree();
#endif
#endif

    return 0;
}

static s32 mstar_mutual_parse_packet(u8 * pPacket, u16 nLength, MutualTouchInfo_t * pInfo, struct ts_fingers *info)
{
    u32 i, ret = 0;
    u8 nCheckSum = 0;
    u32 nX = 0, nY = 0;
    int touch_count = 0;
    u32 nGestureCount = 0;
    u32 nReportGestureType = 0;
    u32 nTmpBuffer[80] = { 0 };
    struct ts_proximity_info *ps_info = &g_mstar_dev_data->ts_platform_data->feature_info.proximity_info;
    ps_info->proximity_status = 0xFF;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
    if (g_report_rate_enable == 1) {
        if (g_int_cont == 4294967295UL) {
            g_int_cont = 0; // Reset count if overflow
            TS_LOG_DEBUG("g_int_cont reset to 0\n");
        }

        if (g_int_cont == 0) {
            // Get start time
            do_gettimeofday(&g_StartTime);

            TS_LOG_DEBUG("Start time : %lu sec, %lu msec\n", g_StartTime.tv_sec, g_StartTime.tv_usec);
        }

        g_int_cont++;

        TS_LOG_DEBUG("g_int_cont = %d\n", g_int_cont);
    }
#endif

    TS_LOG_DEBUG("received raw data from touch panel as following:\n");
    TS_LOG_DEBUG
        ("pPacket[0]=%x \n pPacket[1]=%x pPacket[2]=%x pPacket[3]=%x pPacket[4]=%x \n pPacket[5]=%x pPacket[6]=%x pPacket[7]=%x pPacket[8]=%x \n",
         pPacket[0], pPacket[1], pPacket[2], pPacket[3], pPacket[4], pPacket[5], pPacket[6], pPacket[7],
         pPacket[8]);

    if (IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED) {
        if (pPacket[0] == 0x2C) {
            // Notify android application to retrieve firmware specific debug value packet from device driver by sysfs.
            if (g_touch_kobj != NULL) {
                char szRspFwSpecificLogPacket[512] = { 0 };
                char szValue[3] = { 0 };
                char *pEnvp[3];
                s32 nRetVal = 0;

                strcat(szRspFwSpecificLogPacket, "VALUE=");

                for (i = 0; i < nLength; i++) {
                    sprintf(szValue, "%02x", pPacket[i]);
                    strcat(szRspFwSpecificLogPacket, szValue);
                }

                pEnvp[0] = "STATUS=GET_FW_LOG";
                pEnvp[1] = szRspFwSpecificLogPacket;
                pEnvp[2] = NULL;
                TS_LOG_DEBUG("pEnvp[1] = %s\n", pEnvp[1]);
                TS_LOG_DEBUG("g_demo_packet[] = %s\n", pPacket);

                nRetVal = kobject_uevent_env(g_touch_kobj, KOBJ_CHANGE, pEnvp);
                TS_LOG_DEBUG("kobject_uevent_env() STATUS=GET_FW_LOG, nRetVal = %d\n", nRetVal);
            }

            return -1;
        }
    }

    nCheckSum = mstar_calculate_checksum(&pPacket[0], (nLength - 1));
    TS_LOG_DEBUG("checksum : [%x] == [%x]? \n", pPacket[nLength - 1], nCheckSum);

    if (pPacket[nLength - 1] != nCheckSum) {
        TS_LOG_ERR("WRONG CHECKSUM\n");
#if defined (CONFIG_HUAWEI_DSM)
        if (!dsm_client_ocuppy(ts_dclient)) {
            dsm_client_record(ts_dclient, "Mstar mutual_parse_packet WRONG CHECKSUM\n");
            dsm_client_notify(ts_dclient,DSM_TP_FW_CRC_ERROR_NO);
        }
#endif
        return -1;
    }

    if (g_mstar_dev_data->ts_platform_data->feature_info.wakeup_gesture_enable_info.switch_value == true) {

        if (g_gesture_wakeup_flag == 1) {
            u8 nWakeupMode = 0;
            u8 bIsCorrectFormat = 0;

            TS_LOG_DEBUG("received raw data from touch panel as following:\n");
            TS_LOG_DEBUG
                ("pPacket[0]=%x \n pPacket[1]=%x pPacket[2]=%x pPacket[3]=%x pPacket[4]=%x pPacket[5]=%x \n",
                 pPacket[0], pPacket[1], pPacket[2], pPacket[3], pPacket[4], pPacket[5]);

            if (pPacket[0] == 0xA7 && pPacket[1] == 0x00 && pPacket[2] == 0x06
                && pPacket[3] == PACKET_TYPE_GESTURE_WAKEUP) {
                nWakeupMode = pPacket[4];
                bIsCorrectFormat = 1;
            }
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
            else if (pPacket[0] == 0xA7 && pPacket[1] == 0x00 && pPacket[2] == 0x80
                 && pPacket[3] == PACKET_TYPE_GESTURE_DEBUG) {
                u32 a = 0;

                nWakeupMode = pPacket[4];
                bIsCorrectFormat = 1;

                for (a = 0; a < 0x80; a++) {
                    g_gesture_log_debug[a] = pPacket[a];
                }

                if (!(pPacket[5] >> 7)) // LCM Light Flag = 0
                {
                    nWakeupMode = 0xFE;
                    TS_LOG_DEBUG("gesture debug mode LCM flag = 0\n");
                }
            }
#endif

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
            else if (pPacket[0] == 0xA7 && pPacket[1] == 0x00 && pPacket[2] == 0x80
                 && pPacket[3] == PACKET_TYPE_GESTURE_INFORMATION) {
                u32 a = 0;
                u32 nTmpCount = 0;
                u32 nWidth = 0;
                u32 nHeight = 0;
                nWakeupMode = pPacket[4];
                bIsCorrectFormat = 1;

                for (a = 0; a < 6; a++) //header
                {
                    g_gesture_log_info[nTmpCount] = pPacket[a];
                    nTmpCount++;
                }

                for (a = 6; a < 126; a = a + 3) //parse packet to coordinate
                {
                    u32 nTranX = 0;
                    u32 nTranY = 0;

                    mstar_gesture_convert_coordinate(&pPacket[a], &nTranX, &nTranY);
                    g_gesture_log_info[nTmpCount] = nTranX;
                    nTmpBuffer[nGestureCount * 2] = nTranX;
                    nTmpCount++;
                    g_gesture_log_info[nTmpCount] = nTranY;
                    nTmpBuffer[nGestureCount * 2 + 1] = nTranY;
                    nTmpCount++;
                    nGestureCount++;
                }

                nWidth = (((pPacket[12] & 0xF0) << 4) | pPacket[13]);   //parse width & height
                nHeight = (((pPacket[12] & 0x0F) << 8) | pPacket[14]);

                TS_LOG_DEBUG("Before convert [width,height]=[%d,%d]\n", nWidth, nHeight);

                if ((pPacket[12] == 0xFF) && (pPacket[13] == 0xFF) && (pPacket[14] == 0xFF)) {
                    nWidth = 0;
                    nHeight = 0;
                } else {
                    nWidth = (nWidth * g_mstar_dev_data->x_max) / TPD_WIDTH;
                    nHeight = (nHeight * g_mstar_dev_data->y_max) / TPD_HEIGHT;
                    TS_LOG_DEBUG("After convert [width,height]=[%d,%d]\n", nWidth, nHeight);
                }

                g_gesture_log_info[10] = nWidth;
                g_gesture_log_info[11] = nHeight;

                g_gesture_log_info[nTmpCount] = pPacket[126];   //Dummy
                nTmpCount++;
                g_gesture_log_info[nTmpCount] = pPacket[127];   //checksum
                nTmpCount++;
                TS_LOG_DEBUG("gesture information mode Count = %d\n", nTmpCount);
            }
#endif

            if (bIsCorrectFormat) {
                TS_LOG_DEBUG("nWakeupMode = 0x%x\n", nWakeupMode);
                switch (nWakeupMode) {
                case 0x58:
                    TS_LOG_DEBUG("Light up screen by DOUBLE_CLICK gesture wakeup.\n");
                    if (IS_APP_ENABLE_GESTURE(GESTURE_DOUBLE_CLICK) &&
                        g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture) {
                        nReportGestureType = TS_DOUBLE_CLICK;
                        LOG_JANK_D(JLID_TP_GESTURE_KEY, "JL_TP_GESTURE_KEY");
                        nGestureCount = LINEAR_LOCUS_NUM;
                    }
                    break;

                case 0x64:
                    TS_LOG_DEBUG("GESTURE_WAKEUP_MODE_M_CHARACTER_FLAG.\n");
                    if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_m) &&
                        g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture) {
                        nReportGestureType = TS_LETTER_m;
                        nGestureCount = LETTER_LOCUS_NUM;
                    }
                    break;

                case 0x65:
                    TS_LOG_DEBUG("GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG.\n");
                    if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_w) &&
                        g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture) {
                        nReportGestureType = TS_LETTER_c;
                        nGestureCount = LETTER_LOCUS_NUM;
                    }
                    break;

                case 0x66:
                    TS_LOG_DEBUG("GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG.\n");

                    if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_c) &&
                        g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture) {
                        nReportGestureType = TS_LETTER_c;
                        nGestureCount = LETTER_LOCUS_NUM;
                    }
                    break;

                case 0x67:
                    TS_LOG_DEBUG("GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG.\n");
                    if (IS_APP_ENABLE_GESTURE(GESTURE_LETTER_e) &&
                        g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture) {
                        nReportGestureType = TS_LETTER_e;
                        nGestureCount = LETTER_LOCUS_NUM;
                    }
                    break;

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
                case 0xFF:
                    TS_LOG_INFO("Light up screen by GESTURE_WAKEUP_MODE_FAIL gesture wakeup.\n");
                    break;
#endif
                default:
                    TS_LOG_INFO("Un-supported gesture wakeup mode. Please check your device driver code.\n");
                    break;
                }
            } else {
                TS_LOG_ERR("gesture wakeup packet format is incorrect.\n");
            }

            if (nReportGestureType != 0) {
                wake_lock_timeout(&g_mstar_dev_data->ts_platform_data->ts_wake_lock, 5 * HZ);
                //mutex_lock(&wrong_gesture_lock);

                if (true == g_mstar_dev_data->easy_wakeup_info.off_motion_on) {
                    g_mstar_dev_data->easy_wakeup_info.off_motion_on = false;
                    mstar_easy_wakeup_gesture_report_coordinate(nTmpBuffer, nGestureCount);
                    info->gesture_wakeup_value = nReportGestureType;
                    TS_LOG_DEBUG("%s: info->gesture_wakeup_value = %d\n", __func__,
                            info->gesture_wakeup_value);
                }
                return 0;
                //mutex_unlock(&wrong_gesture_lock);
            }
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
            // Notify android application to retrieve log data mode packet from device driver by sysfs.
            if (g_GestureKObj != NULL && pPacket[3] == PACKET_TYPE_GESTURE_DEBUG) {
                char *pEnvp[2];
                s32 nRetVal = 0;

                pEnvp[0] = "STATUS=GET_GESTURE_DEBUG";
                pEnvp[1] = NULL;

                nRetVal = kobject_uevent_env(g_GestureKObj, KOBJ_CHANGE, pEnvp);
                TS_LOG_DEBUG("kobject_uevent_env() STATUS=GET_GESTURE_DEBUG, nRetVal = %d\n", nRetVal);
            }
#endif

            return -1;
        }
    }

    if (IS_FIRMWARE_DATA_LOG_ENABLED) {
        if (g_fw_mode == FIRMWARE_MODE_DEMO_MODE && pPacket[0] != 0x5A) {
            TS_LOG_DEBUG("WRONG DEMO MODE HEADER\n");
            return -1;
        } else if (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE
               && (pPacket[0] != 0xA7 && pPacket[3] != PACKET_TYPE_TOOTH_PATTERN)) {
            TS_LOG_INFO("WRONG DEBUG MODE HEADER\n");
            return -1;
        }
    } else {
        if (pPacket[0] != 0x5A) {
            TS_LOG_INFO("WRONG DEMO MODE HEADER\n");
            return -1;
        }
    }

    // Process raw data...
    if (pPacket[0] == 0x5A) {
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
        u8 nButton = pPacket[nLength-2]; //Since the key value is stored in 0th~3th bit of variable "button", we can only retrieve 0th~3th bit of it.

        if (nButton != 0xFF)
        {
            TS_LOG_INFO("proximity enable mode = %d, pPacket[nLength-2] = 0x%x\n", ps_info->proximity_enable, pPacket[nLength-2]);

            if (ps_info->proximity_enable && ((pPacket[nLength-2] == 0x80) || (pPacket[nLength-2] == 0x40)))
            {
                if (pPacket[nLength-2] == 0x80) // close to
                {
                    ps_info->proximity_status = 0;
                }
                else if (pPacket[nLength-2] == 0x40) // far away
                {
                    ps_info->proximity_status = 1;
                }
                TS_LOG_DEBUG("mstar proximity status = %d\n", ps_info->proximity_status);
                return -1;
            }
        }
#endif
        for (i = 0; i < MAX_TOUCH_NUM; i++) {
            if ((pPacket[(4 * i) + 1] == 0xFF) && (pPacket[(4 * i) + 2] == 0xFF)
                && (pPacket[(4 * i) + 3] == 0xFF)) {
#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
                _gCurrentTouch[i] = 0;
#endif
                continue;
            }

            nX = (((pPacket[(4 * i) + 1] & 0xF0) << 4) | (pPacket[(4 * i) + 2]));
            nY = (((pPacket[(4 * i) + 1] & 0x0F) << 8) | (pPacket[(4 * i) + 3]));

            pInfo->tPoint[pInfo->nCount].nX = nX * g_mstar_dev_data->x_max / TPD_WIDTH;
            pInfo->tPoint[pInfo->nCount].nY = nY * g_mstar_dev_data->y_max / TPD_HEIGHT;
            pInfo->tPoint[pInfo->nCount].nP = pPacket[4 * (i + 1)];
            pInfo->tPoint[pInfo->nCount].nId = i;

            info->fingers[i].status = TP_FINGER;
            info->fingers[i].x = nX * g_mstar_dev_data->x_max / TPD_WIDTH;
            info->fingers[i].y = nY * g_mstar_dev_data->y_max / TPD_HEIGHT;
            //info->fingers[i].major = pPacket[4*(i+1)];
            //info->fingers[i].minor = pPacket[4*(i+1)];
            info->fingers[i].pressure = pPacket[4 * (i + 1)];

            TS_LOG_DEBUG("[x,y]=[%d,%d]\n", nX, nY);
            TS_LOG_DEBUG("point[%d] : (%d,%d) = %d\n", pInfo->tPoint[pInfo->nCount].nId,
                    pInfo->tPoint[pInfo->nCount].nX, pInfo->tPoint[pInfo->nCount].nY,
                    pInfo->tPoint[pInfo->nCount].nP);

            pInfo->nCount++;
            touch_count = i + 1;
#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
            _gCurrentTouch[i] = 1;
#endif
        }

        if (IS_FIRMWARE_DATA_LOG_ENABLED) {
            // Notify android application to retrieve demo mode packet from device driver by sysfs.
            if (g_touch_kobj != NULL) {
                char szRspDemoModePacket[512] = { 0 };
                char szValue[3] = { 0 };
                char *pEnvp[3];
                s32 nRetVal = 0;

                strcat(szRspDemoModePacket, "VALUE=");

                for (i = 0; i < nLength; i++) {
                    sprintf(szValue, "%02x", pPacket[i]);
                    strcat(szRspDemoModePacket, szValue);
                }

                pEnvp[0] = "STATUS=GET_DEMO_MODE_PACKET";
                pEnvp[1] = szRspDemoModePacket;
                pEnvp[2] = NULL;
                TS_LOG_DEBUG("pEnvp[1] = %s\n", pEnvp[1]);  // TODO : add for debug

                nRetVal = kobject_uevent_env(g_touch_kobj, KOBJ_CHANGE, pEnvp);
                TS_LOG_DEBUG("kobject_uevent_env() STATUS=GET_DEMO_MODE_PACKET, nRetVal = %d\n",
                        nRetVal);
            }
        }
    } else if (pPacket[0] == 0xA7 && pPacket[3] == PACKET_TYPE_TOOTH_PATTERN) {
        for (i = 0; i < MAX_TOUCH_NUM; i++) {
            if ((pPacket[(3 * i) + 5] == 0xFF) && (pPacket[(3 * i) + 6] == 0xFF)
                && (pPacket[(3 * i) + 7] == 0xFF)) {
#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
                _gCurrentTouch[i] = 0;
#endif

                continue;
            }

            nX = (((pPacket[(3 * i) + 5] & 0xF0) << 4) | (pPacket[(3 * i) + 6]));
            nY = (((pPacket[(3 * i) + 5] & 0x0F) << 8) | (pPacket[(3 * i) + 7]));

            pInfo->tPoint[pInfo->nCount].nX = nX * g_mstar_dev_data->x_max / TPD_WIDTH;
            pInfo->tPoint[pInfo->nCount].nY = nY * g_mstar_dev_data->y_max / TPD_HEIGHT;
            pInfo->tPoint[pInfo->nCount].nP = 1;
            pInfo->tPoint[pInfo->nCount].nId = i;

            info->fingers[i].status = TP_FINGER;
            info->fingers[i].x = nX * g_mstar_dev_data->x_max / TPD_WIDTH;
            info->fingers[i].y = nY * g_mstar_dev_data->y_max / TPD_HEIGHT;
            //info->fingers[i].major = pPacket[4*(i+1)];
            //info->fingers[i].minor = pPacket[4*(i+1)];
            info->fingers[i].pressure = pPacket[4 * (i + 1)];
            touch_count = i + 1;
            TS_LOG_DEBUG("[x,y]=[%d,%d]\n", nX, nY);
            TS_LOG_DEBUG("point[%d] : (%d,%d) = %d\n", pInfo->tPoint[pInfo->nCount].nId,
                    pInfo->tPoint[pInfo->nCount].nX, pInfo->tPoint[pInfo->nCount].nY,
                    pInfo->tPoint[pInfo->nCount].nP);

            pInfo->nCount++;

#ifdef CONFIG_ENABLE_TYPE_B_PROTOCOL
            _gCurrentTouch[i] = 1;
#endif
        }

        // Notify android application to retrieve debug mode packet from device driver by sysfs.
        if (g_touch_kobj != NULL) {
            char *pEnvp[2];
            s32 nRetVal = 0;

            memcpy(g_log_packet, g_debug_packet, nLength);  // Copy g_debug_packet to g_log_packet for avoiding the debug mode data which is received by MTPTool APK may be modified.

            pEnvp[0] = "STATUS=GET_DEBUG_MODE_PACKET";
            pEnvp[1] = NULL;

            nRetVal = kobject_uevent_env(g_touch_kobj, KOBJ_CHANGE, pEnvp);
            TS_LOG_DEBUG("kobject_uevent_env() STATUS=GET_DEBUG_MODE_PACKET, nRetVal = %d\n", nRetVal);
        }
    }

    info->cur_finger_number = touch_count;
    return 0;
}

void mstar_enable_gesture_wakeup(u32 * pMode)
{
    u8 szTxData[4] = { 0 };
    u32 i = 0;
    s32 rc;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    struct ts_easy_wakeup_info *info = &g_mstar_dev_data->easy_wakeup_info;
    TS_LOG_DEBUG("mstar_enable_gesture_wakeup  info->easy_wakeup_flag =%x \n", info->easy_wakeup_flag);

    /*if the sleep_gesture_flag is ture,it presents that  the tp is at sleep state */
    if (g_mstar_dev_data->ts_platform_data->feature_info.wakeup_gesture_enable_info.switch_value == false ||
        true == info->easy_wakeup_flag) {
        TS_LOG_INFO("mstar_enable_gesture_wakeup  info->easy_wakeup_flag =%x \n", info->easy_wakeup_flag);
        return;
    }
    TS_LOG_INFO("wakeup mode 0 = 0x%x\n", pMode[0]);
    TS_LOG_INFO("wakeup mode 1 = 0x%x\n", pMode[1]);

#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
    szTxData[0] = 0x59;
    szTxData[1] = 0x00;
    szTxData[2] = ((pMode[1] & 0xFF000000) >> 24);
    szTxData[3] = ((pMode[1] & 0x00FF0000) >> 16);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 4);
        if (rc > 0) {
            TS_LOG_INFO("Enable gesture wakeup index 0 success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Enable gesture wakeup index 0 failed\n");
    }

    szTxData[0] = 0x59;
    szTxData[1] = 0x01;
    szTxData[2] = ((pMode[1] & 0x0000FF00) >> 8);
    szTxData[3] = ((pMode[1] & 0x000000FF) >> 0);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 4);
        if (rc > 0) {
            TS_LOG_INFO("Enable gesture wakeup index 1 success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Enable gesture wakeup index 1 failed\n");
    }

    szTxData[0] = 0x59;
    szTxData[1] = 0x02;
    szTxData[2] = ((pMode[0] & 0xFF000000) >> 24);
    szTxData[3] = ((pMode[0] & 0x00FF0000) >> 16);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 4);
        if (rc > 0) {
            TS_LOG_INFO("Enable gesture wakeup index 2 success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Enable gesture wakeup index 2 failed\n");
    }

    szTxData[0] = 0x59;
    szTxData[1] = 0x03;
    szTxData[2] = ((pMode[0] & 0x0000FF00) >> 8);
    szTxData[3] = ((pMode[0] & 0x000000FF) >> 0);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 4);
        if (rc > 0) {
            TS_LOG_INFO("Enable gesture wakeup index 3 success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Enable gesture wakeup index 3 failed\n");
    }

    mutex_lock(&g_mutex_protect);
    g_gesture_wakeup_flag = 1;  // gesture wakeup is enabled
    mutex_unlock(&g_mutex_protect);
#else

    szTxData[0] = 0x58;
    szTxData[1] = ((pMode[0] & 0x0000FF00) >> 8);
    szTxData[2] = ((pMode[0] & 0x000000FF) >> 0);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);   // delay 20ms
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            TS_LOG_INFO("Enable gesture wakeup success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Enable gesture wakeup failed\n");
    }

    mutex_lock(&g_mutex_protect);
    g_gesture_wakeup_flag = 1;  // gesture wakeup is enabled
    mutex_unlock(&g_mutex_protect);
#endif //CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
    info->easy_wakeup_flag = true;
    return;
}

static void mstar_put_device_outof_easy_wakeup(void)
{
    int retval = 0;
    struct ts_easy_wakeup_info *info = &g_mstar_dev_data->easy_wakeup_info;

    TS_LOG_DEBUG("mstar_put_device_outof_easy_wakeup  info->easy_wakeup_flag =%d\n", info->easy_wakeup_flag);

    if (false == info->easy_wakeup_flag) {
        return;
    }
    info->easy_wakeup_flag = false;
    return;
}

void mstar_disable_gesture_wakeup(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    mutex_lock(&g_mutex_protect);
    g_gesture_wakeup_flag = 0;  // gesture wakeup is disabled
    mutex_unlock(&g_mutex_protect);
}

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
void mstar_gesture_open_debug(u8 gesture_flag)
{
    u8 szTxData[3] = { 0 };
    s32 rc;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    TS_LOG_INFO("Gesture Flag = 0x%x\n", gesture_flag);

    szTxData[0] = 0x30;
    szTxData[1] = 0x01;
    szTxData[2] = gesture_flag;

    mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
    rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
    if (rc < 0) {
        TS_LOG_INFO("Enable gesture debug mode failed\n");
    } else {
        g_gesture_debug_mode = 1;   // gesture debug mode is enabled

        TS_LOG_INFO("Enable gesture debug mode success\n");
    }
}

void mstar_gesture_close_debug(void)
{
    u8 szTxData[3] = { 0 };
    s32 rc;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    szTxData[0] = 0x30;
    szTxData[1] = 0x00;
    szTxData[2] = 0x00;

    mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
    rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
    if (rc < 0) {
        TS_LOG_INFO("Disable gesture debug mode failed\n");
    } else {
        g_gesture_debug_mode = 0;   // gesture debug mode is disabled

        TS_LOG_INFO("Disable gesture debug mode success\n");
    }
}
#endif

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static void mstar_gesture_convert_coordinate(u8 * pRawData, u32 * pTranX, u32 * pTranY)
{
    u32 nX;
    u32 nY;
#ifdef CONFIG_SWAP_X_Y
    u32 nTempX;
    u32 nTempY;
#endif

    nX = (((pRawData[0] & 0xF0) << 4) | pRawData[1]);   // parse the packet to coordinate
    nY = (((pRawData[0] & 0x0F) << 8) | pRawData[2]);

    TS_LOG_DEBUG("[x,y]=[%d,%d]\n", nX, nY);

#ifdef CONFIG_SWAP_X_Y
    nTempY = nX;
    nTempX = nY;
    nX = nTempX;
    nY = nTempY;
#endif

#ifdef CONFIG_REVERSE_X
    nX = 2047 - nX;
#endif

#ifdef CONFIG_REVERSE_Y
    nY = 2047 - nY;
#endif

    /*
     * pRawData[0]~pRawData[2] : the point abs,
     * pRawData[0]~pRawData[2] all are 0xFF, release touch
     */
    if ((pRawData[0] == 0xFF) && (pRawData[1] == 0xFF) && (pRawData[2] == 0xFF)) {
        *pTranX = 0;    // final X coordinate
        *pTranY = 0;    // final Y coordinate
        TS_LOG_INFO("gesture released, final coordinate\n");
    } else {
        /* one touch point */
        *pTranX = (nX * g_mstar_dev_data->x_max) / TPD_WIDTH;
        *pTranY = (nY * g_mstar_dev_data->y_max) / TPD_HEIGHT;
        TS_LOG_INFO("[%s]: [x,y]=[%d,%d], point[x,y]=[%d,%d]\n", __func__, nX, nY, *pTranX, *pTranY);
    }
}

static void mstar_easy_wakeup_gesture_report_coordinate(u32 * buf, u32 count)
{
    int x = 0;
    int y = 0, i = 0;

    if (count != 0) {
        if (count == 2) {
            for (i = 0; i < count; i++) {
                x = (s16) (buf[i * 2 + 0]);
                y = (s16) (buf[i * 2 + 1]);

                TS_LOG_DEBUG("%s: Gesture Repot Point %d:\n" "x = %d\n" "y = %d\n", __func__, i, x, y);
                g_mstar_dev_data->easy_wakeup_info.easywake_position[i] = x << 16 | y;
                TS_LOG_DEBUG("easywake_position[%d] = 0x%04x\n", i,
                         g_mstar_dev_data->easy_wakeup_info.easywake_position[i]);
            }
        } else {
            /*1.beginning */
            x = (s16) (buf[0]);
            y = (s16) (buf[1]);
            g_mstar_dev_data->easy_wakeup_info.easywake_position[0] = x << 16 | y;
            TS_LOG_INFO("easywake_position[1]  beginning= 0x%04x x = %d, y =%d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[0], x, y);

            /*2.end */
            x =  (s16)  buf[2];
            y =  (s16)  buf[3];
            g_mstar_dev_data->easy_wakeup_info.easywake_position[1] = x << 16 | y;
            TS_LOG_INFO("easywake_position[1]  end = 0x%08x,  x= %d , y= %d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[1], x, y);

            /*3.top */
            x =  (s16)  buf[6];
            y =  (s16)  buf[7];
            g_mstar_dev_data->easy_wakeup_info.easywake_position[2] = x << 16 | y;
            TS_LOG_INFO("easywake_position[2]  top = 0x%08x,  top_x= %d , top_y= %d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[2], x, y);

            /*4.leftmost */
            x =  (s16)  buf[8];
            y =  (s16)  buf[9];
            g_mstar_dev_data->easy_wakeup_info.easywake_position[3] = x << 16 | y;
            TS_LOG_INFO("easywake_position[3]  leftmost = 0x%08x,  left_x= %d , left_y= %d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[3], x, y);

            /*5.bottom */
            x =  (s16)  buf[10];
            y =  (s16)  buf[11];
            g_mstar_dev_data->easy_wakeup_info.easywake_position[4] = x << 16 | y;
            TS_LOG_INFO("easywake_position[4]  bottom = 0x%08x,  x= %d , y= %d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[4], x, y);

            /*6.rightmost */
            x =  (s16)  buf[12];
            y =  (s16)  buf[13];
            g_mstar_dev_data->easy_wakeup_info.easywake_position[5] = x << 16 | y;
            TS_LOG_INFO("easywake_position[5]  rightmost = 0x%08x,  x= %d , y= %d \n",
                    g_mstar_dev_data->easy_wakeup_info.easywake_position[5], x, y);
        }
    }

}
#endif

char *mstar_strncat(unsigned char *dest, char *src, size_t dest_size)
{
    size_t dest_len = 0;
    char *start_index = NULL;
    dest_len = strnlen(dest, dest_size);
    start_index = dest + dest_len;
    return strncat(&dest[dest_len], src, dest_size - dest_len - 1);
}

void mstar_set_sensor_test_result(struct ts_rawdata_info *info)
{
    int str_cat_len = 32;
    char tmp_char[32] = { 0 };

    if (mp_test_info.i2c_status == NO_ERR) {
        mstar_strncat(info->result, MSTAR_CONNECT_TEST_PASS, TS_RAWDATA_RESULT_MAX);
    } else {
        mstar_strncat(info->result, MSTAR_CONNECT_TEST_FAIL, TS_RAWDATA_RESULT_MAX);
    }
    if (mp_test_info.allnode_test_result) {
        mstar_strncat(info->result, MSTAR_ALLNODE_TEST_PASS, TS_RAWDATA_RESULT_MAX);
    } else {
        mstar_strncat(info->result, MSTAR_ALLNODE_TEST_FAIL, TS_RAWDATA_RESULT_MAX);
    }
    if (mp_test_info.open_test_result) {
        mstar_strncat(info->result, MSTAR_OPEN_PASS, TS_RAWDATA_RESULT_MAX);
    } else {
        mstar_strncat(info->result, MSTAR_OPEN_FAIL, TS_RAWDATA_RESULT_MAX);
    }
    if (mp_test_info.short_test_result) {
        mstar_strncat(info->result, MSTAR_SHORT_TEST_PASS, TS_RAWDATA_RESULT_MAX);
    } else {
        mstar_strncat(info->result, MSTAR_SHORT_TEST_FAIL, TS_RAWDATA_RESULT_MAX);
    }

    mstar_strncat(info->result, MSTAR_SHORT_2_TEST_PASS, TS_RAWDATA_RESULT_MAX);
}

int mstar_get_raw_data(struct ts_rawdata_info *info, struct ts_cmd_node *out_cmd)
{
    int i = 0;
    int j = 0;
    int ret = -1;
    unsigned char buf[4] = { 0 };
    int buff_index = 0;
    u16 CH_X = 13, CH_Y = 25, nMajor = 0, nMinor = 0;

    mp_test_info.i2c_status = NO_ERR;
    mp_test_info.allnode_test_result = 0;
    mp_test_info.open_test_result = 0;
    mp_test_info.short_test_result = 0;

    mp_test_info.DeltaCData = kcalloc(CH_X * CH_Y, sizeof(s32), GFP_KERNEL);
    mp_test_info.RationData = kcalloc(CH_X * CH_Y, sizeof(s32), GFP_KERNEL);
    mp_test_info.GoldenData = kcalloc(CH_X * CH_Y, sizeof(s32), GFP_KERNEL);
    mp_test_info.ShortData = kcalloc(CH_X + CH_Y, sizeof(s32), GFP_KERNEL);

    if (ERR_ALLOC_MEM(mp_test_info.DeltaCData) || ERR_ALLOC_MEM(mp_test_info.RationData) ||
        ERR_ALLOC_MEM(mp_test_info.GoldenData) || ERR_ALLOC_MEM(mp_test_info.ShortData)) {
        TS_LOG_ERR("Failed to allocate MP mem\n");
        ret = -1;
        goto out;
    }
    ret = mstar_mp_test_entry(g_chip_type, NULL);
    TS_LOG_INFO("MP Test Result = %d \n", ret);
    if (ret == 1 || ret == 2) {
        mstar_mp_test_save_data(g_chip_type, ret);
    } else {
        TS_LOG_ERR("MP Test got unknown failure...won't save data as CSV\n");
    }

    if (ret != ITO_TEST_OK) {
        if (strcmp(DEVICE_FIRMWARE_VERSION, g_fw_cust_ver)) {
            TS_LOG_INFO("please check your firmware version : %s \n", g_fw_cust_ver);
        }
    }

    mstar_mp_test_entry_end(g_chip_type);
    TS_LOG_INFO("*** MP Test End ***\n");

    mstar_set_sensor_test_result(info);

    info->buff[buff_index++] = CH_X;
    info->buff[buff_index++] = CH_Y;

    for (i = 0; i < CH_X * CH_Y; i++) {
        info->buff[buff_index++] = mp_test_info.DeltaCData[i];
    }
    for (i = 0; i < CH_X * CH_Y; i++) {
        info->buff[buff_index++] = mp_test_info.GoldenData[i];
    }
    for (i = 0; i < CH_X * CH_Y; i++) {
        info->buff[buff_index++] = mp_test_info.RationData[i];
    }
    for (i = 0; i < CH_X + CH_Y; i++) {
        info->buff[buff_index++] = mp_test_info.ShortData[i];
    }

    info->used_size = buff_index;
    TS_LOG_INFO("%s, buff_index = %d\n", __func__, buff_index);
    ret = 0;

out:
    if (!ERR_ALLOC_MEM(mp_test_info.DeltaCData)) {
        kfree(mp_test_info.DeltaCData);
        mp_test_info.DeltaCData = NULL;
    }
    if (!ERR_ALLOC_MEM(mp_test_info.RationData)) {
        kfree(mp_test_info.RationData);
        mp_test_info.RationData = NULL;
    }
    if (!ERR_ALLOC_MEM(mp_test_info.GoldenData)) {
        kfree(mp_test_info.GoldenData);
        mp_test_info.GoldenData = NULL;
    }
    if (!ERR_ALLOC_MEM(mp_test_info.ShortData)) {
        kfree(mp_test_info.ShortData);
        mp_test_info.ShortData = NULL;
    }
    return ret;
}

static int mstar_get_glove_switch(unsigned char *mode)
{
    TS_LOG_INFO("%s:", __func__);
    mstar_apknode_get_glove_info(mode);
    //*mode = g_IsEnableGloveMode;
    return 0;
}

static int mstar_set_glove_switch(unsigned char mode)
{
    int ret = 0;

    mstar_finger_touch_report_disable();
    if (mode == 1) {
        mstar_apknode_open_glove();
    } else {
        mstar_apknode_close_glove();
    }

    mstar_finger_touch_report_enable();
    return 0;
}

static int mstar_glove_switch(struct ts_glove_info *info)
{
    int ret = 0;

    if (!info) {
        TS_LOG_ERR("%s:info is null\n", __func__);
        ret = -ENOMEM;
        return ret;
    }

    switch (info->op_action) {
    case TS_ACTION_READ:
        ret = mstar_get_glove_switch(&info->glove_switch);
        if (ret) {
            TS_LOG_ERR("%s:get glove switch fail,ret=%d\n", __func__, ret);
            return ret;
        } else {
            TS_LOG_INFO("%s:glove switch=%d\n", __func__, info->glove_switch);
        }

        break;
    case TS_ACTION_WRITE:
        TS_LOG_INFO("%s:glove switch=%d\n", __func__, info->glove_switch);
        ret = mstar_set_glove_switch(! !info->glove_switch);
        if (ret) {
            TS_LOG_ERR("%s:set glove switch fail, ret=%d\n", __func__, ret);
            return ret;
        }

        break;
    default:
        TS_LOG_ERR("%s:invalid op action:%d\n", __func__, info->op_action);
        return -EINVAL;
    }

    return 0;
}

int mstar_schedule_normal(void)
{
    int error = NO_ERR;

    error = ts_kit_power_control_notify(TS_AFTER_RESUME, NO_SYNC_TIMEOUT);
    if (error) {
        TS_LOG_ERR("ts after resume err\n");
    }

    return 0;
}

int mstar_schedule_gesture(void)
{
    int error = NO_ERR;

    mstar_put_device_outof_easy_wakeup();

    //mstar_dev_hw_reset();

    error = ts_kit_power_control_notify(TS_AFTER_RESUME, NO_SYNC_TIMEOUT);
    if (error) {
        TS_LOG_ERR("ts after resume err\n");
    }

    return 0;
}

static int mstar_set_info_flag(struct ts_kit_device_data *info)
{
    return NO_ERR;
}

static void mstar_get_project_info(void)
{
    u8 temp_buf[4] = {0};

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    mstar_mp_read_flash(g_chip_type, MSTAR_PROJECT_ID_ADDR, EMEM_TYPE_INFO_BLOCK, 12, (u8*)mstar_project_id);
    mstar_project_id[MSTAR_PROJECT_ID_LEN-2] = '\0';
    TS_LOG_INFO("project id: %s\n", mstar_project_id);

    if (mstar_support_get_tp_color) {
        mstar_mp_read_flash(g_chip_type, MSTAR_COLOR_ID_ADDR, EMEM_TYPE_INFO_BLOCK, 4, temp_buf);
        cypress_ts_kit_color[0] = temp_buf[3];
        TS_LOG_INFO("color id: 0x%x,0x%x\n", temp_buf[2], temp_buf[3]);
    }

    mstar_mp_read_flash(g_chip_type, MSTAR_GOLDEN_VER_ADDR, EMEM_TYPE_INFO_BLOCK, 4, temp_buf);
    mstar_golden_sample_version[0] = temp_buf[2];
    mstar_golden_sample_version[1] = temp_buf[3];
    TS_LOG_INFO("golden sample version: 0x%x,0x%x\n", mstar_golden_sample_version[0], mstar_golden_sample_version[1]);

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    mstar_vendor_id = mstar_get_swid(EMEM_INFO);
    mstar_dev_hw_reset();
    TS_LOG_INFO("module vendor id: 0x%x\n", mstar_vendor_id);
}

static int mstar_before_suspend(void)
{
    return NO_ERR;
}

int mstar_suspend(void)
{
    TS_LOG_INFO("mstar_suspend \n");

    if (g_mstar_dev_data->need_wd_check_status) {
        g_esd_enable = 0;
    }
#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    g_charge_enable = 0;
#endif

    if (g_fw_update != 0)   // check whether update frimware is finished
    {
        TS_LOG_INFO("Not allow to power on/off touch ic while update firmware.\n");
        goto out;
    }
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_mstar_dev_data->ts_platform_data->feature_info.proximity_info.proximity_enable == 1) {
        TS_LOG_DEBUG("proximity enable mode = %d\n", g_mstar_dev_data->ts_platform_data->feature_info.proximity_info.proximity_enable);
        goto out;
    }
#endif

    mstar_suspend_flag = true;
    switch (g_mstar_dev_data->easy_wakeup_info.sleep_mode) {
    case TS_POWER_OFF_MODE:
        mstar_enter_sleep_mode();
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
        mstar_regulator_power_switch(false);
#endif
        TS_LOG_INFO("suspend end case TS_POWER_OFF_MODE");
        break;

    case TS_GESTURE_MODE:
        if (g_mstar_dev_data->ts_platform_data->feature_info.wakeup_gesture_enable_info.switch_value == true) {
            TS_LOG_INFO("wakeup_gesture_enable_info.switch_value = true \n");
            if (g_gesture_wakeup_mode[0] != 0x00000000 || g_gesture_wakeup_mode[1] != 0x00000000) {
                enable_irq_wake(g_mstar_dev_data->ts_platform_data->irq_id);
                mstar_enable_gesture_wakeup(&g_gesture_wakeup_mode[0]);
            }
            g_mstar_dev_data->easy_wakeup_info.off_motion_on = true;
            TS_LOG_INFO("susspend end case TS_GESTURE_MODE");
        } else {
            mstar_enter_sleep_mode();
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
            mstar_regulator_power_switch(false);
#endif
        }
        break;

    default:
        break;
    }

out:
    TS_LOG_INFO("suspend end");
    return NO_ERR;
}

/*  do some thing after power on. */
int mstar_after_resume(void *feature_info)
{
    int ret = NO_ERR;

    TS_LOG_INFO("mstar_after_resume +\n");
    if (g_fw_update != 0)   // check whether update frimware is finished
    {
        TS_LOG_INFO("Not allow to power on/off touch ic while update firmware.\n");
        mstar_suspend_flag = false;
        goto out;
    }

    switch (g_mstar_dev_data->easy_wakeup_info.sleep_mode) {
    case TS_POWER_OFF_MODE:
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
        mstar_regulator_power_switch(true);
#endif
        break;

    case TS_GESTURE_MODE:
        if (g_mstar_dev_data->ts_platform_data->feature_info.wakeup_gesture_enable_info.switch_value == true) {
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
            if (g_gesture_debug_mode == 1) {
                mstar_gesture_close_debug();
            }
#endif
        } else {
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
        mstar_regulator_power_switch(true);
#endif
        }

        if (g_gesture_wakeup_flag == 1) {
            TS_LOG_INFO("g_GestureState = %d, g_chip_type = 0x%X\n", ans.g_GestureState, g_chip_type);
            mstar_disable_gesture_wakeup();
        }
        break;

    default:
        break;
    }

    mstar_exit_sleep_mode();
    mstar_suspend_flag = false;

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    {
        u8 szChargerStatus[20] = { 0 };

        mstar_read_file(POWER_SUPPLY_BATTERY_STATUS_PATCH, szChargerStatus, 20);

        TS_LOG_INFO("*** Battery Status : %s ***\n", szChargerStatus);

        g_force_update = 1; // Set flag to force update charger status

        if (strstr(szChargerStatus, "Charging") != NULL || strstr(szChargerStatus, "Full") != NULL || strstr(szChargerStatus, "Fully charged") != NULL) // Charging
        {
            mstar_charge_detect(1); // charger plug-in
        } else      // Not charging
        {
            mstar_charge_detect(0); // charger plug-out
        }

        g_force_update = 0; // Clear flag after force update charger status
    }
#endif

    if (ans.g_IsEnableGloveMode == 1) {
        mstar_apknode_open_glove();
    }

    if (ans.g_IsEnableLeatherSheathMode == 1) {
        mstar_apknode_open_leather_sheath();
    }

    if (IS_FIRMWARE_DATA_LOG_ENABLED) {
        // mark this function call for avoiding device driver may spend longer time to resume from suspend state.
        mstar_restore_fw_to_log_data();
    }
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (g_mstar_dev_data->ts_platform_data->feature_info.proximity_info.proximity_enable == 1) {
        TS_LOG_INFO("resume proximity enable mode = %d restore enable proximity\n",
            g_mstar_dev_data->ts_platform_data->feature_info.proximity_info.proximity_enable);
        mstar_proximity_enable(1);
    }
#endif
    //mstar_finger_touch_report_enable();

#ifdef CONFIG_ENABLE_CHARGER_DETECTION
    g_charge_enable = 1;
#endif

    if (g_mstar_dev_data->need_wd_check_status)
        g_esd_enable = 1;
out:
    TS_LOG_INFO("mstar_after_resume -\n");
    return ret;
}

int mstar_resume(void)
{
    int i = 0;
    int ret = NO_ERR;
    unsigned char buf[3] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);
    switch (g_mstar_dev_data->easy_wakeup_info.sleep_mode) {
    case TS_POWER_OFF_MODE:
        schedule_work(&mstar_resume_normal_work);
        break;

    case TS_GESTURE_MODE:
        //mstar_finger_touch_release();
        schedule_work(&mstar_resume_gesture_work);
        break;

    default:
        TS_LOG_ERR("no resume mode\n");
        return ret;
    }

    return 0;
}

static int mstar_wakeup_gesture_enable_switch(struct ts_wakeup_gesture_enable_info *info)
{
    return NO_ERR;
}

static int mstar_get_brightness_info(void)
{
    int error = NO_ERR;

    return error;
}

int mstar_fw_update_boot(char *file_name)
{
    int ret = 0;
    u16 major = 0, minor = 0;

    ret = mstar_fw_update_swid_entry();

    mstar_get_customer_fw_ver(&major, &minor, &g_fw_cust_ver);
    sprintf(g_mstp_vendor,"%s,msg28xxa,fw=%x.%x", g_mstar_dev_data->module_name, major, (minor&0xFF));

    return ret;
}

static int mstar_fw_update_sd(void)
{
    int ret = 0;
    u16 major = 0, minor = 0;

    ret = mstar_fw_update_sdcard(FIRMWARE_FILE_PATH_FOR_HW, 0);
    if (ret < 0) {
        TS_LOG_ERR("Failed to update fw sd err: %d\n", ret);
        return ret;
    }
    else {
        mstar_get_customer_fw_ver(&major, &minor, &g_fw_cust_ver);
        sprintf(g_mstp_vendor,"%s,msg28xxa,fw=%x.%x", g_mstar_dev_data->module_name, major, (minor&0xFF));
    }

    return ret;
}

int mstar_irq_bottom_half(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
    struct algo_param *algo_p = NULL;
    struct ts_fingers *info = NULL;
    int ret = 0;
    unsigned long nIrqFlag;
    algo_p = &out_cmd->cmd_param.pub_params.algo_param;
    info = &algo_p->info;

    out_cmd->command = TS_INPUT_ALGO;
    algo_p->algo_order = g_mstar_dev_data->algo_id;

    TS_LOG_DEBUG("%s:algo_order:%d\n", __func__, algo_p->algo_order);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
    if (g_mp_test == 0) {
#endif
        mstar_read_touch_data(info);
#ifdef CONFIG_ENABLE_ITO_MP_TEST
    }
#endif

    TS_LOG_DEBUG("*** %s() g_int_flag = %d ***\n", __func__, g_int_flag);

    spin_lock_irqsave(&g_irq_lock, nIrqFlag);

    if (g_int_flag == 0
#ifdef CONFIG_ENABLE_ITO_MP_TEST
        && g_mp_test == 0
#endif
    ) {

        g_int_flag = 1;
    }

    spin_unlock_irqrestore(&g_irq_lock, nIrqFlag);

    return ret;
}

int mstar_power_init(void)
{
    int retval;

    /*set TP volt */
    retval = regulator_set_voltage(g_mstar_dev_data->ts_platform_data->vdd, 2800000, 2800000);
    if (retval != 0) {
        TS_LOG_ERR("[%s]Failed to set voltage of regulator,ret=%d!", __func__, retval);
        return retval;
    }

    retval = regulator_enable(g_mstar_dev_data->ts_platform_data->vdd);
    if (retval != 0) {
        TS_LOG_ERR("[%s]Fail to enable regulator when init,ret=%d!", __func__, retval);
        return retval;
    }
    TS_LOG_INFO("[%s]enable regulator 2800000,ret=%d!", __func__, retval);

    return 0;
}

int mstar_chip_reset(void)
{
    mstar_dev_hw_reset();
    return 0;
}

static int i2c_communicate_check(struct ts_kit_platform_data *mstar_pdata)
{
    int i = 0;
    int ret = NO_ERR;
    u8 cmd = 0;
    u8 buf[4] = { 0 };

    mstar_pdata->client->addr = g_mstar_dev_data->slave_addr;
    TS_LOG_INFO("%s,ret=0x%x", __func__, mstar_pdata->client->addr);
    if (mstar_pdata->client) {
        TS_LOG_INFO("%s: client is not null\n", __func__);
    } else {
        TS_LOG_INFO("%s: client is  null\n", __func__);
    }
    for (i = 0; i < MSTAR_DETECT_I2C_RETRY_TIMES; i++) {
        cmd = 0x37;
        ret = mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &cmd, 1);
        if (ret < 0) {
            TS_LOG_ERR("%s:mstar chip id read fail, ret=%d, i=%d\n", __func__, ret, i);
            msleep(50);
        } else {
            TS_LOG_INFO("%s:mstar chip id read pass\n", __func__);
            return NO_ERR;
        }
    }

    return ret;
}

static int mstar_chip_detect(struct ts_kit_platform_data *pdata)
{
    int ret = NO_ERR;

    // ontim debug node
    if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
    {
        return -EIO;
    }

    g_mstar_dev_data->ts_platform_data = pdata;
    if ((!pdata) && (!pdata->ts_dev)) {
        TS_LOG_ERR("%s device, ts_kit_platform_data *data or data->ts_dev is NULL \n", __func__);
        ret = -ENOMEM;
        goto exit;
    }

    g_mstar_dev_data->ts_platform_data->ts_dev->dev.of_node = g_mstar_dev_data->cnode;
    g_i2c_client = g_mstar_dev_data->ts_platform_data->client;
    mstar_parse_dts(g_mstar_dev_data->ts_platform_data->ts_dev->dev.of_node, g_mstar_dev_data);
    g_mstar_dev_data->is_i2c_one_byte = 0;
    g_mstar_dev_data->is_new_oem_structure = 0;
    g_mstar_dev_data->is_parade_solution = 0;

    mstar_mutex_init();
    tpd_gpio_output(GTP_RST_PORT, 0);
    tpd_gpio_output(GTP_INT_PORT, 0);
    mstar_power_init();
    msleep(50);

    ret = mstar_chip_reset();
    if (ret) {
        TS_LOG_ERR("%s:hardware reset fail, ret=%d\n", __func__, ret);
        goto exit_free_power;
    }

    ret = i2c_communicate_check(pdata);
    if (ret < 0) {
        TS_LOG_ERR("%s:not find mstar device, ret=%d\n", __func__, ret);
        goto exit_free_power;
    } else {
        TS_LOG_INFO("%s:find mstar device\n", __func__);

        strncpy(g_mstar_dev_data->chip_name, MSTAR_CHIP_NAME, MAX_STR_LEN-1);
    }

    tpd_load_status = 1;
    TS_LOG_INFO("%s:mstar chip detect success\n", __func__);

    // ontim debug node
    REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
    return 0;

exit_free_power:
    mstar_regulator_power_switch(false);
exit:

    TS_LOG_INFO("%s:mstar chip detect fail\n", __func__);
    return ret;
}

static int mstar_param_init(void)
{
    int ret = NO_ERR;
    char vendor_name[MSTAR_VENDOR_NAME_LEN] = {0};

    mstar_get_project_info();

    ret = mstar_get_vendor_name_from_dts(mstar_project_id, vendor_name, MSTAR_VENDOR_NAME_LEN);
    if (ret) {
        TS_LOG_ERR("%s:read vendor name fail by project id\n", __func__);
        if (MSTAR_VENDOR_ID_HLT == mstar_vendor_id) {
            strncpy(vendor_name, MSTAR_VENDOR_NAME_HLT, MSTAR_VENDOR_NAME_LEN);
        } else {
            TS_LOG_ERR("%s:read vendor name fail, ret=%d\n", __func__, ret);
            return ret;
        }
    }
    strncpy(g_mstar_dev_data->module_name, vendor_name, MAX_STR_LEN-1);
    return ret;
}

static int mstar_init_chip(void)
{
    int ret = 0;
    g_chip_num = (u8 *) kmalloc(16, GFP_KERNEL);
    ret = mstar_chip_detect_init();
    if (ret == -ENODEV) {
        mstar_unregister_device();
        goto out;
    }

    g_mstar_dev_data->is_in_cell = false;
    g_mstar_dev_data->easy_wakeup_info.sleep_mode = TS_POWER_OFF_MODE;
    g_mstar_dev_data->easy_wakeup_info.easy_wakeup_gesture = false;
    g_mstar_dev_data->easy_wakeup_info.easy_wakeup_flag = false;
    g_mstar_dev_data->easy_wakeup_info.palm_cover_flag = false;
    g_mstar_dev_data->easy_wakeup_info.palm_cover_control = false;
    g_mstar_dev_data->easy_wakeup_info.off_motion_on = false;
    g_mstar_dev_data->ts_platform_data->feature_info.holster_info.holster_switch = false;
    g_mstar_dev_data->ts_platform_data->feature_info.roi_info.roi_supported = 1;

    ret = mstar_param_init();
    if (ret) {
        TS_LOG_ERR("%s:init param fail, ret=%d\n", __func__, ret);
        goto out;
    }

    TS_LOG_INFO("%s:init chip success.\n", __func__);
    return NO_ERR;

out:
    TS_LOG_ERR("%s: init chip error.\n", __func__);
    return ret;
}

static void mstar_shutdown(void)
{

}

static int mstar_input_config(struct input_dev *input_dev)
{
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(BTN_TOOL_FINGER, input_dev->keybit);
    set_bit(TS_PALM_COVERED, input_dev->keybit);
    set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
    set_bit(TS_LETTER_c, input_dev->keybit);
    set_bit(TS_LETTER_e, input_dev->keybit);
    set_bit(TS_LETTER_m, input_dev->keybit);
    set_bit(TS_LETTER_w, input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif
    input_set_abs_params(input_dev, ABS_X, 0, g_mstar_dev_data->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, g_mstar_dev_data->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, g_mstar_dev_data->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, g_mstar_dev_data->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

#ifdef REPORT_2D_W
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_ABS_MT_TOUCH_MAJOR, 0, 0);
#endif
    g_input_dev = input_dev;
    return NO_ERR;
}

static int mstar_wrong_touch(void)
{
    int rc = NO_ERR;
    mutex_lock(&g_mutex);
    g_mstar_dev_data->easy_wakeup_info.off_motion_on = true;
    mutex_unlock(&g_mutex);
    TS_LOG_INFO("done\n");
    return rc;
}

static int mstar_irq_top_half(struct ts_cmd_node *cmd)
{
    cmd->command = TS_INT_PROCESS;
    g_int_esd_flag = 1;
    return NO_ERR;
}

static int mstar_chip_get_info(struct ts_chip_info_param *info)
{
    u16 nMajor, nMinor;

    mstar_get_customer_fw_ver(&nMajor, &nMinor, &g_fw_cust_ver);

    TS_LOG_INFO("mstar fw minor version = %d.%d.%d\n", nMajor, (nMinor & 0xFF), ((nMinor>>8) & 0xFF));

    snprintf(info->fw_vendor, CHIP_INFO_LENGTH * 2, "%d.%d", nMajor, (nMinor & 0xFF));
    strncpy(info->ic_vendor, MSTAR_CHIP_NAME, CHIP_INFO_LENGTH * 2);
    strncpy(info->mod_vendor, g_mstar_dev_data->module_name, CHIP_INFO_LENGTH);

    return NO_ERR;
}

int mstar_chip_get_capacitance_test_type(struct ts_test_type_info *info)
{
    int ret = 0;
    struct ts_kit_device_data *dev_data = NULL;
    dev_data = g_mstar_dev_data;
    if (!info) {
        TS_LOG_ERR("%s:info is Null\n", __func__);
        return -ENOMEM;
    }

    switch (info->op_action) {
    case TS_ACTION_READ:
        memcpy(info->tp_test_type, dev_data->tp_test_type, TS_CAP_TEST_TYPE_LEN);
        TS_LOG_INFO("%s:test_type=%s\n", __func__, info->tp_test_type);
        break;
    case TS_ACTION_WRITE:
        break;
    default:
        TS_LOG_ERR("%s:invalid op action:%d\n", __func__, info->op_action);
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mstar_reset_device(void)
{
    mstar_dev_hw_reset();
    return 0;
}

static int mstar_esdcheck_func(void)
{
    if (g_mstar_dev_data->need_wd_check_status)
        mstar_esd_check();
    return 0;
}

struct ts_device_ops ts_mstar_ops = {
    .chip_detect = mstar_chip_detect,
    .chip_init = mstar_init_chip,
    .chip_get_brightness_info = mstar_get_brightness_info,
    .chip_input_config = mstar_input_config,
    .chip_irq_top_half = mstar_irq_top_half,
    .chip_irq_bottom_half = mstar_irq_bottom_half,
    .chip_fw_update_boot = mstar_fw_update_boot,
    .chip_fw_update_sd = mstar_fw_update_sd,
    .chip_get_info = mstar_chip_get_info,
    .chip_get_capacitance_test_type = mstar_chip_get_capacitance_test_type,
    .chip_set_info_flag = mstar_set_info_flag,
    .chip_before_suspend = mstar_before_suspend,
    .chip_suspend = mstar_suspend,
    .chip_resume = mstar_resume,
    .chip_after_resume = mstar_after_resume,
    .chip_wakeup_gesture_enable_switch = mstar_wakeup_gesture_enable_switch,
    .chip_get_rawdata = mstar_get_raw_data,
//  .chip_get_debug_data = mstar_get_debug_data,
    .chip_glove_switch = mstar_glove_switch,
    .chip_shutdown = mstar_shutdown,
//  .chip_holster_switch = mstar_holster_switch,
//  .chip_roi_switch = mstar_roi_switch,
//  .chip_roi_rawdata = mstar_roi_rawdata,
//  .chip_palm_switch = mstar_palm_switch,
//  .chip_regs_operate = mstar_regs_operate,
//  .chip_calibrate = mstar_calibrate,
//  .chip_calibrate_wakeup_gesture = mstar_calibrate_wakeup_gesture,
    .chip_reset = mstar_reset_device,
    .chip_check_status = mstar_esdcheck_func,
// #if defined(HUAWEI_CHARGER_FB)
//  .chip_charger_switch = mstar_charger_switch,
// #endif
// #if defined(CONFIG_HUAWEI_DSM)
//  .chip_dsm_debug = mstar_rmi4_dsm_debug,
// #endif
// #ifdef HUAWEI_TOUCHSCREEN_TEST
//  .chip_test = test_dbg_cmd_test,
// #endif
    .chip_wrong_touch = mstar_wrong_touch,
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    .chip_proximity_enable = mstar_ps_enable_nodata,
#endif
};

static int __init mstar_ts_module_init(void)
{
    int ret = NO_ERR;
    bool found = false;
    struct device_node *child = NULL;
    struct device_node *root = NULL;

    TS_LOG_INFO("%s: called\n", __func__);

    g_mstar_dev_data = kzalloc(sizeof(struct ts_kit_device_data), GFP_KERNEL);
    if (NULL == g_mstar_dev_data) {
        TS_LOG_ERR("%s:alloc mem for device data fail\n", __func__);
        ret = -ENOMEM;
        goto error_exit;
    }

    root = of_find_compatible_node(NULL, NULL, HUAWEI_TS_KIT);
    if (!root) {
        TS_LOG_ERR("%s:find_compatible_node error\n", __func__);
        ret = -EINVAL;
        goto error_exit;
    }

    for_each_child_of_node(root, child) {
        if (of_device_is_compatible(child, MSTAR_CHIP_NAME)) {
            found = true;
            break;
        }
    }

    if (!found) {
        TS_LOG_ERR("%s:device tree node not found, name=%s\n", __func__, MSTAR_CHIP_NAME);
        ret = -EINVAL;
        goto error_exit;
    }

    g_mstar_dev_data->cnode = child;
    g_mstar_dev_data->ops = &ts_mstar_ops;
    ret = huawei_ts_chip_register(g_mstar_dev_data);
    if (ret) {
        TS_LOG_ERR("%s:chip register fail, ret=%d\n", __func__, ret);
        goto error_exit;
    }

    g_ts_kit_device_data = g_mstar_dev_data;
    TS_LOG_INFO("%s:success\n", __func__);
    return 0;

error_exit:
    kfree(g_mstar_dev_data);
    g_mstar_dev_data = NULL;
    TS_LOG_INFO("%s:fail\n", __func__);
    return ret;
}

static void __exit mstar_ts_module_exit(void)
{
    kfree(g_mstar_dev_data);
    g_mstar_dev_data = NULL;

    return;
}

late_initcall(mstar_ts_module_init);
module_exit(mstar_ts_module_exit);
MODULE_AUTHOR("Huawei Device Company");
MODULE_DESCRIPTION("Huawei TouchScreen Driver");
MODULE_LICENSE("GPL");
