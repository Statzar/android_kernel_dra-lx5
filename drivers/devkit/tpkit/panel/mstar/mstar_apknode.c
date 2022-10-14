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

#include "mstar_apknode.h"

/* important vars externed from main.c */
extern u16 mstar_change_fw_mode(u16 nMode);
extern void mstar_get_touch_packet_addr(u16 * pDataAddress, u16 * pFlagAddress);
extern void mstar_mutual_get_fw_info(MutualFirmwareInfo_t * pInfo);
extern void mstar_enter_sleep_mode(void);
extern s32 mstar_update_fw_sdcard(const char *pFilePath);
extern s32 mstar_update_fw(u8 ** szFwData, EmemType_e eEmemType);
extern u32 mstar_convert_char_to_hex_digit(char *pCh, u32 nLength);
extern void mstar_get_platform_fw_ver(u8 ** ppVersion);

extern MutualFirmwareInfo_t g_mutual_fw_info;

extern u8 g_finger_touch_disable;
extern u16 g_chip_type;
extern u16 g_chip_type_ori;
extern u32 SLAVE_I2C_ID_DBBUS;
extern u32 SLAVE_I2C_ID_DWI2C;
extern struct i2c_client *g_i2c_client;

extern struct kset *g_touch_kset;
extern struct kobject *g_touch_kobj;
extern struct mutex g_mutex;
extern struct mutex g_mutex_protect;

extern u16 FIRMWARE_MODE_UNKNOWN_MODE;
extern u16 FIRMWARE_MODE_DEMO_MODE;
extern u16 FIRMWARE_MODE_DEBUG_MODE;
extern u16 FIRMWARE_MODE_RAW_DATA_MODE;

extern u8 IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
extern u8 IS_DISABLE_ESD_PROTECTION_CHECK;
extern u8 IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED;
extern u8 IS_SELF_FREQ_SCAN_ENABLED;
extern u8 IS_FIRMWARE_DATA_LOG_ENABLED;

extern u8 g_log_packet[MUTUAL_DEBUG_MODE_PACKET_LENGTH];
extern u16 DEMO_MODE_PACKET_LENGTH;
extern u8 g_demo_packet[ILI21XX_DEMO_MODE_PACKET_LENGTH];
extern u16 g_fw_mode;

extern u8 *g_platform_fw_inter_ver; // internal use firmware version for MStar
extern u32 g_platform_fw_ver[3];
extern u8 g_fw_ver_flag;    // 0: old 1:new ,platform FW version V01.010.03
extern u8 *g_fw_cust_ver;   // customer firmware version

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA // for MSG28xx/MSG58xxA/ILI21xx
extern u16 g_fw_packet_data_addr;
extern u16 g_fw_packet_flag_addr;
#endif

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static u32 g_log_gesture_count;
static u8 g_log_gesture_infor_type;
extern u32 g_gesture_log_info[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH];
#endif

#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE  // support at most 64 types of gesture wakeup mode
extern u32 g_gesture_wakeup_mode[2];
#else // support at most 16 types of gesture wakeup mode
extern u32 g_gesture_wakeup_mode[2];
#endif

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern int mstar_proximity_enable(int mode);
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
u32 g_report_rate_enable = 0;
u32 g_int_cont = 0;
u32 g_vaild_touch_cont = 0;
u32 g_int_report_rate = 0;
u32 g_vaild_touch_report_rate = 0;
struct timeval g_start_time;
#endif

/* Externed from update.c */
extern void mstar_erase_emem(EmemType_e eEmemType);
extern void mstar_set_protect_bit(void);
extern void mstar_write_dq_mem_value(u16 nAddr, u32 nData);
extern u32 mstar_read_dq_mem_value(u16 nAddr);
extern s32 mstar_fw_update_sdcard(const char *pFilePath, u8 mode);
extern u8 *g_one_dimen_fw_data;

/* Global variables */
#ifdef CONFIG_ENABLE_JNI_INTERFACE
static MsgToolDrvCmd_t *g_msg_tool_cmd_in = NULL;
static u8 *g_snd_cmd_data = NULL;
static u8 *g_rtn_cmd_data = NULL;
#endif

static u8 g_read_trim_data[3] = { 0 };

static u32 g_debug_reg_count = 0;
static char g_debug_buf[1024] = { 0 };
static u16 g_debug_reg[MAX_DEBUG_REGISTER_NUM] = { 0 };
static u16 g_debug_reg_value[MAX_DEBUG_REGISTER_NUM] = { 0 };

static u8 g_debug_log_time_stamp = 0;
static u32 g_feature_support_status = 0;
static u8 g_debug_cmd_argu[MAX_DEBUG_COMMAND_ARGUMENT_NUM] = { 0 };

static u16 g_debug_cmd_arg_count = 0;
static u32 g_debug_read_data_size = 0;
static u32 g_update_complete = 0;

u8 g_switch_mode_apk = 0;

//u8 g_fw_data[MAX_UPDATE_FIRMWARE_BUFFER_SIZE][1024];
u8 **g_fw_data = NULL;
u32 g_fw_data_cont = 0;

u8 IS_GESTURE_WAKEUP_ENABLED = 0;
u8 IS_GESTURE_DEBUG_MODE_ENABLED = 0;
u8 IS_GESTURE_INFORMATION_MODE_ENABLED = 0;
u8 IS_GESTURE_WAKEUP_MODE_SUPPORT_64_TYPES_ENABLED = 0;

u8 g_hotknot_enable = 0;
u8 g_bypass_hotknot = 0;

struct apk_node_status ans;

void mstar_apknode_open_leather_sheath(void)
{
    s32 rc = 0;
    u8 szTxData[3] = { 0 };
    u32 i = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x02;
    szTxData[2] = 0x01;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            ans.g_IsEnableLeatherSheathMode = 1;

            TS_LOG_INFO("Open leather sheath mode success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Open leather sheath mode failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

void mstar_apknode_close_leath_sheath(void)
{
    s32 rc = 0;
    u8 szTxData[3] = { 0 };
    u32 i = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x02;
    szTxData[2] = 0x00;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            ans.g_IsEnableLeatherSheathMode = 0;

            TS_LOG_INFO("Close leather sheath mode success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Close leather sheath mode failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

void mstar_apknode_open_glove(void)
{
    s32 rc = 0;
    u8 szTxData[3] = { 0 };
    u32 i = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x01;
    szTxData[2] = 0x01;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            ans.g_IsEnableGloveMode = 1;

            TS_LOG_INFO("Open glove mode success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Open glove mode failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

void mstar_apknode_close_glove(void)
{
    s32 rc = 0;
    u8 szTxData[3] = { 0 };
    u32 i = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x01;
    szTxData[2] = 0x00;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            ans.g_IsEnableGloveMode = 0;

            TS_LOG_INFO("Close glove mode success\n");
            break;
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Close glove mode failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

void mstar_apknode_get_glove_info(u8 * pGloveMode)
{
    u8 szTxData[3] = { 0 };
    u8 szRxData[2] = { 0 };
    u32 i = 0;
    s32 rc = 0;

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x01;
    szTxData[2] = 0x02;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            TS_LOG_INFO("Get glove info mstar_iic_write_data() success\n");
        }

        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 1);
        if (rc > 0) {
            TS_LOG_INFO("Get glove info mstar_iic_read_data() success\n");

            if (szRxData[0] == 0x00 || szRxData[0] == 0x01) {
                break;
            } else {
                i = 0;
            }
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Get glove info failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    *pGloveMode = szRxData[0];

    TS_LOG_INFO("*pGloveMode = 0x%x\n", *pGloveMode);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

static void mstar_set_film_mode(u8 nFilmtype)
{
    u8 szTxData[2] = { 0x13, nFilmtype };
    s32 rc;
    TS_LOG_INFO("*** %s() ***\n", __func__);

    mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
    rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 2);
    if (rc > 0) {
        TS_LOG_INFO("Set Film Mode success,\n");
    }
}

static int mstar_get_film_mode(void)
{
    u8 szTxData[1] = { 0x12 };
    u8 szRxData[3] = { 0 };
    s32 rc = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
    rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 1);
    if (rc > 0) {
        TS_LOG_INFO("Get firmware info mstar_iic_write_data() success\n");
    }
    mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
    rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 3);
    if (rc > 0) {
        if (szRxData[1] == FEATURE_FILM_MODE_LOW_STRENGTH) {
            TS_LOG_INFO("Film mode: Low strength\n");
            return FEATURE_FILM_MODE_LOW_STRENGTH;
        } else if (szRxData[1] == FEATURE_FILM_MODE_HIGH_STRENGTH) {
            TS_LOG_INFO("Film mode: High strength\n");
            return FEATURE_FILM_MODE_HIGH_STRENGTH;
        } else if (szRxData[1] == FEATURE_FILM_MODE_DEFAULT) {
            TS_LOG_INFO("Film mode: Default\n");
            return FEATURE_FILM_MODE_DEFAULT;
        }
    }
    return -1;
}

static ssize_t mstar_jni_msg_tool_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    long nRet = 0;
    u8 nBusType = 0;
    u16 nReadLen = 0;
    u8 szCmdData[20] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);
    nBusType = nCount & 0xFF;
    nReadLen = (nCount >> 8) & 0xFFFF;
    if (nBusType == SLAVE_I2C_ID_DBBUS || nBusType == SLAVE_I2C_ID_DWI2C) {
        mstar_iic_read_data(nBusType, &szCmdData[0], nReadLen);
    }

    nRet = copy_to_user(pBuffer, &szCmdData[0], nReadLen);
    return nRet;
}

static void mstar_get_leather_sheath_info(u8 * pLeatherSheathMode)  // used for MSG28xx only
{
    u8 szTxData[3] = { 0 };
    u8 szRxData[2] = { 0 };
    u32 i = 0;
    s32 rc = 0;

    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for device driver can send i2c command to firmware.

    szTxData[0] = 0x06;
    szTxData[1] = 0x02;
    szTxData[2] = 0x02;

    mutex_lock(&g_mutex);

    while (i < 5) {
        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &szTxData[0], 3);
        if (rc > 0) {
            TS_LOG_INFO("Get leather sheath info mstar_iic_write_data() success\n");
        }

        mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
        rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szRxData[0], 1);
        if (rc > 0) {
            TS_LOG_INFO("Get leather sheath info mstar_iic_read_data() success\n");

            if (szRxData[0] == 0x00 || szRxData[0] == 0x01) {
                break;
            } else {
                i = 0;
            }
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("Get leather sheath info failed, rc = %d\n", rc);
    }

    mutex_unlock(&g_mutex);

    *pLeatherSheathMode = szRxData[0];

    TS_LOG_INFO("*pLeatherSheathMode = 0x%x\n", *pLeatherSheathMode);

    g_finger_touch_disable = 0; // Resume finger touch ISR handling after device driver have sent i2c command to firmware.
}

static void mstar_write_trim_code(u8 * uArrayData, u32 nAddr)
{
    u8 *nBuf;
    nBuf = uArrayData;
    mstar_finger_touch_report_disable();
    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    nBuf[0] = ((nBuf[0] & 0x01) << 7) + ((nBuf[0] & 0x7F) >> 1);
    //DEBUG("*** set Write *** 0x%x 0x%x\n", nBuf[0],nBuf[1]);
    mstar_set_protect_bit();

    // set write done
    //mstar_set_reg_low_byte 0x1606 0x01
    // Set Password
    mstar_set_reg_low_byte(0x1616, 0xAA);
    mstar_set_reg_low_byte(0x1617, 0x55);
    mstar_set_reg_low_byte(0x1618, 0xA5);
    mstar_set_reg_low_byte(0x1619, 0x5A);

    // disable cpu read, initial read
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);

    // set info block
    mstar_set_reg_low_byte(0x1606, 0x40);
    mstar_set_reg_low_byte(0x1607, 0x00);

    // set info double buffer
    mstar_set_reg_low_byte(0x1610, 0x00);

    // data align
    mstar_set_reg_low_byte(0x1640, 0x01);

    //set info block
    mstar_set_reg_low_byte(0x1607, 0x08);
    //set info double buffer
    mstar_set_reg_low_byte(0x1604, 0x01);
    // eflash mode trigger
    mstar_set_reg_low_byte(0x1606, 0x41);
    // set initial data
    mstar_set_reg_low_byte(0x1602, 0xA5);
    mstar_set_reg_low_byte(0x1602, 0x5A);
    mstar_set_reg_low_byte(0x1602, nBuf[1]);
    mstar_set_reg_low_byte(0x1602, nBuf[0]);
    // set initial address (for latch SA, CA)
    mstar_set_reg_low_byte(0x1600, 0x00);
    mstar_set_reg_low_byte(0x1601, 0x00);

    // set initial address (for latch PA)
    mstar_set_reg_low_byte(0x1600, 0x00);
    mstar_set_reg_low_byte(0x1601, 0x00);
    // set write done
    mstar_set_reg_low_byte(0x1606, 0x84);
    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();
    mdelay(100);
    mstar_finger_touch_report_enable();
    mstar_dev_hw_reset();
    //DisableBypassHotknot();
    kfree(nBuf);
}

static int mstar_read_trim_code(u16 nAddr, u16 nLength)
{
    u8 tx_data[4] = { 0 };
    u8 rx_data[20] = { 0 };
    u8 *pBuf;
    u8 result;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    pBuf = (u8 *) kmalloc(nLength, GFP_KERNEL);
    mstar_finger_touch_report_disable();
    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    TS_LOG_INFO("*** set read ***0x%x 0x%x, 0x%x\n", nAddr, nAddr >> 8, nAddr & (0x00FF));
    tx_data[0] = 0x10;
    tx_data[1] = nAddr >> 8;
    tx_data[2] = (nAddr & (0x00FF)) * 2;
    result = mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
    mdelay(50);
    result = mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &rx_data[0], 2);
    TS_LOG_INFO("0x%x, (rx_data[0]&0x3F) << 1 = 0x%x, (rx_data[0] >> 7 = 0x%x\n", rx_data[1],
            (rx_data[1] & 0x7F) << 1, (rx_data[1] >> 7));
    pBuf[0] = rx_data[1];
    pBuf[1] = ((rx_data[1] & 0x7F) << 1) + (rx_data[1] >> 7);
    pBuf[2] = rx_data[0];
    TS_LOG_INFO("0x%x,0x%x,0x%x\n", pBuf[0], pBuf[1], pBuf[2]);
    g_read_trim_data[0] = pBuf[0];
    g_read_trim_data[1] = pBuf[1];
    g_read_trim_data[2] = pBuf[2];
    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();
    mdelay(100);
    mstar_finger_touch_report_enable();
    //TouchDeviceResetHw();
    kfree(pBuf);
    return result;
}

#ifdef CONFIG_ENABLE_JNI_INTERFACE
/*
static void _DebugJniShowArray(u8 *pBuf, u16 nLen)
{
    int i;

    for (i = 0; i < nLen; i ++)
    {
        TS_LOG_INFO( "%02X ", pBuf[i]);

        if (i%16==15)
        {
            TS_LOG_INFO( "\n");
        }
    }
    TS_LOG_INFO( "\n");
}
*/
u64 mstar_ptr_to_u64(u8 * pValue)
{
    uintptr_t nValue = (uintptr_t) pValue;
    return (u64) (0xFFFFFFFFFFFFFFFF & nValue);
}

u8 *U64ToPtr(u64 nValue)
{
    uintptr_t pValue = (uintptr_t) nValue;
    return (u8 *) pValue;
}

static void mstar_jni_reg_get_xbyte(MsgToolDrvCmd_t * pCmd)
{
    u16 nAddr = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    nAddr = (g_snd_cmd_data[1] << 8) | g_snd_cmd_data[0];
    mstar_get_reg_xbit(nAddr, g_rtn_cmd_data, pCmd->nRtnCmdLen, MAX_I2C_TRANSACTION_LENGTH_LIMIT);
    //_DebugJniShowArray(g_rtn_cmd_data, pCmd->nRtnCmdLen);
}

static void mstar_jni_clear_msgtool_mem(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    memset(g_msg_tool_cmd_in, 0, sizeof(MsgToolDrvCmd_t));
    memset(g_snd_cmd_data, 0, 1024);
    memset(g_rtn_cmd_data, 0, 1024);
}

static MsgToolDrvCmd_t *mstar_jni_trans_cmd_from_user(unsigned long nArg)
{
    long nRet;
    MsgToolDrvCmd_t tCmdIn;
    MsgToolDrvCmd_t *pTransCmd;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_jni_clear_msgtool_mem();

    pTransCmd = (MsgToolDrvCmd_t *) g_msg_tool_cmd_in;
    nRet = copy_from_user(&tCmdIn, (void *)nArg, sizeof(MsgToolDrvCmd_t));
    pTransCmd->nCmdId = tCmdIn.nCmdId;

    //_DebugJniShowArray(&tCmdIn, sizeof( MsgToolDrvCmd_t));
    if (tCmdIn.nSndCmdLen > 0) {
        pTransCmd->nSndCmdLen = tCmdIn.nSndCmdLen;
        nRet = copy_from_user(g_snd_cmd_data, U64ToPtr(tCmdIn.nSndCmdDataPtr), pTransCmd->nSndCmdLen);
    }

    if (tCmdIn.nRtnCmdLen > 0) {
        pTransCmd->nRtnCmdLen = tCmdIn.nRtnCmdLen;
        nRet = copy_from_user(g_rtn_cmd_data, U64ToPtr(tCmdIn.nRtnCmdDataPtr), pTransCmd->nRtnCmdLen);
    }

    return pTransCmd;
}

static void mstar_jni_trans_cmd_to_user(MsgToolDrvCmd_t * pTransCmd, unsigned long nArg)
{
    MsgToolDrvCmd_t tCmdOut;
    long nRet;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    nRet = copy_from_user(&tCmdOut, (void *)nArg, sizeof(MsgToolDrvCmd_t));

    //_DebugJniShowArray(&tCmdOut, sizeof( MsgToolDrvCmd_t));
    nRet = copy_to_user(U64ToPtr(tCmdOut.nRtnCmdDataPtr), g_rtn_cmd_data, tCmdOut.nRtnCmdLen);
}

void mstar_apknode_create_jni_msg(void)
{
    TS_LOG_INFO("_DrvJniCreateMsgToolMem\n");
    g_msg_tool_cmd_in = (MsgToolDrvCmd_t *) kmalloc(sizeof(MsgToolDrvCmd_t), GFP_KERNEL);
    g_snd_cmd_data = (u8 *) kmalloc(1024, GFP_KERNEL);
    g_rtn_cmd_data = (u8 *) kmalloc(1024, GFP_KERNEL);
}

void mstar_apknode_delete_jni_msg(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_msg_tool_cmd_in) {
        kfree(g_msg_tool_cmd_in);
        g_msg_tool_cmd_in = NULL;
    }

    if (g_snd_cmd_data) {
        kfree(g_snd_cmd_data);
        g_snd_cmd_data = NULL;
    }

    if (g_rtn_cmd_data) {
        kfree(g_rtn_cmd_data);
        g_rtn_cmd_data = NULL;
    }
}
#endif //CONFIG_ENABLE_JNI_INTERFACE

static struct proc_dir_entry *g_proc_class_entry = NULL;
static struct proc_dir_entry *g_proc_ts_msg20xx_entry = NULL;
static struct proc_dir_entry *g_proc_device_entry = NULL;
static struct proc_dir_entry *g_proc_chip_type_entry = NULL;
static struct proc_dir_entry *g_proc_fw_data_entry = NULL;
static struct proc_dir_entry *g_proc_fw_update_entry = NULL;
static struct proc_dir_entry *g_proc_cust_fw_ver_entry = NULL;
static struct proc_dir_entry *g_proc_plateform_fw_ver_entry = NULL;
static struct proc_dir_entry *g_proc_drv_ver_entry = NULL;
static struct proc_dir_entry *g_proc_sd_fw_update_entry = NULL;
static struct proc_dir_entry *g_proc_fw_debug_entry = NULL;
static struct proc_dir_entry *g_proc_fw_set_debug_value_entry = NULL;
static struct proc_dir_entry *g_proc_fw_smbus_debug_entry = NULL;
static struct proc_dir_entry *g_proc_fw_set_dqmem_value_entry = NULL;
static struct proc_dir_entry *g_proc_fw_mode_entry = NULL;
static struct proc_dir_entry *g_proc_fw_sensor_entry = NULL;
static struct proc_dir_entry *g_proc_fw_packet_header_entry = NULL;
static struct proc_dir_entry *g_proc_query_feature_support_status_entry = NULL;
static struct proc_dir_entry *g_proc_change_feature_support_status_entry = NULL;
static struct proc_dir_entry *g_proc_gesture_wakeup_entry = NULL;
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
static struct proc_dir_entry *g_proc_gesture_debug_entry = NULL;
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static struct proc_dir_entry *g_proc_gesture_info_entry = NULL;
#endif //CONFIG_ENABLE_GESTURE_INFORMATION_MODE
#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
static struct proc_dir_entry *g_proc_report_rate_entry = NULL;
#endif //CONFIG_ENABLE_COUNT_REPORT_RATE
static struct proc_dir_entry *g_proc_glove_entry = NULL;
static struct proc_dir_entry *g_proc_open_glove_entry = NULL;
static struct proc_dir_entry *g_proc_close_glove_entry = NULL;
static struct proc_dir_entry *g_proc_leather_sheath_entry = NULL;
static struct proc_dir_entry *g_proc_film_entry = NULL;
#ifdef CONFIG_ENABLE_JNI_INTERFACE
static struct proc_dir_entry *g_proc_jni_method_entry = NULL;
#endif //CONFIG_ENABLE_JNI_INTERFACE
static struct proc_dir_entry *g_proc_selinux_fw_update_entry = NULL;
static struct proc_dir_entry *g_proc_force_fw_update_entry = NULL;
#ifdef MP_TEST_FUNCTION_2
static struct proc_dir_entry *g_proc_mp_test_customised_entry = NULL;
#endif
static struct proc_dir_entry *g_proc_trim_code_entry = NULL;
static struct proc_dir_entry *g_proc_proximity_entry = NULL;

static ssize_t mstar_apknode_proximity_write(struct file *filp, const char *buff, size_t size, loff_t * pPos)
{
    int res = 0;
    char cmd[10] = { 0 };

    if (buff != NULL) {
        res = copy_from_user(cmd, buff, size - 1);
        if (res < 0) {
            TS_LOG_ERR("copy data from user space, failed\n");
            return -1;
        }
    }

    TS_LOG_INFO("size = %d, cmd = %s\n", (int)size, cmd);

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    if (strcmp(cmd, "1") == 0) {
        TS_LOG_INFO("enable Proximity mode\n");
        mstar_proximity_enable(1);
    } else if (strcmp(cmd, "0") == 0) {
        TS_LOG_INFO("disable Proximity mode\n");
        mstar_proximity_enable(0);
    } else
        TS_LOG_ERR("Unknown command\n");
#endif

    return size;
}

#ifdef MP_TEST_FUNCTION_2
static ssize_t mstar_apknode_mp_test_customised_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    return nCount;
}

static ssize_t mstar_apknode_mp_test_customised_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                              loff_t * pPos)
{
    u16 nMajor = 0, nMinor = 0;
    u32 nLength = 0;
    u8 result[16];
    int res = 0;

    if (*pPos != 0)
        return 0;

    mstar_get_customer_fw_ver(&nMajor, &nMinor, &g_fw_cust_ver);
    TS_LOG_INFO("*** %s() g_fw_cust_ver = %s ***\n", __func__, g_fw_cust_ver);

    mstar_get_platform_fw_ver(&g_platform_fw_inter_ver);
    TS_LOG_INFO("*** %s() g_platform_fw_inter_ver = %s ***\n", __func__, g_platform_fw_inter_ver);

    mstar_mp_get_fw_ver_on_flash();

    TS_LOG_INFO("*** %s() Driver version = %s ***\n", __func__, DEVICE_DRIVER_RELEASE_VERSION);

    res = mstar_mp_test_entry(g_chip_type, NULL);
    TS_LOG_INFO("MP Test Result = %d \n", res);

    if (res == 1 || res == 2) {
        //mstar_mp_test_save_data(g_chip_type, res);
    } else {
        TS_LOG_ERR("MP Test got unknown failure...won't save data as CSV\n");
    }

    nLength = sprintf(result, "%d", res);

    res = copy_to_user((int *)pBuffer, &res, nLength);
    if (res < 0) {
        printk("Failed to copy data to user space\n");
    }
    *pPos += nLength;
    mstar_mp_test_entry_end(g_chip_type);
    TS_LOG_INFO("end\n");
    return nLength;
}
#endif

static ssize_t mstar_apknode_mp_flow_switch_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[128] = { 0 };

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if(mp_test_info.mode == 1) {
        mp_test_info.mode = 2;
    } else if (mp_test_info.mode == 2) {
        mp_test_info.mode = 1;
    }

    nLength = sprintf(nUserTempBuffer, "mp_test_info.mode = %d\n", mp_test_info.mode);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength))
        return -EFAULT;

    TS_LOG_INFO("mp_test_info.mode = %d\n", mp_test_info.mode);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_chip_type_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    nLength = sprintf(nUserTempBuffer, "%d", g_chip_type);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength))
        return -EFAULT;

    TS_LOG_INFO("g_chip_type = 0x%x, g_chip_type_ori = 0x%x\n", g_chip_type, g_chip_type_ori);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_chip_type_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

//    g_chip_type = mstar_get_chip_type();

    return nCount;
}

static ssize_t mstar_apknode_fw_data_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    TS_LOG_INFO("*** %s() g_fw_data_cont = %d ***\n", __func__, g_fw_data_cont);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    *pPos += g_fw_data_cont;

    return g_fw_data_cont;
}

static ssize_t mstar_apknode_fw_data_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                        loff_t * pPos)
{
    u32 nNum = nCount / 1024;
    u32 nRemainder = nCount % 1024;
    u32 i;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (ERR_ALLOC_MEM(g_fw_data)) {
        TS_LOG_ERR("FW Count is zero\n");

        //dicky
        g_fw_data = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
        if (ERR_ALLOC_MEM(g_fw_data)) {
            TS_LOG_ERR("Failed to allocate FW buffer\n");
            goto out;
        }
    }

    if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
        || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
        if (nNum > 0)   // nCount >= 1024
        {
            for (i = 0; i < nNum; i++) {
                memset(g_debug_buf, 0, 1024);
                if (copy_from_user(g_debug_buf, pBuffer + (i * 1024), 1024)) {
                    TS_LOG_INFO("copy_from_user() failed\n");

                    return -EFAULT;
                }

                g_fw_data[g_fw_data_cont] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);
                if (ERR_ALLOC_MEM(g_fw_data[g_fw_data_cont])) {
                    TS_LOG_ERR("Failed to allocate FW buffer\n");
                    goto out;
                }

                memcpy(g_fw_data[g_fw_data_cont], g_debug_buf, 1024);

                g_fw_data_cont++;
            }

            if (nRemainder > 0) // Handle special firmware size like MSG22XX(48.5KB)
            {
                TS_LOG_INFO("nRemainder = %d\n", nRemainder);
                memset(g_debug_buf, 0, 1024);
                if (copy_from_user(g_debug_buf, pBuffer + (i * 1024), nRemainder)) {
                    TS_LOG_INFO("copy_from_user() failed\n");

                    return -EFAULT;
                }

                g_fw_data[g_fw_data_cont] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);
                if (ERR_ALLOC_MEM(g_fw_data[g_fw_data_cont])) {
                    TS_LOG_ERR("Failed to allocate FW buffer\n");
                    goto out;
                }

                memcpy(g_fw_data[g_fw_data_cont], g_debug_buf, nRemainder);

                g_fw_data_cont++;
            }
        } else      // nCount < 1024
        {
            if (nCount > 0) {
                memset(g_debug_buf, 0, 1024);
                if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
                    TS_LOG_INFO("copy_from_user() failed\n");

                    return -EFAULT;
                }

                g_fw_data[g_fw_data_cont] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);
                if (ERR_ALLOC_MEM(g_fw_data[g_fw_data_cont])) {
                    TS_LOG_ERR("Failed to allocate FW buffer\n");
                    goto out;
                }

                memcpy(g_fw_data[g_fw_data_cont], g_debug_buf, nCount);

                g_fw_data_cont++;
            }
        }
    } else {
        TS_LOG_INFO("Unsupported chip type = 0x%x\n", g_chip_type);
    }

    TS_LOG_INFO("*** g_fw_data_cont = %d ***\n", g_fw_data_cont);

    if (g_debug_buf != NULL) {
        TS_LOG_INFO("*** buf[0] = %c ***\n", g_debug_buf[0]);
    }

    return nCount;

out:
    if (!ERR_ALLOC_MEM(g_fw_data)) {
        for (i = 0; i < 130; i++) {
            if (!ERR_ALLOC_MEM(g_fw_data[i])) {
                kfree(g_fw_data[i]);
                g_fw_data[i] = NULL;
            }
        }

        kfree(g_fw_data);
        g_fw_data = NULL;
    }
    return nCount;
}

static ssize_t mstar_apknode_fw_update_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    nLength = sprintf(nUserTempBuffer, "%d", g_update_complete);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength))
        return -EFAULT;

    TS_LOG_INFO("*** g_update_complete = %d ***\n", g_update_complete);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_update_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    int i = 0;
    s32 nRetVal = 0;

    TS_LOG_INFO("*** %s() g_fw_data_cont = %d ***\n", __func__, g_fw_data_cont);

    if (ERR_ALLOC_MEM(g_fw_data) || g_fw_data_cont == 0) {
        TS_LOG_ERR("FW Buffer is NULL\n");
        goto out;
    }

    g_one_dimen_fw_data = vmalloc(MSG28XX_FIRMWARE_WHOLE_SIZE * 1024 * sizeof(u8));
    if (ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        TS_LOG_ERR("Failed to allocate FW buffer\n");
        goto out;
    }

    mstar_finger_touch_report_disable();

    if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
        || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
        nRetVal = mstar_update_fw(g_fw_data, EMEM_ALL);
    } else {
        TS_LOG_INFO("This chip type (0x%x) does not support update firmware by MTPTool APK\n", g_chip_type);
        nRetVal = -1;
    }

    if (0 != nRetVal) {
        g_update_complete = 0;
        TS_LOG_INFO("Update FAILED\n");
    } else {
        g_update_complete = 1;
        TS_LOG_INFO("Update SUCCESS\n");
    }

    mstar_finger_touch_report_enable();

out:
    if (!ERR_ALLOC_MEM(g_fw_data)) {
        for (i = 0; i < 130; i++) {
            if (!ERR_ALLOC_MEM(g_fw_data[i])) {
                kfree(g_fw_data[i]);
                g_fw_data[i] = NULL;
            }
        }

        kfree(g_fw_data);
        g_fw_data = NULL;
    }
    if (!ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        vfree(g_one_dimen_fw_data);
        g_one_dimen_fw_data = NULL;
    }
    return nCount;
}

static ssize_t mstar_apknode_customer_fw_ver_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                           loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    u16 nMajor = 0, nMinor = 0;
    TS_LOG_INFO("*** %s() g_fw_cust_ver = %s ***\n", __func__, g_fw_cust_ver);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
    mstar_get_customer_fw_ver(&nMajor, &nMinor, &g_fw_cust_ver);
    nLength = snprintf(nUserTempBuffer, sizeof(nUserTempBuffer), "%s", g_fw_cust_ver);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }
    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_customer_fw_ver_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u16 nMajor = 0, nMinor = 0;

    mstar_get_customer_fw_ver(&nMajor, &nMinor, &g_fw_cust_ver);

    TS_LOG_INFO("*** %s() g_fw_cust_ver = %s ***\n", __func__, g_fw_cust_ver);

    return nCount;
}

static ssize_t mstar_apknode_platform_fw_ver_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                           loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() g_platform_fw_inter_ver = %s ***\n", __func__, g_platform_fw_inter_ver);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
    nLength = snprintf(nUserTempBuffer, sizeof(nUserTempBuffer), "%s", g_platform_fw_inter_ver);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_platform_fw_ver_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    mstar_get_platform_fw_ver(&g_platform_fw_inter_ver);

    TS_LOG_INFO("*** %s() g_platform_fw_inter_ver = %s ***\n", __func__, g_platform_fw_inter_ver);

    return nCount;
}

static ssize_t mstar_apknode_drive_ver_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
    nLength = sprintf(nUserTempBuffer, "%s", DEVICE_DRIVER_RELEASE_VERSION);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_drive_ver_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    return nCount;
}

static ssize_t mstar_apknode_fw_sdcard_update_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u16 nMajor = 0, nMinor = 0;
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    mstar_get_customer_fw_ver(&nMajor, &nMinor, &g_fw_cust_ver);

    TS_LOG_INFO("*** %s() g_fw_cust_ver = %s ***\n", __func__, g_fw_cust_ver);

    nLength = snprintf(nUserTempBuffer, sizeof(nUserTempBuffer), "%s\n", g_fw_cust_ver);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_sdcard_update_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                             loff_t * pPos)
{
    char *pValid = NULL;
    char *pTmpFilePath = NULL;
    char szFilePath[100] = { 0 };
    char *pStr = NULL;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    TS_LOG_INFO("pBuffer = %s\n", g_debug_buf);
    pStr = g_debug_buf;
    if (pStr != NULL) {
        pValid = strstr(pStr, ".bin");

        if (pValid) {
            pTmpFilePath = strsep((char **)&pStr, ".");
            TS_LOG_INFO("pTmpFilePath = %s\n", pTmpFilePath);
            if (pTmpFilePath != NULL) {
                strcat(szFilePath, pTmpFilePath);
                strcat(szFilePath, ".bin");
            }
            TS_LOG_INFO("szFilePath = %s\n", szFilePath);

            if (0 != mstar_update_fw_sdcard(szFilePath)) {
                TS_LOG_INFO("Update FAILED\n");
            } else {
                TS_LOG_INFO("Update SUCCESS\n");
            }
        } else {
            TS_LOG_INFO("The file type of the update firmware bin file is not a .bin file.\n");
        }
    } else {
        TS_LOG_INFO("The file path of the update firmware bin file is NULL.\n");
    }

    return nCount;
}

static ssize_t mstar_apknode_selinux_fw_update_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                             loff_t * pPos)
{
    u32 nLength = 0;
    s32 nRetVal = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
        || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A) {
        TS_LOG_INFO("FIRMWARE_FILE_PATH_ON_SD_CARD = %s\n", FIRMWARE_FILE_PATH_ON_SD_CARD);
        nRetVal = mstar_fw_update_sdcard(FIRMWARE_FILE_PATH_ON_SD_CARD, 1);
    } else {
        TS_LOG_INFO("This chip type (0x%x) does not support selinux limit firmware update\n", g_chip_type);
        nRetVal = -1;
    }

    if (0 != nRetVal) {
        g_update_complete = 0;
        TS_LOG_INFO("Update FAILED\n");
    } else {
        g_update_complete = 1;
        TS_LOG_INFO("Update SUCCESS\n");
    }

    nLength = sprintf(nUserTempBuffer, "%d", g_update_complete);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    TS_LOG_INFO("*** g_update_complete = %d ***\n", g_update_complete);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_force_fw_update_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                           loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    TS_LOG_INFO("*** IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = %d ***\n", IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED);

    IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = 1;    // Enable force firmware update
    g_feature_support_status = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;

    nLength = sprintf(nUserTempBuffer, "%d", g_feature_support_status);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    TS_LOG_INFO("*** g_feature_support_status = %d ***\n", g_feature_support_status);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_debug_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 i, nLength = 0;
    u8 nBank, nAddr;
    u16 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
    u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    for (i = 0; i < g_debug_reg_count; i++) {
        szRegData[i] = mstar_get_reg_16bit(g_debug_reg[i]);
    }

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    for (i = 0; i < g_debug_reg_count; i++) {
        nBank = (g_debug_reg[i] >> 8) & 0xFF;
        nAddr = g_debug_reg[i] & 0xFF;

        TS_LOG_INFO("reg(0x%02X,0x%02X)=0x%04X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%02X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%02X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }

    nLength = strlen(szOut);
    if (copy_to_user(pBuffer, szOut, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_debug_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                         loff_t * pPos)
{
    u32 i;
    char *pCh = NULL;
    char *pStr = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("*** pBuffer[0] = %c ***\n", g_debug_buf[0]);
        TS_LOG_INFO("*** pBuffer[1] = %c ***\n", g_debug_buf[1]);
        TS_LOG_INFO("*** pBuffer[2] = %c ***\n", g_debug_buf[2]);
        TS_LOG_INFO("*** pBuffer[3] = %c ***\n", g_debug_buf[3]);
        TS_LOG_INFO("*** pBuffer[4] = %c ***\n", g_debug_buf[4]);
        TS_LOG_INFO("*** pBuffer[5] = %c ***\n", g_debug_buf[5]);

        TS_LOG_INFO("nCount = %d\n", (int)nCount);

        g_debug_buf[nCount] = '\0';
        pStr = g_debug_buf;

        i = 0;

        while ((pCh = strsep((char **)&pStr, " ,")) && (i < MAX_DEBUG_REGISTER_NUM)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            g_debug_reg[i] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));

            TS_LOG_INFO("g_debug_reg[%d] = 0x%04X\n", i, g_debug_reg[i]);
            i++;
        }
        g_debug_reg_count = i;

        TS_LOG_INFO("g_debug_reg_count = %d\n", g_debug_reg_count);
    }

    return nCount;
}

static ssize_t mstar_apknode_fw_set_debug_value_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                              loff_t * pPos)
{
    u32 i, nLength = 0;
    u8 nBank, nAddr;
    u16 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
    u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = {
    0};

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    for (i = 0; i < g_debug_reg_count; i++) {
        szRegData[i] = mstar_get_reg_16bit(g_debug_reg[i]);
    }

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    for (i = 0; i < g_debug_reg_count; i++) {
        nBank = (g_debug_reg[i] >> 8) & 0xFF;
        nAddr = g_debug_reg[i] & 0xFF;

        TS_LOG_INFO("reg(0x%02X,0x%02X)=0x%04X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%02X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%02X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }

    nLength = strlen(szOut);
    if (copy_to_user(pBuffer, szOut, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_set_debug_value_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    u32 i, j, k;
    char *pCh = NULL;
    char *pStr = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("*** pBuffer[0] = %c ***\n", g_debug_buf[0]);
        TS_LOG_INFO("*** pBuffer[1] = %c ***\n", g_debug_buf[1]);

        TS_LOG_INFO("nCount = %d\n", (int)nCount);
        g_debug_buf[nCount] = '\0';
        pStr = g_debug_buf;

        i = 0;
        j = 0;
        k = 0;

        while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            if ((i % 2) == 0) {
                g_debug_reg[j] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("g_debug_reg[%d] = 0x%04X\n", j, g_debug_reg[j]);
                j++;
            } else  // (i%2) == 1
            {
                g_debug_reg_value[k] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("g_debug_reg_value[%d] = 0x%04X\n", k, g_debug_reg_value[k]);
                k++;
            }

            i++;
        }
        g_debug_reg_count = j;

        TS_LOG_INFO("g_debug_reg_count = %d\n", g_debug_reg_count);

        mstar_dbbus_enter_serial_debug();
        mstar_dbbus_stop_mcu();
        mstar_dbbus_iic_use_bus();
        mstar_dbbus_iic_reshape();

        for (i = 0; i < g_debug_reg_count; i++) {
            mstar_set_reg_16bit(g_debug_reg[i], g_debug_reg_value[i]);
            TS_LOG_INFO("g_debug_reg[%d] = 0x%04X, g_debug_reg_value[%d] = 0x%04X\n", i, g_debug_reg[i], i, g_debug_reg_value[i]);  // add for debug
        }

        mstar_dbbus_iic_not_use_bus();
        mstar_dbbus_not_stop_mcu();
        mstar_dbbus_exit_serial_debug();
    }

    return nCount;
}

static ssize_t mstar_apknode_fw_smbus_debug_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    u32 i = 0, nLength = 0;
    u8 szSmBusRxData[MAX_I2C_TRANSACTION_LENGTH_LIMIT] = { 0 };
    u8 szOut[MAX_I2C_TRANSACTION_LENGTH_LIMIT * 2] = { 0 };
    u8 szValue[10] = { 0 };
    s32 rc = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaReset();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    mutex_lock(&g_mutex);
    TS_LOG_INFO("*** %s() *** mutex_lock(&g_mutex)\n", __func__);   // add for debug

    while (i < 5) {
        if (g_debug_cmd_arg_count > 0)  // Send write command
        {
            TS_LOG_INFO("Execute I2C SMBUS write command\n");

            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &g_debug_cmd_argu[0], g_debug_cmd_arg_count);
            if (rc > 0) {
                TS_LOG_INFO("mstar_iic_write_data(0x%X, 0x%X, %d) success\n", SLAVE_I2C_ID_DWI2C,
                        g_debug_cmd_argu[0], g_debug_cmd_arg_count);

                if (g_debug_read_data_size == 0) {
                    break;  // No need to execute I2C SMBUS read command later. So, break here.
                }
            }
        }

        if (g_debug_read_data_size > 0) // Send read command
        {
            TS_LOG_INFO("Execute I2C SMBUS read command\n");

            mdelay(I2C_WRITE_COMMAND_DELAY_FOR_FIRMWARE);
            rc = mstar_iic_read_data(SLAVE_I2C_ID_DWI2C, &szSmBusRxData[0], g_debug_read_data_size);
            if (rc > 0) {
                TS_LOG_INFO("mstar_iic_read_data(0x%X, 0x%X, %d) success\n", SLAVE_I2C_ID_DWI2C,
                        szSmBusRxData[0], g_debug_read_data_size);
                break;
            }
        }

        i++;
    }
    if (i == 5) {
        TS_LOG_INFO("mstar_iic_write_data() & mstar_iic_read_data() failed, rc = %d\n", rc);
    }

    for (i = 0; i < g_debug_read_data_size; i++)    // Output format 2.
    {
        TS_LOG_INFO("szSmBusRxData[%d] = 0x%x\n", i, szSmBusRxData[i]);

        sprintf(szValue, "%02x", szSmBusRxData[i]);
        strcat(szOut, szValue);

        if (i < (g_debug_read_data_size - 1)) {
            strcat(szOut, ",");
        }
    }

    TS_LOG_INFO("*** %s() *** mutex_unlock(&g_mutex)\n", __func__); // add for debug
    mutex_unlock(&g_mutex);
    nLength = strlen(szOut);
    if (copy_to_user(pBuffer, szOut, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_smbus_debug_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                           loff_t * pPos)
{
    u32 i, j;
    char szCmdType[5] = { 0 };
    char *pCh = NULL;
    char *pStr = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("*** pBuffer[0] = %c ***\n", g_debug_buf[0]);
        TS_LOG_INFO("*** pBuffer[1] = %c ***\n", g_debug_buf[1]);
        TS_LOG_INFO("*** pBuffer[2] = %c ***\n", g_debug_buf[2]);
        TS_LOG_INFO("*** pBuffer[3] = %c ***\n", g_debug_buf[3]);
        TS_LOG_INFO("*** pBuffer[4] = %c ***\n", g_debug_buf[4]);
        TS_LOG_INFO("*** pBuffer[5] = %c ***\n", g_debug_buf[5]);

        TS_LOG_INFO("nCount = %d\n", (int)nCount);

        // Reset to 0 before parsing the adb command
        g_debug_cmd_arg_count = 0;
        g_debug_read_data_size = 0;

        g_debug_buf[nCount] = '\0';
        pStr = g_debug_buf;

        i = 0;
        j = 0;

        while ((pCh = strsep((char **)&pStr, " ,")) && (j < MAX_DEBUG_COMMAND_ARGUMENT_NUM)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            if (strcmp(pCh, "w") == 0 || strcmp(pCh, "r") == 0) {
                memcpy(szCmdType, pCh, strlen(pCh));
            } else if (strcmp(szCmdType, "w") == 0) {
                g_debug_cmd_argu[j] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("g_debug_cmd_argu[%d] = 0x%02X\n", j, g_debug_cmd_argu[j]);

                j++;

                g_debug_cmd_arg_count = j;
                TS_LOG_INFO("g_debug_cmd_arg_count = %d\n", g_debug_cmd_arg_count);
            } else if (strcmp(szCmdType, "r") == 0) {
                sscanf(pCh, "%d", &g_debug_read_data_size);
                TS_LOG_INFO("g_debug_read_data_size = %d\n", g_debug_read_data_size);
            } else {
                TS_LOG_INFO("Un-supported adb command format!\n");
            }

            i++;
        }
    }

    return nCount;
}

static ssize_t mstar_apknode_fw_set_dq_mem_value_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    u32 i, nLength = 0;
    u8 nBank, nAddr;
    u32 szRegData[MAX_DEBUG_REGISTER_NUM] = { 0 };
    u8 szOut[MAX_DEBUG_REGISTER_NUM * 25] = { 0 }, szValue[10] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    for (i = 0; i < g_debug_reg_count; i++) {
        szRegData[i] = mstar_read_dq_mem_value(g_debug_reg[i]);
    }

    for (i = 0; i < g_debug_reg_count; i++) {
        nBank = (g_debug_reg[i] >> 8) & 0xFF;
        nAddr = g_debug_reg[i] & 0xFF;

        TS_LOG_INFO("reg(0x%02X,0x%02X)=0x%08X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%02X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%02X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }
    nLength = strlen(szOut);
    if (copy_to_user(pBuffer, szOut, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_set_dq_mem_value_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u32 i, j, k;
    char *pCh = NULL;
    char *pStr = NULL;
    u16 nRealDQMemAddr = 0;
    u32 nRealDQMemValue = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("*** pBuffer[0] = %c ***\n", g_debug_buf[0]);
        TS_LOG_INFO("*** pBuffer[1] = %c ***\n", g_debug_buf[1]);

        TS_LOG_INFO("nCount = %d\n", (int)nCount);
        g_debug_buf[nCount] = '\0';
        pStr = g_debug_buf;

        i = 0;
        j = 0;
        k = 0;

        while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            if ((i % 2) == 0) {
                g_debug_reg[j] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("g_debug_reg[%d] = 0x%04X\n", j, g_debug_reg[j]);
                j++;
            } else  // (i%2) == 1
            {
                g_debug_reg_value[k] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("g_debug_reg_value[%d] = 0x%04X\n", k, g_debug_reg_value[k]);
                k++;
            }

            i++;
        }
        g_debug_reg_count = j;

        TS_LOG_INFO("g_debug_reg_count = %d\n", g_debug_reg_count);

        if ((g_debug_reg[0] % 4) == 0) {
            nRealDQMemAddr = g_debug_reg[0];
            nRealDQMemValue = mstar_read_dq_mem_value(nRealDQMemAddr);
            g_debug_reg[0] = nRealDQMemAddr;
            TS_LOG_INFO("nRealDQMemValue Raw = %X\n", nRealDQMemValue);
            nRealDQMemValue &= 0xFFFF0000;
            nRealDQMemValue |= g_debug_reg_value[0];
            TS_LOG_INFO("nRealDQMemValue Modify = %X\n", nRealDQMemValue);
            mstar_write_dq_mem_value(nRealDQMemAddr, nRealDQMemValue);
        } else if ((g_debug_reg[0] % 4) == 2) {
            nRealDQMemAddr = g_debug_reg[0] - 2;
            nRealDQMemValue = mstar_read_dq_mem_value(nRealDQMemAddr);
            g_debug_reg[0] = nRealDQMemAddr;
            TS_LOG_INFO("nRealDQMemValue Raw = %X\n", nRealDQMemValue);

            nRealDQMemValue &= 0x0000FFFF;
            nRealDQMemValue |= (g_debug_reg_value[0] << 16);
            TS_LOG_INFO("nRealDQMemValue Modify = %X\n", nRealDQMemValue);
            mstar_write_dq_mem_value(nRealDQMemAddr, nRealDQMemValue);
        }
    }

    return nCount;
}

static ssize_t mstar_apknode_fw_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        mstar_mutual_get_fw_info(&g_mutual_fw_info);
        mutex_lock(&g_mutex_protect);
        g_fw_mode = g_mutual_fw_info.nFirmwareMode;
        mutex_unlock(&g_mutex_protect);
        TS_LOG_INFO("%s() firmware mode = 0x%x\n", __func__, g_mutual_fw_info.nFirmwareMode);

        nLength = sprintf(nUserTempBuffer, "%x", g_mutual_fw_info.nFirmwareMode);
    }
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }
    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                        loff_t * pPos)
{
    u32 nMode = 0;
    memset(g_debug_buf, 0, 16);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_debug_buf != NULL) {
        sscanf(g_debug_buf, "%x", &nMode);
        TS_LOG_INFO("firmware mode = 0x%x\n", nMode);

        g_switch_mode_apk = 0;

        if (nMode == FIRMWARE_MODE_DEMO_MODE)   //demo mode
        {
            g_fw_mode = mstar_change_fw_mode(FIRMWARE_MODE_DEMO_MODE);
        } else if (nMode == FIRMWARE_MODE_DEBUG_MODE)   //debug mode
        {
            g_fw_mode = mstar_change_fw_mode(FIRMWARE_MODE_DEBUG_MODE);
            g_switch_mode_apk = 1;
            g_debug_log_time_stamp = 0; // Set g_debug_log_time_stamp for filter duplicate packet on MTPTool APK
        } else {
            TS_LOG_INFO("*** Undefined Firmware Mode ***\n");
        }
    }

    TS_LOG_INFO("*** g_fw_mode = 0x%x ***\n", g_fw_mode);

    return nCount;
}

static ssize_t mstar_apknode_fw_sensor_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        if (g_mutual_fw_info.nLogModePacketHeader == 0xA7) {
            nLength =
                sprintf(nUserTempBuffer, "%d,%d,%d,%d", g_mutual_fw_info.nMx, g_mutual_fw_info.nMy,
                    g_mutual_fw_info.nSs, g_mutual_fw_info.nSd);
        } else {
            TS_LOG_INFO("Undefined debug mode packet format : 0x%x\n",
                    g_mutual_fw_info.nLogModePacketHeader);
            nLength = 0;
        }
    }
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }
    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_sensor_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    return nCount;
}

static ssize_t mstar_apknode_fw_packet_header_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2120
        || g_chip_type == CHIP_TYPE_ILI2121) {
        nLength = sprintf(nUserTempBuffer, "%d", g_mutual_fw_info.nLogModePacketHeader);

    }
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }
    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_fw_packet_header_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                             loff_t * pPos)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    return nCount;
}

static ssize_t _DrvKObjectPacketShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
    u32 nLength = 0;
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (strcmp(pAttr->attr.name, "packet") == 0) {
        if (g_fw_mode == FIRMWARE_MODE_DEMO_MODE) {
            if (g_demo_packet != NULL) {
                TS_LOG_INFO("g_fw_mode=%x, g_demo_packet[0]=%x, g_demo_packet[1]=%x\n", g_fw_mode,
                        g_demo_packet[0], g_demo_packet[1]);
                TS_LOG_INFO("g_demo_packet[2]=%x, g_demo_packet[3]=%x\n", g_demo_packet[2],
                        g_demo_packet[3]);
                TS_LOG_INFO("g_demo_packet[4]=%x, g_demo_packet[5]=%x\n", g_demo_packet[4],
                        g_demo_packet[5]);

                memcpy(pBuf, g_demo_packet, DEMO_MODE_PACKET_LENGTH);

                nLength = DEMO_MODE_PACKET_LENGTH;
                TS_LOG_INFO("nLength = %d\n", nLength);
            } else {
                TS_LOG_INFO("g_demo_packet is NULL\n");
            }
        } else      //g_fw_mode == FIRMWARE_MODE_DEBUG_MODE || g_fw_mode == FIRMWARE_MODE_RAW_DATA_MODE
        {
            if (g_log_packet != NULL) {
                TS_LOG_INFO("g_fw_mode=%x, g_log_packet[0]=%x, g_log_packet[1]=%x\n", g_fw_mode,
                        g_log_packet[0], g_log_packet[1]);
                TS_LOG_INFO("g_log_packet[2]=%x, g_log_packet[3]=%x\n", g_log_packet[2],
                        g_log_packet[3]);
                TS_LOG_INFO("g_log_packet[4]=%x, g_log_packet[5]=%x\n", g_log_packet[4],
                        g_log_packet[5]);

                if ((g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
                     || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A
                     || g_chip_type == CHIP_TYPE_ILI2120 || g_chip_type == CHIP_TYPE_ILI2121)
                    && (g_fw_mode == FIRMWARE_MODE_DEBUG_MODE) && (g_log_packet[0] == 0xA7)) {
                    memcpy(pBuf, g_log_packet, g_mutual_fw_info.nLogModePacketLength);

                    if (g_debug_log_time_stamp >= 255) {
                        g_debug_log_time_stamp = 0;
                    } else {
                        g_debug_log_time_stamp++;
                    }

                    pBuf[g_mutual_fw_info.nLogModePacketLength] = g_debug_log_time_stamp;
                    TS_LOG_INFO("g_debug_log_time_stamp=%d\n", pBuf[g_mutual_fw_info.nLogModePacketLength]);    // TODO : add for debug

                    nLength = g_mutual_fw_info.nLogModePacketLength + 1;
                    TS_LOG_INFO("nLength = %d\n", nLength);
                } else {
                    TS_LOG_INFO("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
                }
            } else {
                TS_LOG_INFO("g_log_packet is NULL\n");
            }
        }
    } else {
        TS_LOG_INFO("pAttr->attr.name = %s \n", pAttr->attr.name);
    }

    return nLength;
}

static ssize_t mstar_apknode_kobject_packet_store(struct kobject *pKObj, struct kobj_attribute *pAttr,
                           const char *pBuf, size_t nCount)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "packet") == 0)
    {

    }
*/
    return nCount;
}

static struct kobj_attribute packet_attr =
__ATTR(packet, 0664, _DrvKObjectPacketShow, mstar_apknode_kobject_packet_store);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
    &packet_attr.attr,
    NULL,           /* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
    .attrs = attrs,
};

//------------------------------------------------------------------------------//

static ssize_t mstar_apknode_query_feature_support_status_read(struct file *pFile, char __user * pBuffer,
                                size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    nLength = sprintf(nUserTempBuffer, "%d", g_feature_support_status);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    TS_LOG_INFO("*** g_feature_support_status = %d ***\n", g_feature_support_status);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_query_feature_support_status_write(struct file *pFile, const char __user * pBuffer,
                                 size_t nCount, loff_t * pPos)
{
    u32 nFeature = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        sscanf(g_debug_buf, "%x", &nFeature);
        TS_LOG_INFO("nFeature = 0x%x\n", nFeature);

        if (nFeature == FEATURE_GESTURE_WAKEUP_MODE) {
            g_feature_support_status = IS_GESTURE_WAKEUP_ENABLED;
        } else if (nFeature == FEATURE_GESTURE_DEBUG_MODE) {
            g_feature_support_status = IS_GESTURE_DEBUG_MODE_ENABLED;
        } else if (nFeature == FEATURE_GESTURE_INFORMATION_MODE) {
            g_feature_support_status = IS_GESTURE_INFORMATION_MODE_ENABLED;
        } else if (nFeature == FEATURE_TOUCH_DRIVER_DEBUG_LOG) {
            //g_feature_support_status = TOUCH_DRIVER_DEBUG_LOG_LEVEL;
        } else if (nFeature == FEATURE_FIRMWARE_DATA_LOG) {
            g_feature_support_status = IS_FIRMWARE_DATA_LOG_ENABLED;

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
            if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
                || g_chip_type == CHIP_TYPE_ILI2118A || g_chip_type == CHIP_TYPE_ILI2117A
                || g_chip_type == CHIP_TYPE_ILI2120 || g_chip_type == CHIP_TYPE_ILI2121) {
                if (g_feature_support_status == 1)  // If the debug mode data log function is supported, then get packet address and flag address for segment read finger touch data.
                {
                    mstar_get_touch_packet_addr(&g_fw_packet_data_addr, &g_fw_packet_flag_addr);
                }
            }
#endif //CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
        } else if (nFeature == FEATURE_FORCE_TO_UPDATE_FIRMWARE) {
            g_feature_support_status = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
        } else if (nFeature == FEATURE_DISABLE_ESD_PROTECTION_CHECK) {
            g_feature_support_status = IS_DISABLE_ESD_PROTECTION_CHECK;
        } else if (nFeature == FEATURE_APK_PRINT_FIRMWARE_SPECIFIC_LOG) {
            g_feature_support_status = IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED;
        } else if (nFeature == FEATURE_SELF_FREQ_SCAN) {
            TS_LOG_INFO("*** change to  FEATURE_SELF_FREQ_SCAN ***\n");
            g_feature_support_status = IS_SELF_FREQ_SCAN_ENABLED;
        } else {
            TS_LOG_INFO("*** Undefined Feature ***\n");
        }
    }

    TS_LOG_INFO("*** g_feature_support_status = %d ***\n", g_feature_support_status);

    return nCount;
}

static ssize_t mstar_apknode_change_feature_support_status_read(struct file *pFile, char __user * pBuffer,
                                 size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    nLength = sprintf(nUserTempBuffer, "%d", g_feature_support_status);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    TS_LOG_INFO("*** g_feature_support_status = %d ***\n", g_feature_support_status);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_change_feature_support_status_write(struct file *pFile, const char __user * pBuffer,
                                  size_t nCount, loff_t * pPos)
{
    u32 i;
    u32 nFeature = 0, nNewValue = 0;
    char *pCh = NULL;
    char *pStr = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("nCount = %d\n", (int)nCount);
        g_debug_buf[nCount] = '\0';
        pStr = g_debug_buf;

        i = 0;

        while ((pCh = strsep((char **)&pStr, " ,")) && (i < 3)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            if (i == 0) {
                nFeature = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("nFeature = 0x%04X\n", nFeature);
            } else if (i == 1) {
                nNewValue = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));
                TS_LOG_INFO("nNewValue = %d\n", nNewValue);
            } else {
                TS_LOG_INFO("End of parsing adb command.\n");
            }

            i++;
        }
        if (nFeature == FEATURE_GESTURE_WAKEUP_MODE) {
            IS_GESTURE_WAKEUP_ENABLED = nNewValue;
            g_feature_support_status = IS_GESTURE_WAKEUP_ENABLED;
        } else if (nFeature == FEATURE_GESTURE_DEBUG_MODE) {
            IS_GESTURE_DEBUG_MODE_ENABLED = nNewValue;
            g_feature_support_status = IS_GESTURE_DEBUG_MODE_ENABLED;
        } else if (nFeature == FEATURE_GESTURE_INFORMATION_MODE) {
            IS_GESTURE_INFORMATION_MODE_ENABLED = nNewValue;
            g_feature_support_status = IS_GESTURE_INFORMATION_MODE_ENABLED;
        } else if (nFeature == FEATURE_TOUCH_DRIVER_DEBUG_LOG) {
            //TOUCH_DRIVER_DEBUG_LOG_LEVEL = nNewValue;
            //g_feature_support_status = TOUCH_DRIVER_DEBUG_LOG_LEVEL;
        } else if (nFeature == FEATURE_FIRMWARE_DATA_LOG) {
            IS_FIRMWARE_DATA_LOG_ENABLED = nNewValue;
            g_feature_support_status = IS_FIRMWARE_DATA_LOG_ENABLED;
        } else if (nFeature == FEATURE_FORCE_TO_UPDATE_FIRMWARE) {
            IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = nNewValue;
            g_feature_support_status = IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
        } else if (nFeature == FEATURE_DISABLE_ESD_PROTECTION_CHECK) {
            IS_DISABLE_ESD_PROTECTION_CHECK = nNewValue;
            g_feature_support_status = IS_DISABLE_ESD_PROTECTION_CHECK;
        } else if (nFeature == FEATURE_APK_PRINT_FIRMWARE_SPECIFIC_LOG) {
            IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED = nNewValue;
            g_feature_support_status = IS_APK_PRINT_FIRMWARE_SPECIFIC_LOG_ENABLED;
        } else if (nFeature == FEATURE_SELF_FREQ_SCAN) {
            IS_SELF_FREQ_SCAN_ENABLED = nNewValue;
            g_feature_support_status = IS_SELF_FREQ_SCAN_ENABLED;
        } else {
            TS_LOG_INFO("*** Undefined Feature ***\n");
        }

        TS_LOG_INFO("*** g_feature_support_status = %d ***\n", g_feature_support_status);
    }

    return nCount;
}

//------------------------------------------------------------------------------//
static ssize_t mstar_apknode_gesture_wakeup_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
    TS_LOG_INFO("g_gesture_wakeup_mode = 0x%x, 0x%x\n", g_gesture_wakeup_mode[0], g_gesture_wakeup_mode[1]);

    nLength = sprintf(nUserTempBuffer, "%x,%x", g_gesture_wakeup_mode[0], g_gesture_wakeup_mode[1]);
#else
    TS_LOG_INFO("g_gesture_wakeup_mode = 0x%x\n", g_gesture_wakeup_mode[0]);

    nLength = sprintf(nUserTempBuffer, "%x", g_gesture_wakeup_mode[0]);
#endif //CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_gesture_wakeup_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u32 nLength = 0;
    u32 nWakeupMode[2] = { 0 };
    char *pStr = NULL;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    pStr = g_debug_buf;
    if (pStr != NULL) {
#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
        u32 i;
        char *pCh = NULL;

        i = 0;
        while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            nWakeupMode[i] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));

            TS_LOG_INFO("nWakeupMode[%d] = 0x%04X\n", i, nWakeupMode[i]);
            i++;
        }
#else
        sscanf(g_debug_buf, "%x", &nWakeupMode[0]);
        TS_LOG_INFO("nWakeupMode = 0x%x\n", nWakeupMode[0]);
#endif //CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE

        nLength = nCount;
        TS_LOG_INFO("nLength = %d\n", nLength);

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE1_FLAG) == GESTURE_WAKEUP_MODE_RESERVE1_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE1_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE1_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE2_FLAG) == GESTURE_WAKEUP_MODE_RESERVE2_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE2_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE2_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE3_FLAG) == GESTURE_WAKEUP_MODE_RESERVE3_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE3_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE3_FLAG);
        }

#ifdef CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE
        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE4_FLAG) == GESTURE_WAKEUP_MODE_RESERVE4_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE4_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE4_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE5_FLAG) == GESTURE_WAKEUP_MODE_RESERVE5_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE5_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE5_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE6_FLAG) == GESTURE_WAKEUP_MODE_RESERVE6_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE6_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE6_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE7_FLAG) == GESTURE_WAKEUP_MODE_RESERVE7_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE7_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE7_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE8_FLAG) == GESTURE_WAKEUP_MODE_RESERVE8_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE8_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE8_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE9_FLAG) == GESTURE_WAKEUP_MODE_RESERVE9_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE9_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE9_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE10_FLAG) == GESTURE_WAKEUP_MODE_RESERVE10_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE10_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE10_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE11_FLAG) == GESTURE_WAKEUP_MODE_RESERVE11_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE11_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE11_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE12_FLAG) == GESTURE_WAKEUP_MODE_RESERVE12_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE12_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE12_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE13_FLAG) == GESTURE_WAKEUP_MODE_RESERVE13_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE13_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE13_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE14_FLAG) == GESTURE_WAKEUP_MODE_RESERVE14_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE14_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE14_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE15_FLAG) == GESTURE_WAKEUP_MODE_RESERVE15_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE15_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE15_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE16_FLAG) == GESTURE_WAKEUP_MODE_RESERVE16_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE16_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE16_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE17_FLAG) == GESTURE_WAKEUP_MODE_RESERVE17_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE17_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE17_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE18_FLAG) == GESTURE_WAKEUP_MODE_RESERVE18_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE18_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE18_FLAG);
        }

        if ((nWakeupMode[0] & GESTURE_WAKEUP_MODE_RESERVE19_FLAG) == GESTURE_WAKEUP_MODE_RESERVE19_FLAG) {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] | GESTURE_WAKEUP_MODE_RESERVE19_FLAG;
        } else {
            g_gesture_wakeup_mode[0] = g_gesture_wakeup_mode[0] & (~GESTURE_WAKEUP_MODE_RESERVE19_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE20_FLAG) == GESTURE_WAKEUP_MODE_RESERVE20_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE20_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE20_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE21_FLAG) == GESTURE_WAKEUP_MODE_RESERVE21_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE21_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE21_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE22_FLAG) == GESTURE_WAKEUP_MODE_RESERVE22_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE22_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE22_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE23_FLAG) == GESTURE_WAKEUP_MODE_RESERVE23_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE23_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE23_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE24_FLAG) == GESTURE_WAKEUP_MODE_RESERVE24_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE24_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE24_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE25_FLAG) == GESTURE_WAKEUP_MODE_RESERVE25_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE25_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE25_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE26_FLAG) == GESTURE_WAKEUP_MODE_RESERVE26_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE26_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE26_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE27_FLAG) == GESTURE_WAKEUP_MODE_RESERVE27_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE27_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE27_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE28_FLAG) == GESTURE_WAKEUP_MODE_RESERVE28_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE28_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE28_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE29_FLAG) == GESTURE_WAKEUP_MODE_RESERVE29_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE29_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE29_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE30_FLAG) == GESTURE_WAKEUP_MODE_RESERVE30_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE30_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE30_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE31_FLAG) == GESTURE_WAKEUP_MODE_RESERVE31_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE31_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE31_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE32_FLAG) == GESTURE_WAKEUP_MODE_RESERVE32_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE32_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE32_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE33_FLAG) == GESTURE_WAKEUP_MODE_RESERVE33_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE33_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE33_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE34_FLAG) == GESTURE_WAKEUP_MODE_RESERVE34_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE34_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE34_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE35_FLAG) == GESTURE_WAKEUP_MODE_RESERVE35_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE35_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE35_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE36_FLAG) == GESTURE_WAKEUP_MODE_RESERVE36_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE36_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE36_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE37_FLAG) == GESTURE_WAKEUP_MODE_RESERVE37_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE37_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE37_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE38_FLAG) == GESTURE_WAKEUP_MODE_RESERVE38_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE38_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE38_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE39_FLAG) == GESTURE_WAKEUP_MODE_RESERVE39_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE39_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE39_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE40_FLAG) == GESTURE_WAKEUP_MODE_RESERVE40_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE40_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE40_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE41_FLAG) == GESTURE_WAKEUP_MODE_RESERVE41_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE41_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE41_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE42_FLAG) == GESTURE_WAKEUP_MODE_RESERVE42_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE42_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE42_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE43_FLAG) == GESTURE_WAKEUP_MODE_RESERVE43_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE43_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE43_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE44_FLAG) == GESTURE_WAKEUP_MODE_RESERVE44_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE44_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE44_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE45_FLAG) == GESTURE_WAKEUP_MODE_RESERVE45_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE45_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE45_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE46_FLAG) == GESTURE_WAKEUP_MODE_RESERVE46_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE46_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE46_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE47_FLAG) == GESTURE_WAKEUP_MODE_RESERVE47_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE47_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE47_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE48_FLAG) == GESTURE_WAKEUP_MODE_RESERVE48_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE48_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE48_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE49_FLAG) == GESTURE_WAKEUP_MODE_RESERVE49_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE49_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE49_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE50_FLAG) == GESTURE_WAKEUP_MODE_RESERVE50_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE50_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE50_FLAG);
        }

        if ((nWakeupMode[1] & GESTURE_WAKEUP_MODE_RESERVE51_FLAG) == GESTURE_WAKEUP_MODE_RESERVE51_FLAG) {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] | GESTURE_WAKEUP_MODE_RESERVE51_FLAG;
        } else {
            g_gesture_wakeup_mode[1] = g_gesture_wakeup_mode[1] & (~GESTURE_WAKEUP_MODE_RESERVE51_FLAG);
        }
#endif //CONFIG_SUPPORT_64_TYPES_GESTURE_WAKEUP_MODE

        TS_LOG_INFO("g_gesture_wakeup_mode = 0x%x,  0x%x\n", g_gesture_wakeup_mode[0],
                g_gesture_wakeup_mode[1]);
    }

    return nCount;
}

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
static ssize_t mstar_apknode_gesture_debug_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                              loff_t * pPos)
{
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    TS_LOG_INFO("g_GestureDebugMode = 0x%x\n", g_GestureDebugMode); // add for debug

    nLength = sprintf(nUserTempBuffer, "%d", g_GestureDebugMode);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_gesture_debug_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    u8 ucGestureMode[2] = { 0 };
    u8 i = 0;
    char *pCh = NULL;
    char *pStr = NULL;
    memset(g_debug_buf, 0, 1024);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    pStr = g_debug_buf;
    if (pStr != NULL) {
        i = 0;
        while ((pCh = strsep((char **)&pStr, " ,")) && (i < 2)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            ucGestureMode[i] = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));

            TS_LOG_INFO("ucGestureMode[%d] = 0x%04X\n", i, ucGestureMode[i]);
            i++;
        }

        g_GestureDebugMode = ucGestureMode[0];
        g_GestureDebugFlag = ucGestureMode[1];

        TS_LOG_INFO("Gesture flag = 0x%x\n", g_GestureDebugFlag);

        if (g_GestureDebugMode == 0x01) //open gesture debug mode
        {
            DrvOpenGestureDebugMode(g_GestureDebugFlag);

//            input_report_key(g_InputDevice, RESERVER42, 1);
            input_report_key(g_InputDevice, KEY_POWER, 1);
            input_sync(g_InputDevice);
//            input_report_key(g_InputDevice, RESERVER42, 0);
            input_report_key(g_InputDevice, KEY_POWER, 0);
            input_sync(g_InputDevice);
        } else if (g_GestureDebugMode == 0x00)  //close gesture debug mode
        {
            DrvCloseGestureDebugMode();
        } else {
            TS_LOG_INFO("*** Undefined Gesture Debug Mode ***\n");
        }
    }

    return nCount;
}

static ssize_t mstar_apknode_kboject_gesture_debug_show(struct kobject *pKObj, struct kobj_attribute *pAttr,
                             char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (strcmp(pAttr->attr.name, "gesture_debug") == 0) {
        if (g_LogGestureDebug != NULL) {
            TS_LOG_INFO("g_LogGestureDebug[0]=%x, g_LogGestureDebug[1]=%x\n", g_LogGestureDebug[0],
                    g_LogGestureDebug[1]);
            TS_LOG_INFO("g_LogGestureDebug[2]=%x, g_LogGestureDebug[3]=%x\n", g_LogGestureDebug[2],
                    g_LogGestureDebug[3]);
            TS_LOG_INFO("g_LogGestureDebug[4]=%x, g_LogGestureDebug[5]=%x\n", g_LogGestureDebug[4],
                    g_LogGestureDebug[5]);

            if (g_LogGestureDebug[0] == 0xA7 && g_LogGestureDebug[3] == 0x51) {
                for (i = 0; i < 0x80; i++) {
                    pBuf[i] = g_LogGestureDebug[i];
                }

                nLength = 0x80;
                TS_LOG_INFO("nLength = %d\n", nLength);
            } else {
                TS_LOG_INFO("CURRENT MODE IS NOT GESTURE DEBUG MODE/WRONG GESTURE DEBUG MODE HEADER\n");
            }
        } else {
            TS_LOG_INFO("g_LogGestureDebug is NULL\n");
        }
    } else {
        TS_LOG_INFO("pAttr->attr.name = %s \n", pAttr->attr.name);
    }

    return nLength;
}

static ssize_t mstar_apknode_kboject_gesture_debug_store(struct kobject *pKObj, struct kobj_attribute *pAttr,
                              const char *pBuf, size_t nCount)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    return nCount;
}

static struct kobj_attribute gesture_attr =
__ATTR(gesture_debug, 0664, mstar_apknode_kboject_gesture_debug_show, mstar_apknode_kboject_gesture_debug_store);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *gestureattrs[] = {
    &gesture_attr.attr,
    NULL,           /* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
struct attribute_group gestureattr_group = {
    .attrs = gestureattrs,
};
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static ssize_t mstar_apknode_gesture_info_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                             loff_t * pPos)
{
    u8 szOut[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH * 5] = { 0 }, szValue[10] = {
    0};
    u32 szLogGestureInfo[GESTURE_WAKEUP_INFORMATION_PACKET_LENGTH] = { 0 };
    u32 i = 0;
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    g_log_gesture_count = 0;
    if (g_log_gesture_infor_type == FIRMWARE_GESTURE_INFORMATION_MODE_A)    //FIRMWARE_GESTURE_INFORMATION_MODE_A
    {
        for (i = 0; i < 2; i++) //0 EventFlag; 1 RecordNum
        {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[4 + i];
            g_log_gesture_count++;
        }

        for (i = 2; i < 8; i++) //2~3 Xst Yst; 4~5 Xend Yend; 6~7 char_width char_height
        {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[4 + i];
            g_log_gesture_count++;
        }
    } else if (g_log_gesture_infor_type == FIRMWARE_GESTURE_INFORMATION_MODE_B) //FIRMWARE_GESTURE_INFORMATION_MODE_B
    {
        for (i = 0; i < 2; i++) //0 EventFlag; 1 RecordNum
        {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[4 + i];
            g_log_gesture_count++;
        }

        for (i = 0; i < g_gesture_log_info[5] * 2; i++) //(X and Y)*RecordNum
        {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[12 + i];
            g_log_gesture_count++;
        }
    } else if (g_log_gesture_infor_type == FIRMWARE_GESTURE_INFORMATION_MODE_C) //FIRMWARE_GESTURE_INFORMATION_MODE_C
    {
        for (i = 0; i < 6; i++) //header
        {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[i];
            g_log_gesture_count++;
        }

        for (i = 6; i < 86; i++) {
            szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[i];
            g_log_gesture_count++;
        }

        szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[86]; //dummy
        g_log_gesture_count++;
        szLogGestureInfo[g_log_gesture_count] = g_gesture_log_info[87]; //checksum
        g_log_gesture_count++;
    } else {
        TS_LOG_INFO("*** Undefined GESTURE INFORMATION MODE ***\n");
    }

    for (i = 0; i < g_log_gesture_count; i++) {
        sprintf(szValue, "%d", szLogGestureInfo[i]);
        strcat(szOut, szValue);
        strcat(szOut, ",");
    }

    nLength = strlen(szOut);

    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_gesture_info_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                              loff_t * pPos)
{
    u32 nMode = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 16);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        sscanf(g_debug_buf, "%x", &nMode);
        g_log_gesture_infor_type = nMode;
    }

    TS_LOG_INFO("*** g_log_gesture_infor_type type = 0x%x ***\n", g_log_gesture_infor_type);

    return nCount;
}
#endif //CONFIG_ENABLE_GESTURE_INFORMATION_MODE

#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
static ssize_t mstar_apknode_report_rate_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    struct timeval tEndTime;
    suseconds_t nStartTime, nEndTime, nElapsedTime;
    u32 nLength = 0;
    u8 nUserTempBuffer[16] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    TS_LOG_INFO("g_int_cont = %d, g_vaild_touch_cont = %d\n", g_int_cont, g_vaild_touch_cont);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }
    // Get end time
    do_gettimeofday(&tEndTime);

    nStartTime = g_start_time.tv_sec + g_start_time.tv_usec / 1000000;
    nEndTime = tEndTime.tv_sec + tEndTime.tv_usec / 1000000;

    nElapsedTime = nEndTime - nStartTime;

    TS_LOG_INFO("Start time : %lu sec, %lu msec\n", g_start_time.tv_sec, g_start_time.tv_usec);
    TS_LOG_INFO("End time : %lu sec, %lu msec\n", tEndTime.tv_sec, tEndTime.tv_usec);

    TS_LOG_INFO("Elapsed time : %lu sec\n", nElapsedTime);

    // Calculate report rate
    if (nElapsedTime != 0) {
        g_int_report_rate = g_int_cont / nElapsedTime;
        g_vaild_touch_report_rate = g_vaild_touch_cont / nElapsedTime;
    } else {
        g_int_report_rate = 0;
        g_vaild_touch_report_rate = 0;
    }

    TS_LOG_INFO("g_int_report_rate = %d, g_vaild_touch_report_rate = %d\n", g_int_report_rate,
            g_vaild_touch_report_rate);

    g_int_cont = 0;     // Reset count
    g_vaild_touch_cont = 0;

    nLength = sprintf(nUserTempBuffer, "%d,%d", g_int_report_rate, g_vaild_touch_report_rate);
    if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
        TS_LOG_INFO("copy to user error\n");
        return -EFAULT;
    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_report_rate_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                        loff_t * pPos)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 16);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");

        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        sscanf(g_debug_buf, "%d", &g_report_rate_enable);

        TS_LOG_INFO("g_report_rate_enable = %d\n", g_report_rate_enable);   // 1 : enable report rate calculation, 0 : disable report rate calculation, 2 : reset count

        g_int_cont = 0; // Reset count
        g_vaild_touch_cont = 0;
    }

    return nCount;
}
#endif //CONFIG_ENABLE_COUNT_REPORT_RATE

static ssize_t mstar_apknode_glove_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;
    u8 nGloveMode = 0;
    u8 nUserTempBuffer[16] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        mstar_finger_touch_report_disable();

        mstar_apknode_get_glove_info(&nGloveMode);

        mstar_finger_touch_report_enable();

        TS_LOG_INFO("Glove Mode = 0x%x\n", nGloveMode);

        nLength = sprintf(nUserTempBuffer, "%x", nGloveMode);
        if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
            TS_LOG_INFO("copy to user error\n");
            return -EFAULT;
        }

    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_glove_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                           loff_t * pPos)
{
    u32 nGloveMode = 0;
    u32 i = 0;
    char *pCh = NULL;
    char *pStr = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 16);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }

    pStr = g_debug_buf;
    if (g_debug_buf != NULL) {
        i = 0;
        while ((pCh = strsep((char **)&pStr, ",")) && (i < 1)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            nGloveMode = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));

            i++;
        }

        TS_LOG_INFO("Glove Mode = 0x%x\n", nGloveMode);

        mstar_finger_touch_report_disable();

        if (nGloveMode == 0x01) //open glove mode
        {
            mstar_apknode_open_glove();
        } else if (nGloveMode == 0x00)  //close glove mode
        {
            mstar_apknode_close_glove();
        } else {
            TS_LOG_INFO("*** Undefined Glove Mode ***\n");
        }
        TS_LOG_INFO("ans.g_IsEnableGloveMode = 0x%x\n", ans.g_IsEnableGloveMode);

        mstar_finger_touch_report_enable();
    }

    return nCount;
}

static ssize_t mstar_apknode_glove_open_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        mstar_finger_touch_report_disable();

        mstar_apknode_open_glove();

        mstar_finger_touch_report_enable();
    }
    TS_LOG_INFO("ans.g_IsEnableGloveMode = 0x%x\n", ans.g_IsEnableGloveMode);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_glove_close_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    u32 nLength = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        mstar_finger_touch_report_disable();

        mstar_apknode_close_glove();

        mstar_finger_touch_report_enable();
    }
    TS_LOG_INFO("g_IsEnableGloveMode = 0x%x\n", ans.g_IsEnableGloveMode);

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_leather_sheath_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                               loff_t * pPos)
{
    u32 nLength = 0;
    u8 nLeatherSheathMode = 0;
    u8 nUserTempBuffer[16] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // If file position is non-zero, then assume the string has been read and indicate there is no more data to be read.
    if (*pPos != 0) {
        return 0;
    }

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2118A
        || g_chip_type == CHIP_TYPE_ILI2117A) {
        mstar_finger_touch_report_disable();

        mstar_get_leather_sheath_info(&nLeatherSheathMode);

        mstar_finger_touch_report_enable();

        TS_LOG_INFO("Leather Sheath Mode = 0x%x\n", nLeatherSheathMode);

        nLength = sprintf(nUserTempBuffer, "%x", nLeatherSheathMode);
        if (copy_to_user(pBuffer, nUserTempBuffer, nLength)) {
            TS_LOG_INFO("copy to user error\n");
            return -EFAULT;
        }

    }

    *pPos += nLength;

    return nLength;
}

static ssize_t mstar_apknode_leather_sheath_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                            loff_t * pPos)
{
    u32 nLeatherSheathMode = 0;
    u32 i = 0;
    char *pCh = NULL;
    char *pStr = NULL;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 16);
    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");
        return -EFAULT;
    }
    pStr = g_debug_buf;
    if (pStr != NULL) {
        i = 0;
        while ((pCh = strsep((char **)&pStr, ",")) && (i < 1)) {
            TS_LOG_INFO("pCh = %s\n", pCh);

            nLeatherSheathMode = mstar_convert_char_to_hex_digit(pCh, strlen(pCh));

            i++;
        }

        TS_LOG_INFO("Leather Sheath Mode = 0x%x\n", nLeatherSheathMode);

        mstar_finger_touch_report_disable();

        if (nLeatherSheathMode == 0x01) //open leather sheath mode
        {
            mstar_apknode_open_leather_sheath();
        } else if (nLeatherSheathMode == 0x00)  //close leather sheath mode
        {
            mstar_apknode_close_leath_sheath();
        } else {
            TS_LOG_INFO("*** Undefined Leather Sheath Mode ***\n");
        }

        TS_LOG_INFO("ans.g_IsEnableLeatherSheathMode = 0x%x\n", ans.g_IsEnableLeatherSheathMode);

        mstar_finger_touch_report_enable();
    }

    return nCount;
}

static ssize_t mstar_apknode_trim_code_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    return nCount;
}

static ssize_t mstar_apknode_trim_code_read(struct file *pFile, char __user * pBuffer, size_t nCount, loff_t * pPos)
{
    struct file *pfile = NULL, *ffile = NULL;
    mm_segment_t fs;
    s32 i32Temp = 0;
    u8 nTrimCodeMin = 0, nTrimCodeMax = 0, nInitTrimCode = 0, nTempBuf[3] = { 0 }, nSetData = 0, i = 0;
    fs = get_fs();
    set_fs(KERNEL_DS);
    pfile = filp_open(ILITEK_TRIMCODE_INITIAL_PATH_ON_SD_CARD, O_RDONLY, 0);
    mstar_read_trim_code(0x143D, 3);
    if (IS_ERR(pfile)) {

        TS_LOG_INFO("Error occurred while opening file %s.\n", ILITEK_TRIMCODE_INITIAL_PATH_ON_SD_CARD);
        mdelay(100);
        ffile = filp_open(ILITEK_TRIMCODE_INITIAL_PATH_ON_SD_CARD, O_CREAT | O_RDWR, 0);
        sprintf(g_debug_buf, "trim code initial data:0x%x\n", g_read_trim_data[1]);
        nInitTrimCode = g_read_trim_data[1];
        ffile->f_op->write(ffile, g_debug_buf, strlen(g_debug_buf) * sizeof(char), &ffile->f_pos);
        set_fs(fs);
        filp_close(ffile, NULL);
    } else {
        pfile->f_op->read(pfile, g_debug_buf, 1024, &pfile->f_pos);
        sscanf(g_debug_buf, "trim code initial data:0x%x", &i32Temp);
        nInitTrimCode = (u8) i32Temp;
        TS_LOG_INFO("%s\n,nInitTrimCode=0x%x\n", g_debug_buf, nInitTrimCode);
    }
    nTrimCodeMax = nInitTrimCode + 2;
    if (nInitTrimCode - 2 < 0) {
        nTrimCodeMin = 0;
    } else {
        nTrimCodeMin = nInitTrimCode - 2;
    }
    TS_LOG_INFO("max:%d,min:%d,read trim:%d\n", nTrimCodeMax, nTrimCodeMin, g_read_trim_data[1]);
    {
        switch (g_read_trim_data[1])
        {
        case 0:
        case 64:
        case 128:
        case 192:
            nTempBuf[0] = g_read_trim_data[1] + 1;
            nTempBuf[1] = g_read_trim_data[2];
            TS_LOG_INFO("Read trim code: %d, modify level: 1\n", g_read_trim_data[1]);
            if (nTempBuf[0] < nTrimCodeMin && nTempBuf[0] < nTrimCodeMax) {
                TS_LOG_INFO("modify value overflow\n");
                return -1;
            }
            break;
        case 255:
        case 63:
        case 127:
        case 191:
            nTempBuf[0] = g_read_trim_data[1] - 1;
            nTempBuf[1] = g_read_trim_data[2];
            TS_LOG_INFO("Read trim code: %d, modify level: -1, -2\n", g_read_trim_data[1]);
            if (nTempBuf[0] < nTrimCodeMin && nTempBuf[0] < nTrimCodeMax) {
                TS_LOG_INFO("modify value overflow\n");
                return -1;
            }
            break;
        default:
            nTempBuf[0] = g_read_trim_data[1] - 1;
            nTempBuf[1] = g_read_trim_data[2];
            TS_LOG_INFO("Read trim code: %d, modify level: 1, -1, -2\n", g_read_trim_data[1]);
            if (nTempBuf[0] < nTrimCodeMin && nTempBuf[0] < nTrimCodeMax) {
                TS_LOG_INFO("modify value overflow\n");
                return -1;
            }
        }
    }
    nSetData = nTempBuf[0];
    for (i = 0; i < 6; i++) {
        mstar_write_trim_code(nTempBuf, 0);
        mstar_read_trim_code(0x143D, 3);
        if (g_read_trim_data[1] == nSetData) {
            TS_LOG_INFO("Set Trim code: %d,status:Pass\n", g_read_trim_data[1]);
            return 0;
            break;
        } else {
            TS_LOG_INFO("Set Trim code error,Read Trim code: %d,retry count:%d\n", g_read_trim_data[1], i);
        }
    }
    TS_LOG_INFO("Read Trim code: %d,status:error\n", g_read_trim_data[1]);
    return -1;
}

static long mstar_apknode_jni_msgtool_ioctl(struct file *pFile, unsigned int nCmd, unsigned long nArg)
{
    long nRet = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    switch (nCmd) {
    case MSGTOOL_IOCTL_RUN_CMD:
        {
            MsgToolDrvCmd_t *pTransCmd;
            pTransCmd = mstar_jni_trans_cmd_from_user(nArg);

            switch (pTransCmd->nCmdId) {
            case MSGTOOL_RESETHW:
                mstar_dev_hw_reset();
                break;
            case MSGTOOL_REGGETXBYTEVALUE:
                mstar_jni_reg_get_xbyte(pTransCmd);
                mstar_jni_trans_cmd_to_user(pTransCmd, nArg);
                break;
            case MSGTOOL_HOTKNOTSTATUS:
                g_rtn_cmd_data[0] = g_hotknot_enable;
                mstar_jni_trans_cmd_to_user(pTransCmd, nArg);
                break;
            case MSGTOOL_FINGERTOUCH:
                if (pTransCmd->nSndCmdLen == 1) {
                    TS_LOG_INFO("*** JNI enable touch ***\n");
                    mstar_finger_touch_report_enable();
                    g_finger_touch_disable = 0; // Resume finger touch ISR handling after MTPTool APK have sent i2c command to firmware.
                } else if (pTransCmd->nSndCmdLen == 0) {
                    TS_LOG_INFO("*** JNI disable touch ***\n");
                    mstar_finger_touch_report_disable();
                    g_finger_touch_disable = 1; // Skip finger touch ISR handling temporarily for MTPTool APK can send i2c command to firmware.
                }
                break;
            case MSGTOOL_BYPASSHOTKNOT:
                if (pTransCmd->nSndCmdLen == 1) {
                    TS_LOG_INFO("*** JNI enable bypass hotknot ***\n");
                    g_bypass_hotknot = 1;
                } else if (pTransCmd->nSndCmdLen == 0) {
                    TS_LOG_INFO("*** JNI disable bypass hotknot ***\n");
                    g_bypass_hotknot = 0;
                }
                break;
            case MSGTOOL_DEVICEPOWEROFF:
                mstar_enter_sleep_mode();
                break;
            case MSGTOOL_GETSMDBBUS:
                TS_LOG_INFO("*** MSGTOOL_GETSMDBBUS ***\n");
                g_rtn_cmd_data[0] = SLAVE_I2C_ID_DBBUS & 0xFF;
                g_rtn_cmd_data[1] = SLAVE_I2C_ID_DWI2C & 0xFF;
                mstar_jni_trans_cmd_to_user(pTransCmd, nArg);
                break;
            case MSGTOOL_SETIICDATARATE:
                TS_LOG_INFO("*** MSGTOOL_SETIICDATARATE ***\n");
                mstar_set_ii2c_data_rate(g_i2c_client,
                             ((g_snd_cmd_data[1] << 8) | g_snd_cmd_data[0]) * 1000);
                break;
            case MSGTOOL_ERASE_FLASH:
                TS_LOG_INFO("*** MSGTOOL_ERASE_FLASH ***\n");
                if (pTransCmd->nSndCmdDataPtr == 0) {
                    TS_LOG_INFO("*** erase Main block ***\n");
                    mstar_erase_emem(EMEM_MAIN);
                } else if (pTransCmd->nSndCmdDataPtr == 1) {
                    TS_LOG_INFO("*** erase INFO block ***\n");
                    mstar_erase_emem(EMEM_INFO);
                }
                break;
            default:
                break;
            }
        }
        break;

    default:
        nRet = -EINVAL;
        break;
    }

    return nRet;
}

static ssize_t mstar_apknode_jni_msgtool_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                        loff_t * pPos)
{
    long nRet = 0;
    u8 nBusType = 0;
    u16 nWriteLen = 0;
    u8 szCmdData[20] = { 0 };
    u8 buffer[128] = { 0 };
    TS_LOG_INFO("*** %s() ***\n", __func__);
    TS_LOG_INFO("*** nCount = %d ***\n", (int)nCount);
    // copy data from user space
    nBusType = nCount & 0xFF;
    if (nBusType == SLAVE_I2C_ID_DBBUS || nBusType == SLAVE_I2C_ID_DWI2C) {
        nWriteLen = (nCount >> 8) & 0xFFFF;
        nRet = copy_from_user(szCmdData, &pBuffer[0], nWriteLen);
        mstar_iic_write_data(nBusType, &szCmdData[0], nWriteLen);
    } else {
        nRet = copy_from_user(buffer, pBuffer, nCount - 1);
        if (nRet < 0) {
            printk("%s, copy data from user space, failed", __func__);
            return -1;
        }
        if (strcmp(buffer, "erase_flash") == 0) {
            TS_LOG_INFO("start Erase Flash\n");
            mstar_erase_emem(EMEM_MAIN);
            TS_LOG_INFO("end Erase Flash\n");
        }
    }
    return nCount;
}

static ssize_t mstar_apknode_set_film_mode_write(struct file *pFile, const char __user * pBuffer, size_t nCount,
                          loff_t * pPos)
{
    u8 nFilmType = 0;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    memset(g_debug_buf, 0, 1024);

    if (copy_from_user(g_debug_buf, pBuffer, nCount)) {
        TS_LOG_INFO("copy_from_user() failed\n");

        return -EFAULT;
    }
    if (g_debug_buf != NULL) {
        TS_LOG_INFO("nCount = %d\n", (int)nCount);
        g_debug_buf[nCount] = '\0';
        nFilmType = mstar_convert_char_to_hex_digit(g_debug_buf, strlen(g_debug_buf));
        TS_LOG_INFO("nFeature = 0x%02X\n", nFilmType);
        mstar_set_film_mode(nFilmType);
    }

    return nCount;
}

static ssize_t mstar_apknode_set_film_mode_read(struct file *pFile, char __user * pBuffer, size_t nCount,
                         loff_t * pPos)
{
    u8 nFilmType = 0;
    TS_LOG_INFO("*** %s() ***\n", __func__);
    nFilmType = mstar_get_film_mode();
    TS_LOG_INFO("*** %s() ***, nFilmType = %d\n", __func__, nFilmType);
    if (copy_to_user(pBuffer, &nFilmType, 1)) {
        return -EFAULT;
    }
    return 0;
}

static ssize_t class_ts_info_show(struct class *class, struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "mstar class_ts_info_show test");
}

static CLASS_ATTR(ts_info, S_IRUSR | S_IWUSR, class_ts_info_show, NULL);

static struct class *touchscreen_class;

static ssize_t gesture_show(struct class *class, struct class_attribute *attr, char *buf)
{
    if (ans.g_GestureState)
        return sprintf(buf, "gesture: on\n");
    else
        return sprintf(buf, "gesture: off\n");
}

static ssize_t gesture_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    if (!strncmp(buf, "on", 2))
        ans.g_GestureState = true;
    else if (!strncmp(buf, "off", 3))
        ans.g_GestureState = false;
    TS_LOG_DEBUG("buf = %s, g_GestureState = %d, count = %zu\n", buf, ans.g_GestureState, count);
    return count;
}

static CLASS_ATTR(gesture, S_IRUSR | S_IWUSR, gesture_show, gesture_store);

static const struct file_operations g_fops_mp_flow_switch = {
    .read = mstar_apknode_mp_flow_switch_read,
};

static const struct file_operations g_fops_chip_type = {
    .read = mstar_apknode_chip_type_read,
    .write = mstar_apknode_chip_type_write,
};

static const struct file_operations g_fops_fw_data = {
    .read = mstar_apknode_fw_data_read,
    .write = mstar_apknode_fw_data_write,
};

static const struct file_operations g_fops_apk_fw_update = {
    .read = mstar_apknode_fw_update_read,
    .write = mstar_apknode_fw_update_write,
};

static const struct file_operations g_fops_customer_fw_ver = {
    .read = mstar_apknode_customer_fw_ver_read,
    .write = mstar_apknode_customer_fw_ver_write,
};

static const struct file_operations g_fops_platform_fw_ver = {
    .read = mstar_apknode_platform_fw_ver_read,
    .write = mstar_apknode_platform_fw_ver_write,
};

static const struct file_operations g_fops_drive_ver = {
    .read = mstar_apknode_drive_ver_read,
    .write = mstar_apknode_drive_ver_write,
};

static const struct file_operations g_fops_sdcard_fw_update = {
    .read = mstar_apknode_fw_sdcard_update_read,
    .write = mstar_apknode_fw_sdcard_update_write,
};

static const struct file_operations g_fops_fw_debug = {
    .read = mstar_apknode_fw_debug_read,
    .write = mstar_apknode_fw_debug_write,
};

static const struct file_operations g_fops_fw_set_debug_value = {
    .read = mstar_apknode_fw_set_debug_value_read,
    .write = mstar_apknode_fw_set_debug_value_write,
};

static const struct file_operations g_fops_fw_smbus_debug = {
    .read = mstar_apknode_fw_smbus_debug_read,
    .write = mstar_apknode_fw_smbus_debug_write,
};

static const struct file_operations g_fops_fw_set_dq_mem_value = {
    .read = mstar_apknode_fw_set_dq_mem_value_read,
    .write = mstar_apknode_fw_set_dq_mem_value_write,
};

static const struct file_operations g_fops_fw_mode = {
    .read = mstar_apknode_fw_mode_read,
    .write = mstar_apknode_fw_mode_write,
};

static const struct file_operations g_fops_fw_sensor = {
    .read = mstar_apknode_fw_sensor_read,
    .write = mstar_apknode_fw_sensor_write,
};

static const struct file_operations g_fops_fw_packet_header = {
    .read = mstar_apknode_fw_packet_header_read,
    .write = mstar_apknode_fw_packet_header_write,
};

static const struct file_operations g_fops_query_feature_support_status = {
    .read = mstar_apknode_query_feature_support_status_read,
    .write = mstar_apknode_query_feature_support_status_write,
};

static const struct file_operations g_fops_change_feature_support_status = {
    .read = mstar_apknode_change_feature_support_status_read,
    .write = mstar_apknode_change_feature_support_status_write,
};

static const struct file_operations g_fops_gesture_wakeup_mode = {
    .read = mstar_apknode_gesture_wakeup_mode_read,
    .write = mstar_apknode_gesture_wakeup_mode_write,
};

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
static const struct file_operations g_fops_gesture_debug_mode = {
    .read = mstar_apknode_gesture_debug_mode_read,
    .write = mstar_apknode_gesture_debug_mode_write,
};
#endif
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
static const struct file_operations g_fops_gesture_info_mode = {
    .read = mstar_apknode_gesture_info_mode_read,
    .write = mstar_apknode_gesture_info_mode_write,
};
#endif
#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
static const struct file_operations g_fops_report_rate = {
    .read = mstar_apknode_report_rate_read,
    .write = mstar_apknode_report_rate_write,
};
#endif
static const struct file_operations g_fops_glove_mode = {
    .read = mstar_apknode_glove_mode_read,
    .write = mstar_apknode_glove_mode_write,
};

static const struct file_operations g_fops_open_glove_mode = {
    .read = mstar_apknode_glove_open_read,
};

static const struct file_operations g_fops_close_glove_mode = {
    .read = mstar_apknode_glove_close_read,
};

static const struct file_operations g_fops_leather_sheath_mode = {
    .read = mstar_apknode_leather_sheath_mode_read,
    .write = mstar_apknode_leather_sheath_mode_write,
};

static const struct file_operations g_fops_film_mode = {
    .read = mstar_apknode_set_film_mode_read,
    .write = mstar_apknode_set_film_mode_write,
};

#ifdef CONFIG_ENABLE_JNI_INTERFACE
static const struct file_operations g_fops_jni_method = {
    .read = mstar_jni_msg_tool_read,
    .write = mstar_apknode_jni_msgtool_write,
    .unlocked_ioctl = mstar_apknode_jni_msgtool_ioctl,
    .compat_ioctl = mstar_apknode_jni_msgtool_ioctl,
};
#endif
static const struct file_operations g_fops_selinux_fw_update = {
    .read = mstar_apknode_selinux_fw_update_read,
};

static const struct file_operations g_fops_force_fw_update = {
    .read = mstar_apknode_force_fw_update_read,
};

static const struct file_operations _gProcTrimCode = {
    .read = mstar_apknode_trim_code_read,
    .write = mstar_apknode_trim_code_write,
};

#ifdef MP_TEST_FUNCTION_2
static const struct file_operations g_fops_mp_test_customised = {
    .write = mstar_apknode_mp_test_customised_write,
    .read = mstar_apknode_mp_test_customised_read,
};
#endif
static const struct file_operations g_fops_proximity = {
    .write = mstar_apknode_proximity_write,
};

struct procfs_table {
    char *node_name;
    struct proc_dir_entry *node;
    const struct file_operations *fops;
    bool is_created;
};

struct procfs_table p_table[] = {
#ifdef MP_TEST_FUNCTION_2
    {PROC_NODE_MP_TEST_CUSTOMISED, NULL, &g_fops_mp_test_customised, false},
#endif
    {PROC_NODE_CHIP_TYPE, NULL, &g_fops_chip_type, false},
    {PROC_NODE_FIRMWARE_DATA, NULL, &g_fops_fw_data, false},
    {PROC_NODE_FIRMWARE_UPDATE, NULL, &g_fops_apk_fw_update, false},
    {PROC_NODE_CUSTOMER_FIRMWARE_VERSION, NULL, &g_fops_customer_fw_ver, false},
    {PROC_NODE_PLATFORM_FIRMWARE_VERSION, NULL, &g_fops_platform_fw_ver, false},
    {PROC_NODE_DEVICE_DRIVER_VERSION, NULL, &g_fops_drive_ver, false},
    {PROC_NODE_SD_CARD_FIRMWARE_UPDATE, NULL, &g_fops_sdcard_fw_update, false},
    {PROC_NODE_FIRMWARE_DEBUG, NULL, &g_fops_fw_debug, false},
    {PROC_NODE_FIRMWARE_SET_DEBUG_VALUE, NULL, &g_fops_fw_set_debug_value, false},
    {PROC_NODE_FIRMWARE_SMBUS_DEBUG, NULL, &g_fops_fw_smbus_debug, false},
    {PROC_NODE_FIRMWARE_SET_DQMEM_VALUE, NULL, &g_fops_fw_set_dq_mem_value, false},
    {PROC_NODE_FIRMWARE_MODE, NULL, &g_fops_fw_mode, false},
    {PROC_NODE_FIRMWARE_SENSOR, NULL, &g_fops_fw_sensor, false},
    {PROC_NODE_FIRMWARE_PACKET_HEADER, NULL, &g_fops_fw_packet_header, false},
    {PROC_NODE_QUERY_FEATURE_SUPPORT_STATUS, NULL, &g_fops_query_feature_support_status, false},
    {PROC_NODE_CHANGE_FEATURE_SUPPORT_STATUS, NULL, &g_fops_change_feature_support_status, false},
    {PROC_NODE_GESTURE_WAKEUP_MODE, NULL, &g_fops_gesture_wakeup_mode, false},
#ifdef CONFIG_ENABLE_GESTURE_INFORMATION_MODE
    {PROC_NODE_GESTURE_INFORMATION_MODE, NULL, &g_fops_gesture_info_mode, false},
#endif
#ifdef CONFIG_ENABLE_COUNT_REPORT_RATE
    {PROC_NODE_REPORT_RATE, NULL, &g_fops_report_rate, false},
#endif
    {PROC_NODE_GLOVE_MODE, NULL, &g_fops_glove_mode, false},
    {PROC_NODE_OPEN_GLOVE_MODE, NULL, &g_fops_open_glove_mode, false},
    {PROC_NODE_CLOSE_GLOVE_MODE, NULL, &g_fops_close_glove_mode, false},
    {PROC_NODE_LEATHER_SHEATH_MODE, NULL, &g_fops_leather_sheath_mode, false},
    {PROC_NODE_CONTROL_FILM_MODE, NULL, &g_fops_film_mode, false},
#ifdef CONFIG_ENABLE_JNI_INTERFACE
    {PROC_NODE_JNI_NODE, NULL, &g_fops_jni_method, false},
#endif
    {PROC_NODE_SELINUX_LIMIT_FIRMWARE_UPDATE, NULL, &g_fops_selinux_fw_update, false},
    {PROC_NODE_FORCE_FIRMWARE_UPDATE, NULL, &g_fops_force_fw_update, false},
    {PROC_NODE_TRIM_CODE, NULL, &_gProcTrimCode, false},
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
    {PROC_NODE_PROXIMITY_MODE, NULL, &g_fops_proximity, false},
#endif
    {"mp_flow_switch", NULL, &g_fops_mp_flow_switch, false},
};

int mstar_apknode_create_procfs(void)
{
    int i = 0;
    s32 ret = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    ans.g_IsEnableLeatherSheathMode = 0;
    ans.g_IsEnableGloveMode = 0;
    ans.g_GestureState = false;

    g_proc_class_entry = proc_mkdir(PROC_NODE_CLASS, NULL);

    g_proc_ts_msg20xx_entry = proc_mkdir(PROC_NODE_MS_TOUCHSCREEN_MSG20XX, g_proc_class_entry);

    g_proc_device_entry = proc_mkdir(PROC_NODE_DEVICE, g_proc_ts_msg20xx_entry);

    /* A main loop to create apk nodes */
    for (i = 0; i < ARRAY_SIZE(p_table); i++) {
        p_table[i].node =
            proc_create(p_table[i].node_name, PROCFS_AUTHORITY, g_proc_device_entry, p_table[i].fops);

        if (p_table[i].node == NULL) {
            TS_LOG_ERR("Failed to allocate %s mem\n", p_table[i].node_name);
        }

        p_table[i].is_created = true;
        TS_LOG_DEBUG("Created -> %s node successfully\n", p_table[i].node_name);
    }

    /* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
    g_touch_kset = kset_create_and_add("kset_example", NULL, kernel_kobj);
    if (!g_touch_kset) {
        TS_LOG_INFO("*** kset_create_and_add() failed, ret = %d ***\n", ret);
        return -ENOMEM;
    }

    g_touch_kobj = kobject_create();
    if (!g_touch_kobj) {
        TS_LOG_INFO("*** kobject_create() failed, ret = %d ***\n", ret);

        kset_unregister(g_touch_kset);
        g_touch_kset = NULL;
        return -ENOMEM;
    }

    g_touch_kobj->kset = g_touch_kset;

    ret = kobject_add(g_touch_kobj, NULL, "%s", "kobject_example");
    if (ret != 0) {
        TS_LOG_INFO("*** kobject_add() failed, ret = %d ***\n", ret);

        kobject_put(g_touch_kobj);
        g_touch_kobj = NULL;
        kset_unregister(g_touch_kset);
        g_touch_kset = NULL;
        return -ENOMEM;
    }

    /* create the files associated with this kobject */
    ret = sysfs_create_group(g_touch_kobj, &attr_group);
    if (ret != 0) {
        TS_LOG_INFO("*** sysfs_create_file() failed, ret = %d ***\n", ret);

        kobject_put(g_touch_kobj);
        g_touch_kobj = NULL;
        kset_unregister(g_touch_kset);
        g_touch_kset = NULL;
        return -ENOMEM;
    }

    touchscreen_class = class_create(THIS_MODULE, "touchscreen");
    if (IS_ERR_OR_NULL(touchscreen_class)) {
        TS_LOG_INFO("%s: create class error!\n", __func__);
        return -ENOMEM;
    }

    ret = class_create_file(touchscreen_class, &class_attr_ts_info);
    if (ret < 0) {
        TS_LOG_INFO("%s class_create_file failed!\n", __func__);
        class_destroy(touchscreen_class);
        return -ENOMEM;
    }

    ret = class_create_file(touchscreen_class, &class_attr_gesture);
    if (ret < 0) {
        TS_LOG_INFO("%s create gesture file failed!\n", __func__);
        class_destroy(touchscreen_class);
        return -ENOMEM;
    }
#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
    g_proc_gesture_debug_entry =
        proc_create(PROC_NODE_GESTURE_DEBUG_MODE, PROCFS_AUTHORITY, g_proc_device_entry,
            &g_fops_gesture_debug_mode);
    if (NULL == g_proc_gesture_debug_entry) {
        TS_LOG_INFO("Failed to create procfs file node(%s)!\n", PROC_NODE_GESTURE_DEBUG_MODE);
        return -ENOMEM;
    } else {
        TS_LOG_INFO("Create procfs file node(%s) OK!\n", PROC_NODE_GESTURE_DEBUG_MODE);
    }

    /* create a kset with the name of "kset_gesture" which is located under /sys/kernel/ */
    g_GestureKSet = kset_create_and_add("kset_gesture", NULL, kernel_kobj);
    if (!g_GestureKSet) {
        TS_LOG_INFO("*** kset_create_and_add() failed, ret = %d ***\n", ret);
        return -ENOMEM;
    }

    g_GestureKObj = kobject_create();
    if (!g_GestureKObj) {
        TS_LOG_INFO("*** kobject_create() failed, ret = %d ***\n", ret);

        kset_unregister(g_GestureKSet);
        g_GestureKSet = NULL;
        return -ENOMEM;
    }

    g_GestureKObj->kset = g_GestureKSet;

    ret = kobject_add(g_GestureKObj, NULL, "%s", "kobject_gesture");
    if (ret != 0) {
        TS_LOG_INFO("*** kobject_add() failed, ret = %d ***\n", ret);

        kobject_put(g_GestureKObj);
        g_GestureKObj = NULL;
        kset_unregister(g_GestureKSet);
        g_GestureKSet = NULL;
        return -ENOMEM;
    }

    /* create the files associated with this g_GestureKObj */
    ret = sysfs_create_group(g_GestureKObj, &gestureattr_group);
    if (ret != 0) {
        TS_LOG_INFO("*** sysfs_create_file() failed, ret = %d ***\n", ret);

        kobject_put(g_GestureKObj);
        g_GestureKObj = NULL;
        kset_unregister(g_GestureKSet);
        g_GestureKSet = NULL;
        return -ENOMEM;
    }
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

    return ret;
}

void mstar_apknode_remove_procfs(void)
{
    int i;

    TS_LOG_INFO("%s\n", __func__);

    /* A main loop to destroy nodes */
    for (i = 0; i < ARRAY_SIZE(p_table); i++) {
        if (p_table[i].is_created) {
            remove_proc_entry(p_table[i].node_name, g_proc_device_entry);
        }
    }

    /* Destroy device dir */
    if (g_proc_device_entry != NULL) {
        remove_proc_entry(PROC_NODE_DEVICE, g_proc_ts_msg20xx_entry);
        g_proc_device_entry = NULL;
        TS_LOG_INFO("Remove procfs file node(%s) OK!\n", PROC_NODE_DEVICE);
    }

    /* Destroy ms-touchscreen-msg20xx dir */
    if (g_proc_ts_msg20xx_entry != NULL) {
        remove_proc_entry(PROC_NODE_MS_TOUCHSCREEN_MSG20XX, g_proc_class_entry);
        g_proc_ts_msg20xx_entry = NULL;
        TS_LOG_INFO("Remove procfs file node(%s) OK!\n", PROC_NODE_MS_TOUCHSCREEN_MSG20XX);
    }

    /* Destroy class dir */
    if (g_proc_class_entry != NULL) {
        remove_proc_entry(PROC_NODE_CLASS, NULL);
        g_proc_class_entry = NULL;
        TS_LOG_INFO("Remove procfs file node(%s) OK!\n", PROC_NODE_CLASS);
    }
}
