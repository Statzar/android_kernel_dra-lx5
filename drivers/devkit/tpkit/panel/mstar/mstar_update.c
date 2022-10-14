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
 * @file    ilitek_drv_update.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_common.h"
#include <linux/firmware.h>

/*=============================================================*/
// VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
/*
 * Note.
 * Please modify the name of the below .h depends on the vendor TP that you are using.
 */

#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static SwIdData_t g_swid_data;
#endif

static u32 g_update_retry_cont = UPDATE_FIRMWARE_RETRY_COUNT;
static u32 g_update_info_block_first = 0;
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

u8 *g_one_dimen_fw_data = NULL;
static u8 *g_fw_data_buf = NULL;

static u16 g_sfr_addr3_byte0_to_1 = 0x0000; // for MSG28xx
static u16 g_sfr_addr3_byte2_to_3 = 0x0000;

extern struct i2c_client *g_i2c_client;
extern u32 SLAVE_I2C_ID_DBBUS;
extern u32 SLAVE_I2C_ID_DWI2C;
extern u8 IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED;
extern struct mutex g_mutex;

extern u8 **g_fw_data;
extern u32 g_fw_data_cont;
extern u8 g_fw_ver_flag;
extern u16 g_chip_type;
extern u16 g_chip_type_ori;

extern u8 g_fw_update;
extern u8 *g_chip_num;
extern char mstar_project_id[MSTAR_PROJECT_ID_LEN];

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u8 g_gesture_wakeup_flag;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

static void mstar_read_dq_mem_start(void);
static void mstar_read_dq_mem_end(void);

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static void mstar_update_fw_swid_do_work(struct work_struct *pWork);
#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static void mstar_fw_update_swid(void);
#endif //CONFIG_ENABLE_CHIP_TYPE_MSG28XX
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

void mstar_optimize_current_consump(void)
{
    u32 i;
    u8 szDbBusTxData[35] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);
    TS_LOG_INFO("g_chip_type = 0x%x\n", g_chip_type);

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_gesture_wakeup_flag == 1) {
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2118A) // CHIP_TYPE_MSG58XXA not need to execute the following code
    {
        mutex_lock(&g_mutex);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
        DmaReset();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

        mstar_dev_hw_reset();

        mstar_dbbus_enter_serial_debug();
        mstar_dbbus_stop_mcu();
        mstar_dbbus_iic_use_bus();
        mstar_dbbus_iic_reshape();

        // Enable burst mode
        mstar_set_reg_low_byte(0x1608, 0x21);

        szDbBusTxData[0] = 0x10;
        szDbBusTxData[1] = 0x15;
        szDbBusTxData[2] = 0x20;    //bank:0x15, addr:h0010

        for (i = 0; i < 8; i++) {
            szDbBusTxData[i + 3] = 0xFF;
        }

        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3 + 8); // Write 0xFF for reg 0x1510~0x1513

        szDbBusTxData[0] = 0x10;
        szDbBusTxData[1] = 0x15;
        szDbBusTxData[2] = 0x28;    //bank:0x15, addr:h0014

        for (i = 0; i < 16; i++) {
            szDbBusTxData[i + 3] = 0x00;
        }

        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3 + 16);    // Write 0x00 for reg 0x1514~0x151B

        szDbBusTxData[0] = 0x10;
        szDbBusTxData[1] = 0x21;
        szDbBusTxData[2] = 0x40;    //bank:0x21, addr:h0020

        for (i = 0; i < 8; i++) {
            szDbBusTxData[i + 3] = 0xFF;
        }

        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3 + 8); // Write 0xFF for reg 0x2120~0x2123

        szDbBusTxData[0] = 0x10;
        szDbBusTxData[1] = 0x21;
        szDbBusTxData[2] = 0x20;    //bank:0x21, addr:h0010

        for (i = 0; i < 32; i++) {
            szDbBusTxData[i + 3] = 0xFF;
        }

        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3 + 32);    // Write 0xFF for reg 0x2110~0x211F

        // Clear burst mode
        mstar_set_reg_low_byte(0x1608, 0x20);

        mstar_dbbus_iic_not_use_bus();
        mstar_dbbus_not_stop_mcu();
        mstar_dbbus_exit_serial_debug();

        mutex_unlock(&g_mutex);
    }
}

u16 mstar_get_chip_type(void)
{
    s32 rc = 0;
    u16 nChipType = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    // Check chip type by using DbBus for MSG22XX/MSG28XX/MSG58XX/MSG58XXA/ILI2117A/ILI2118A.
    // Erase TP Flash first
    rc = mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    mstar_dbbus_i2c_response_ack();

    // Stop MCU
    mstar_set_reg_low_byte(0x0FE6, 0x01);

#ifdef CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_SUPPORT_I2C_SPEED_400K    // for MSG22xx only
    // Exit flash low power mode
    mstar_set_reg_low_byte(0x1619, BIT1);

    // Change PIU clock to 48MHz
    mstar_set_reg_low_byte(0x1E23, BIT6);

    // Change mcu clock deglitch mux source
    mstar_set_reg_low_byte(0x1E54, BIT0);
#endif //CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_SUPPORT_I2C_SPEED_400K

    // Set Password
    mstar_set_reg_low_byte(0x1616, 0xAA);
    mstar_set_reg_low_byte(0x1617, 0x55);
    mstar_set_reg_low_byte(0x1618, 0xA5);
    mstar_set_reg_low_byte(0x1619, 0x5A);

    // disable cpu read, in,tial); read
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);
    mstar_set_reg_low_byte(0x1607, 0x00);

    // set info block
    mstar_set_reg_low_byte(0x1607, 0x08);
    // set info double buffer
    mstar_set_reg_low_byte(0x1604, 0x01);

    // set eflash mode to read mode
    mstar_set_reg_low_byte(0x1606, 0x01);
    mstar_set_reg_low_byte(0x1610, 0x01);
    mstar_set_reg_low_byte(0x1611, 0x00);

    // set read address
    mstar_set_reg_low_byte(0x1600, 0x05);
    mstar_set_reg_low_byte(0x1601, 0x00);

    nChipType = mstar_get_reg_16bit(0x160A) & 0xFFFF;

    if (nChipType == CHIP_TYPE_ILI2117A || nChipType == CHIP_TYPE_ILI2118A) {
        TS_LOG_INFO("----------------------ILI Chip Type=0x%x-------------------------\n", nChipType);
        g_chip_type_ori = nChipType;
    } else if (nChipType == CHIP_TYPE_MSG2836A || nChipType == CHIP_TYPE_MSG2846A || nChipType == CHIP_TYPE_MSG5846A
           || nChipType == CHIP_TYPE_MSG5856A) {
        g_chip_type_ori = nChipType;
        sprintf(g_chip_num, "MSG%xA", nChipType);
        TS_LOG_INFO("----------------------MSG Chip Type=0x%x, %s-------------------------\n", nChipType,
                g_chip_num);
        nChipType = CHIP_TYPE_MSG58XXA;
    } else {
        nChipType = mstar_get_reg_16bit(0x1ECC) & 0xFF;

        TS_LOG_INFO("----------------------MSG Chip Type=0x%x-------------------------\n", nChipType);
        g_chip_type_ori = nChipType;

        if (nChipType != CHIP_TYPE_MSG21XX &&   // (0x01)
            nChipType != CHIP_TYPE_MSG21XXA &&  // (0x02)
            nChipType != CHIP_TYPE_MSG26XXM &&  // (0x03)
            nChipType != CHIP_TYPE_MSG22XX &&   // (0x7A)
            nChipType != CHIP_TYPE_MSG28XX &&   // (0x85)
            nChipType != CHIP_TYPE_MSG58XXA)    // (0xBF)
        {
            if (nChipType != 0) {
                nChipType = CHIP_TYPE_MSG58XXA;
            }
        }
    }
    TS_LOG_INFO("*** g_chip_type_ori = 0x%x ***\n", g_chip_type_ori);
    TS_LOG_INFO("*** Chip Type = 0x%x ***\n", nChipType);

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    return nChipType;
}

void mstar_get_customer_fw_ver_dbbus(EmemType_e eEmemType, u16 * pMajor, u16 * pMinor, u8 ** ppVersion) // support MSG28xx only
{
    u16 nReadAddr = 0;
    u8 szTmpData[4] = { 0 };

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2118A) {
        mstar_dbbus_enter_serial_debug();
        mstar_dbbus_stop_mcu();
        mstar_dbbus_iic_use_bus();
        mstar_dbbus_iic_reshape();

        // Stop mcu
        mstar_set_reg_low_byte(0x0FE6, 0x01);

        if (eEmemType == EMEM_MAIN) // Read SW ID from main block
        {
            mstar_read_eflash_start(0x7FFD, EMEM_MAIN);
            nReadAddr = 0x7FFD;
        } else if (eEmemType == EMEM_INFO)  // Read SW ID from info block
        {
            mstar_read_eflash_start(0x81FB, EMEM_INFO);
            nReadAddr = 0x81FB;
        }

        mstar_read_eflash_do_read(nReadAddr, &szTmpData[0]);

        mstar_read_eflash_end();

        /*
           Ex. Major in Main Block :
           Major low byte at address 0x7FFD

           Major in Info Block :
           Major low byte at address 0x81FB
         */

        *pMajor = (szTmpData[1] << 8);
        *pMajor |= szTmpData[0];
        *pMinor = (szTmpData[3] << 8);
        *pMinor |= szTmpData[2];

        TS_LOG_INFO("*** Major = %d ***\n", *pMajor);
        TS_LOG_INFO("*** Minor = %d ***\n", *pMinor);

        if (*ppVersion == NULL) {
            *ppVersion = kzalloc(sizeof(u8) * 12, GFP_KERNEL);
        }

        sprintf(*ppVersion, "%05d.%05d", *pMajor, *pMinor);

        mstar_dbbus_iic_not_use_bus();
        mstar_dbbus_not_stop_mcu();
        mstar_dbbus_exit_serial_debug();
    }
}

static void mstar_fw_store_data(u8 * pBuf, u32 nSize)
{
    u32 nCount = nSize / 1024;
    u32 nRemainder = nSize % 1024;
    u32 i;

    if (nCount > 0)     // nSize >= 1024
    {
        for (i = 0; i < nCount; i++) {
            memcpy(g_fw_data[g_fw_data_cont], pBuf + (i * 1024), 1024);

            g_fw_data_cont++;
        }

        if (nRemainder > 0) // Handle special firmware size like MSG22XX(48.5KB)
        {
            TS_LOG_INFO("nRemainder = %d\n", nRemainder);

            memcpy(g_fw_data[g_fw_data_cont], pBuf + (i * 1024), nRemainder);

            g_fw_data_cont++;
        }
    } else          // nSize < 1024
    {
        if (nSize > 0) {
            memcpy(g_fw_data[g_fw_data_cont], pBuf, nSize);

            g_fw_data_cont++;
        }
    }
}

static void mstar_convert_fw_two_dimen_to_one(u8 ** szTwoDimenFwData, u8 * pOneDimenFwData)
{
    u32 i, j;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    for (i = 0; i < MSG28XX_FIRMWARE_WHOLE_SIZE; i++) {
        for (j = 0; j < 1024; j++) {
            pOneDimenFwData[i * 1024 + j] = szTwoDimenFwData[i][j];
        }
    }
}

static u32 mstar_calculate_crc(u8 * pFwData, u32 nOffset, u32 nSize)
{
    u32 i;
    u32 nData = 0, nCrc = 0;
    u32 nCrcRule = 0x0C470C06;  // 0000 1100 0100 0111 0000 1100 0000 0110

    for (i = 0; i < nSize; i += 4) {
        nData =
            (pFwData[nOffset + i]) | (pFwData[nOffset + i + 1] << 8) | (pFwData[nOffset + i + 2] << 16) |
            (pFwData[nOffset + i + 3] << 24);
        nCrc = (nCrc >> 1) ^ (nCrc << 1) ^ (nCrc & nCrcRule) ^ nData;
    }

    return nCrc;
}

static void mstar_access_eflash_init(void)
{
    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1606, 0x20);
    mstar_set_reg_low_byte(0x1608, 0x20);

    // Clear PROGRAM erase password
    mstar_set_reg_16bit(0x1618, 0xA55A);
}

static void mstar_isp_burst_write_eflash_start(u16 nStartAddr, u8 * pFirstData, u32 nBlockSize, u16 nPageNum,
                           EmemType_e eEmemType)
{
    u16 nWriteAddr = nStartAddr / 4;
    u8 szDbBusTxData[3] = { 0 };

    TS_LOG_INFO("*** %s() nStartAddr = 0x%x, nBlockSize = %d, nPageNum = %d, eEmemType = %d ***\n", __func__,
            nStartAddr, nBlockSize, nPageNum, eEmemType);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);

    // Set e-flash mode to page write mode
    mstar_set_reg_16bit(0x1606, 0x0080);

    // Set data align
    mstar_set_reg_low_byte(0x1640, 0x01);

    if (eEmemType == EMEM_INFO) {
        mstar_set_reg_low_byte(0x1607, 0x08);
    }
    // Set double buffer
    mstar_set_reg_low_byte(0x1604, 0x01);

    // Set page write number
    mstar_set_reg_16bit(0x161A, nPageNum);

    // Set e-flash mode trigger(Trigger write mode)
    mstar_set_reg_low_byte(0x1606, 0x81);

    // Set init data
    mstar_set_reg_low_byte(0x1602, pFirstData[0]);
    mstar_set_reg_low_byte(0x1602, pFirstData[1]);
    mstar_set_reg_low_byte(0x1602, pFirstData[2]);
    mstar_set_reg_low_byte(0x1602, pFirstData[3]);

    // Set initial address(for latch SA, CA)
    mstar_set_reg_16bit(0x1600, nWriteAddr);

    // Set initial address(for latch PA)
    mstar_set_reg_16bit(0x1600, nWriteAddr);

    // Enable burst mode
    mstar_set_reg_low_byte(0x1608, 0x21);

    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x16;
    szDbBusTxData[2] = 0x02;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3);

    szDbBusTxData[0] = 0x20;
//    szDbBusTxData[1] = 0x00;
//    szDbBusTxData[2] = 0x00;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 1);
}

static void mstar_isp_burst_write_eflash_do_write(u8 * pBufferData, u32 nLength)
{
    u32 i;
    u8 szDbBusTxData[3 + MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE] = { 0 };

    szDbBusTxData[0] = 0x10;
    szDbBusTxData[1] = 0x16;
    szDbBusTxData[2] = 0x02;

    for (i = 0; i < nLength; i++) {
        szDbBusTxData[3 + i] = pBufferData[i];
    }

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 3 + nLength);
}

static void mstar_isp_burst_write_eflash_end(void)
{
    u8 szDbBusTxData[1] = { 0 };

    TS_LOG_INFO("*** %s() ***\n", __func__);

    szDbBusTxData[0] = 0x21;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 1);

    szDbBusTxData[0] = 0x7E;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 1);

    // Clear burst mode
    mstar_set_reg_low_byte(0x1608, 0x20);
}

static void mstar_write_eflash_start(u16 nStartAddr, u8 * pFirstData, EmemType_e eEmemType)
{
    u16 nWriteAddr = nStartAddr / 4;

    TS_LOG_INFO("*** %s() nStartAddr = 0x%x, eEmemType = %d ***\n", __func__, nStartAddr, eEmemType);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);

    // Set e-flash mode to write mode
    mstar_set_reg_16bit(0x1606, 0x0040);

    // Set data align
    mstar_set_reg_low_byte(0x1640, 0x01);

    if (eEmemType == EMEM_INFO) {
        mstar_set_reg_low_byte(0x1607, 0x08);
    }
    // Set double buffer
    mstar_set_reg_low_byte(0x1604, 0x01);

    // Set e-flash mode trigger(Trigger write mode)
    mstar_set_reg_low_byte(0x1606, 0x81);

    // Set init data
    mstar_set_reg_low_byte(0x1602, pFirstData[0]);
    mstar_set_reg_low_byte(0x1602, pFirstData[1]);
    mstar_set_reg_low_byte(0x1602, pFirstData[2]);
    mstar_set_reg_low_byte(0x1602, pFirstData[3]);

    // Set initial address(for latch SA, CA)
    mstar_set_reg_16bit(0x1600, nWriteAddr);

    // Set initial address(for latch PA)
    mstar_set_reg_16bit(0x1600, nWriteAddr);
}

static void mstar_write_eflash_do_write(u16 nStartAddr, u8 * pBufferData)
{
    u16 nWriteAddr = nStartAddr / 4;

    TS_LOG_INFO("*** %s() nWriteAddr = %d ***\n", __func__, nWriteAddr);

    // Write data
    mstar_set_reg_low_byte(0x1602, pBufferData[0]);
    mstar_set_reg_low_byte(0x1602, pBufferData[1]);
    mstar_set_reg_low_byte(0x1602, pBufferData[2]);
    mstar_set_reg_low_byte(0x1602, pBufferData[3]);

    // Set address
    mstar_set_reg_16bit(0x1600, nWriteAddr);
}

static void mstar_write_eflash_end(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // Do nothing
}

void mstar_read_eflash_start(u16 nStartAddr, EmemType_e eEmemType)
{
    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);

    mstar_set_reg_low_byte(0x1606, 0x02);

    mstar_set_reg_16bit(0x1600, nStartAddr);

    if (eEmemType == EMEM_MAIN) {
        // Set main block
        mstar_set_reg_low_byte(0x1607, 0x00);

        // Set main double buffer
        mstar_set_reg_low_byte(0x1604, 0x01);

        // Set e-flash mode to read mode for main
        mstar_set_reg_16bit(0x1606, 0x0001);
    } else if (eEmemType == EMEM_INFO) {
        // Set info block
        mstar_set_reg_low_byte(0x1607, 0x08);

        // Set info double buffer
        mstar_set_reg_low_byte(0x1604, 0x01);

        // Set e-flash mode to read mode for info
        mstar_set_reg_16bit(0x1606, 0x0801);
    }
}

void mstar_read_eflash_do_read(u16 nReadAddr, u8 * pReadData)
{
    u16 nRegData1 = 0, nRegData2 = 0;

    TS_LOG_INFO("*** %s() nReadAddr = 0x%x ***\n", __func__, nReadAddr);

    // Set read address
    mstar_set_reg_16bit(0x1600, nReadAddr);

    // Read 16+16 bits
    nRegData1 = mstar_get_reg_16bit(0x160A);
    nRegData2 = mstar_get_reg_16bit(0x160C);

    pReadData[0] = nRegData1 & 0xFF;
    pReadData[1] = (nRegData1 >> 8) & 0xFF;
    pReadData[2] = nRegData2 & 0xFF;
    pReadData[3] = (nRegData2 >> 8) & 0xFF;
}

void mstar_read_eflash_end(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // Set read done
    mstar_set_reg_low_byte(0x1606, 0x02);

    // Unset info flag
    mstar_set_reg_low_byte(0x1607, 0x00);

    // Clear address
    mstar_set_reg_16bit(0x1600, 0x0000);
}

static void mstar_get_sft_addr3_value(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);

    // Set e-flash mode to read mode
    mstar_set_reg_low_byte(0x1606, 0x01);
    mstar_set_reg_low_byte(0x1610, 0x01);
    mstar_set_reg_low_byte(0x1607, 0x20);

    // Set read address
    mstar_set_reg_low_byte(0x1600, 0x03);
    mstar_set_reg_low_byte(0x1601, 0x00);

    g_sfr_addr3_byte0_to_1 = mstar_get_reg_16bit(0x160A);
    g_sfr_addr3_byte2_to_3 = mstar_get_reg_16bit(0x160C);

    TS_LOG_INFO("g_sfr_addr3_byte0_to_1 = 0x%4X, g_sfr_addr3_byte2_to_3 = 0x%4X\n", g_sfr_addr3_byte0_to_1,
            g_sfr_addr3_byte2_to_3);
}

static void mstar_unset_protect_bit(void)
{
    u8 nB0, nB1, nB2, nB3;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_get_sft_addr3_value();

    nB0 = g_sfr_addr3_byte0_to_1 & 0xFF;
    nB1 = (g_sfr_addr3_byte0_to_1 & 0xFF00) >> 8;

    nB2 = g_sfr_addr3_byte2_to_3 & 0xFF;
    nB3 = (g_sfr_addr3_byte2_to_3 & 0xFF00) >> 8;

    TS_LOG_INFO("nB0 = 0x%2X, nB1 = 0x%2X, nB2 = 0x%2X, nB3 = 0x%2X\n", nB0, nB1, nB2, nB3);

    nB2 = nB2 & 0xBF;   // 10111111
    nB3 = nB3 & 0xFC;   // 11111100

    TS_LOG_INFO("nB0 = 0x%2X, nB1 = 0x%2X, nB2 = 0x%2X, nB3 = 0x%2X\n", nB0, nB1, nB2, nB3);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);
    mstar_set_reg_low_byte(0x1610, 0x80);
    mstar_set_reg_low_byte(0x1607, 0x10);

    // Trigger SFR write
    mstar_set_reg_low_byte(0x1606, 0x01);

    // Set write data
    mstar_set_reg_low_byte(0x1602, nB0);
    mstar_set_reg_low_byte(0x1602, nB1);
    mstar_set_reg_low_byte(0x1602, nB2);
    mstar_set_reg_low_byte(0x1602, nB3);

    // Set write address
    mstar_set_reg_low_byte(0x1600, 0x03);
    mstar_set_reg_low_byte(0x1601, 0x00);

    // Set TM mode = 0
    mstar_set_reg_low_byte(0x1607, 0x00);

#ifdef CONFIG_ENABLE_HIGH_SPEED_ISP_MODE
    mstar_set_reg_low_byte(0x1606, 0x01);
    mstar_set_reg_low_byte(0x1606, 0x20);
#endif //CONFIG_ENABLE_HIGH_SPEED_ISP_MODE
}

void mstar_set_protect_bit(void)
{
    u8 nB0, nB1, nB2, nB3;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    nB0 = g_sfr_addr3_byte0_to_1 & 0xFF;
    nB1 = (g_sfr_addr3_byte0_to_1 & 0xFF00) >> 8;

    nB2 = g_sfr_addr3_byte2_to_3 & 0xFF;
    nB3 = (g_sfr_addr3_byte2_to_3 & 0xFF00) >> 8;

    TS_LOG_INFO("nB0 = 0x%2X, nB1 = 0x%2X, nB2 = 0x%2X, nB3 = 0x%2X\n", nB0, nB1, nB2, nB3);

    // Disable cpu read flash
    mstar_set_reg_low_byte(0x1608, 0x20);
    mstar_set_reg_low_byte(0x1606, 0x20);
    mstar_set_reg_low_byte(0x1610, 0x80);
    mstar_set_reg_low_byte(0x1607, 0x10);

    // Trigger SFR write
    mstar_set_reg_low_byte(0x1606, 0x01);

    // Set write data
    mstar_set_reg_low_byte(0x1602, nB0);
    mstar_set_reg_low_byte(0x1602, nB1);
    mstar_set_reg_low_byte(0x1602, nB2);
    mstar_set_reg_low_byte(0x1602, nB3);

    // Set write address
    mstar_set_reg_low_byte(0x1600, 0x03);
    mstar_set_reg_low_byte(0x1601, 0x00);
    mstar_set_reg_low_byte(0x1606, 0x02);
}

void mstar_erase_emem(EmemType_e eEmemType)
{
    u32 nInfoAddr = 0x20;
    u32 nTimeOut = 0;
    u8 nRegData = 0;

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    mstar_dev_hw_reset();

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    mstar_dbbus_i2c_response_ack();

    TS_LOG_INFO("Erase start\n");

    mstar_access_eflash_init();

    // Stop mcu
    mstar_set_reg_low_byte(0x0FE6, 0x01);

    // Set PROGRAM erase password
    mstar_set_reg_16bit(0x1618, 0x5AA5);

    mstar_unset_protect_bit();

    if (eEmemType == EMEM_MAIN) // 128KB
    {
        TS_LOG_INFO("Erase main block\n");

        // Set main block
        mstar_set_reg_low_byte(0x1607, 0x00);

        // Set e-flash mode to erase mode
        mstar_set_reg_low_byte(0x1606, 0xC0);

        // Set page erase main
        mstar_set_reg_low_byte(0x1607, 0x03);

        // e-flash mode trigger
        mstar_set_reg_low_byte(0x1606, 0xC1);

        nTimeOut = 0;
        while (1)   // Wait erase done
        {
            nRegData = mstar_get_reg_low_byte(0x160E);
            nRegData = (nRegData & BIT3);

            TS_LOG_INFO("Wait erase done flag nRegData = 0x%x\n", nRegData);

            if (nRegData == BIT3) {
                break;
            }

            mdelay(10);

            if ((nTimeOut++) > 10) {
                TS_LOG_INFO("Erase main block failed. Timeout.\n");

                goto EraseEnd;
            }
        }
    } else if (eEmemType == EMEM_INFO)  // 2KB
    {
        TS_LOG_INFO("Erase info block\n");

        // Set info block
        mstar_set_reg_low_byte(0x1607, 0x08);

        // Set info double buffer
        mstar_set_reg_low_byte(0x1604, 0x01);

        // Set e-flash mode to erase mode
        mstar_set_reg_low_byte(0x1606, 0xC0);

        // Set page erase info
        mstar_set_reg_low_byte(0x1607, 0x09);

        for (nInfoAddr = 0x20; nInfoAddr <= MSG28XX_EMEM_INFO_MAX_ADDR; nInfoAddr += 0x20) {
            TS_LOG_INFO("nInfoAddr = 0x%x\n", nInfoAddr);   // add for debug

            // Set address
            mstar_set_reg_16bit(0x1600, nInfoAddr);

            // e-flash mode trigger
            mstar_set_reg_low_byte(0x1606, 0xC1);

            nTimeOut = 0;
            while (1)   // Wait erase done
            {
                nRegData = mstar_get_reg_low_byte(0x160E);
                nRegData = (nRegData & BIT3);

                TS_LOG_INFO("Wait erase done flag nRegData = 0x%x\n", nRegData);

                if (nRegData == BIT3) {
                    break;
                }

                mdelay(10);

                if ((nTimeOut++) > 10) {
                    TS_LOG_INFO("Erase info block failed. Timeout.\n");

                    // Set main block
                    mstar_set_reg_low_byte(0x1607, 0x00);

                    goto EraseEnd;
                }
            }
        }

        // Set main block
        mstar_set_reg_low_byte(0x1607, 0x00);
    }

EraseEnd:

    mstar_set_protect_bit();

    mstar_set_reg_low_byte(0x1606, 0x00);
    mstar_set_reg_low_byte(0x1607, 0x00);

    // Clear PROGRAM erase password
    mstar_set_reg_16bit(0x1618, 0xA55A);

    TS_LOG_INFO("Erase end\n");

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();
}

static void mstar_program_emem(EmemType_e eEmemType)
{
    u32 i, j;
#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME) || defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
    u32 k;
#endif
    u32 nPageNum = 0, nLength = 0, nIndex = 0, nWordNum = 0;
    u32 nRetryTime = 0;
    u8 nRegData = 0;
    u8 szFirstData[MSG28XX_EMEM_SIZE_BYTES_ONE_WORD] = { 0 };
    u8 szBufferData[MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE] = { 0 };
#ifdef CONFIG_ENABLE_HIGH_SPEED_ISP_MODE
    u8 szWriteData[3 + MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * 2] = { 0 };
#endif //CONFIG_ENABLE_HIGH_SPEED_ISP_MODE

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_ENABLE_DMA_IIC
    DmaReset();
#endif //CONFIG_ENABLE_DMA_IIC
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

    mstar_dev_hw_reset();

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    mstar_dbbus_i2c_response_ack();

    TS_LOG_INFO("Program start\n");

    mstar_access_eflash_init();

    // Stop mcu
    mstar_set_reg_low_byte(0x0FE6, 0x01);

    // Set PROGRAM erase password
    mstar_set_reg_16bit(0x1618, 0x5AA5);

    mstar_unset_protect_bit();

    if (eEmemType == EMEM_MAIN) // Program main block
    {
        TS_LOG_INFO("Program main block\n");

#ifdef CONFIG_ENABLE_HIGH_SPEED_ISP_MODE
        nPageNum = (MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024) / MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE;    // 128*1024/128=1024

        // Set ISP mode
        mstar_set_reg_16bit_on(0x1EBE, BIT15);

        mstar_set_reg_low_byte(0x1604, 0x01);

        mstar_set_reg_16bit(0x161A, nPageNum);
        mstar_set_reg_16bit(0x1600, 0x0000);    // Set initial address
        mstar_set_reg_16bit_on(0x3C00, BIT0);   // Disable INT GPIO mode
        mstar_set_reg_16bit_on(0x1EA0, BIT1);   // Set ISP INT enable
        mstar_set_reg_16bit(0x1E34, 0x0000);    // Set DQMem start address

        mstar_read_dq_mem_start();

        szWriteData[0] = 0x10;
        szWriteData[1] = 0x00;
        szWriteData[2] = 0x00;

        nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * 2; //128*2=256

        for (j = 0; j < nLength; j++) {
            szWriteData[3 + j] = g_fw_data[0][j];
        }
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szWriteData[0], 3 + nLength); // Write the first two pages(page 0 & page 1)

        mstar_read_dq_mem_end();

        mstar_set_reg_16bit_on(0x1EBE, BIT15);  // Set ISP mode
        mstar_set_reg_16bit_on(0x1608, BIT0);   // Set burst mode
        mstar_set_reg_16bit_on(0x161A, BIT13);  // Set ISP trig

        udelay(2000);   // delay about 2ms

        mstar_read_dq_mem_start();

        nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE; //128

        for (i = 2; i < nPageNum; i++) {
            if (i == 2) {
                szWriteData[0] = 0x10;
                szWriteData[1] = 0x00;
                szWriteData[2] = 0x00;

                for (j = 0; j < nLength; j++) {
                    szWriteData[3 + j] =
                        g_fw_data[i / 8][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE *
                                 (i - (8 * (i / 8))) + j];
                }

                mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szWriteData[0], 3 + nLength);
            } else if (i == (nPageNum - 1)) {
                szWriteData[0] = 0x10;
                szWriteData[1] = 0x00;
                szWriteData[2] = 0x80;

                for (j = 0; j < nLength; j++) {
                    szWriteData[3 + j] =
                        g_fw_data[i / 8][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE *
                                 (i - (8 * (i / 8))) + j];
                }

                szWriteData[3 + 128] = 0xFF;
                szWriteData[3 + 129] = 0xFF;
                szWriteData[3 + 130] = 0xFF;
                szWriteData[3 + 131] = 0xFF;

                mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szWriteData[0], 3 + nLength + 4);
            } else {
//                szWriteData[0] = 0x10;
//                szWriteData[1] = 0x00;
                if (szWriteData[2] == 0x00) {
                    szWriteData[2] = 0x80;
                }
                else  // szWriteData[2] == 0x80
                {
                    szWriteData[2] = 0x00;
                }

                for (j = 0; j < nLength; j++) {
                    szWriteData[3 + j] =
                        g_fw_data[i / 8][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE *
                                 (i - (8 * (i / 8))) + j];
                }

                mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szWriteData[0], 3 + nLength);
            }
        }

        mstar_read_dq_mem_end();

#else

#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
        nPageNum = (MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024) / 8;   // 128*1024/8=16384
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
        nPageNum = (MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024) / 32;  // 128*1024/32=4096
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
        nPageNum = (MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024) / MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE;    // 128*1024/128=1024
#endif

        nIndex = 0;

        for (i = 0; i < nPageNum; i++) {
            if (i == 0) {
                // Read first data 4 bytes
                nLength = MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;

                szFirstData[0] = g_fw_data[0][0];
                szFirstData[1] = g_fw_data[0][1];
                szFirstData[2] = g_fw_data[0][2];
                szFirstData[3] = g_fw_data[0][3];

                mstar_isp_burst_write_eflash_start(nIndex, &szFirstData[0],
                                   MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024, nPageNum,
                                   EMEM_MAIN);

                nIndex += nLength;

#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
                nLength = 8 - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD; // 4 = 8 - 4
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
                nLength = 32 - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;    // 28 = 32 - 4
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
                nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;  // 124 = 128 - 4
#endif

                for (j = 0; j < nLength; j++) {
                    szBufferData[j] = g_fw_data[0][4 + j];
                }
            } else {
#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
                nLength = 8;
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
                nLength = 32;
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
                nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE; // 128
#endif

                for (j = 0; j < nLength; j++) {
#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
                    szBufferData[j] = g_fw_data[i / 128][8 * (i - (128 * (i / 128))) + j];
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
                    szBufferData[j] = g_fw_data[i / 32][32 * (i - (32 * (i / 32))) + j];
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
                    szBufferData[j] =
                        g_fw_data[i / 8][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE *
                                 (i - (8 * (i / 8))) + j];
#endif
                }
            }

            mstar_isp_burst_write_eflash_do_write(&szBufferData[0], nLength);

            udelay(2000);   // delay about 2ms

            nIndex += nLength;
        }

        mstar_isp_burst_write_eflash_end();

        // Set write done
        mstar_set_reg_16bit_on(0x1606, BIT2);

        // Check RBB
        nRegData = mstar_get_reg_low_byte(0x160E);
        nRetryTime = 0;

        while ((nRegData & BIT3) != BIT3) {
            mdelay(10);

            nRegData = mstar_get_reg_low_byte(0x160E);

            if (nRetryTime++ > 100) {
                TS_LOG_INFO("main block can't wait write to done.\n");

                goto ProgramEnd;
            }
        }
#endif //CONFIG_ENABLE_HIGH_SPEED_ISP_MODE
    } else if (eEmemType == EMEM_INFO)  // Program info block
    {
        TS_LOG_INFO("Program info block\n");

        nPageNum = (MSG28XX_FIRMWARE_INFO_BLOCK_SIZE * 1024) / MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE;    // 2*1024/128=16

        nIndex = 0;
        nIndex += MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE; //128

        // Skip firt page(page 0) & Update page 1~14 by isp burst write mode
        for (i = 1; i < (nPageNum - 1); i++)    // Skip the first 128 byte and the last 128 byte of info block
        {
            if (i == 1) {
                // Read first data 4 bytes
                nLength = MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;

                szFirstData[0] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE];
                szFirstData[1] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE +
                                        1];
                szFirstData[2] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE +
                                        2];
                szFirstData[3] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE +
                                        3];

                mstar_isp_burst_write_eflash_start(nIndex, &szFirstData[0],
                                   MSG28XX_FIRMWARE_INFO_BLOCK_SIZE * 1024,
                                   nPageNum - 1, EMEM_INFO);

                nIndex += nLength;

                nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;  // 124 = 128 - 4

#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
                for (j = 0; j < (nLength / 8); j++) // 124/8 = 15
                {
                    for (k = 0; k < 8; k++) {
                        szBufferData[k] =
                            g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                            [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 4 + (8 * j) + k];
                    }

                    mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 8);

                    udelay(2000);   // delay about 2ms
                }

                for (k = 0; k < (nLength % 8); k++) // 124%8 = 4
                {
                    szBufferData[k] =
                        g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                        [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 4 + (8 * j) + k];
                }

                mstar_isp_burst_write_eflash_do_write(&szBufferData[0], (nLength % 8)); // 124%8 = 4

                udelay(2000);   // delay about 2ms
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
                for (j = 0; j < (nLength / 32); j++)    // 124/8 = 3
                {
                    for (k = 0; k < 32; k++) {
                        szBufferData[k] =
                            g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                            [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 4 + (32 * j) + k];
                    }

                    mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 32);

                    udelay(2000);   // delay about 2ms
                }

                for (k = 0; k < (nLength % 32); k++)    // 124%32 = 28
                {
                    szBufferData[k] =
                        g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                        [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 4 + (32 * j) + k];
                }

                mstar_isp_burst_write_eflash_do_write(&szBufferData[0], (nLength % 32));    // 124%8 = 28

                udelay(2000);   // delay about 2ms
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
                for (j = 0; j < nLength; j++) {
                    szBufferData[j] =
                        g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                        [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 4 + j];
                }
#endif
            } else {
                nLength = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE; //128

#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME)
                if (i < 8)  // 1 < i < 8
                {
                    for (j = 0; j < (nLength / 8); j++) // 128/8 = 16
                    {
                        for (k = 0; k < 8; k++) {
                            szBufferData[k] =
                                g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                                [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * i + (8 * j) + k];
                        }

                        mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 8);

                        udelay(2000);   // delay about 2ms
                    }
                } else  // i >= 8
                {
                    for (j = 0; j < (nLength / 8); j++) // 128/8 = 16
                    {
                        for (k = 0; k < 8; k++) {
                            szBufferData[k] =
                                g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                                      1][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * (i -
                                                         8) +
                                     (8 * j) + k];
                        }

                        mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 8);

                        udelay(2000);   // delay about 2ms
                    }
                }
#elif defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
                if (i < 8)  // 1 < i < 8
                {
                    for (j = 0; j < (nLength / 32); j++)    // 128/32 = 4
                    {
                        for (k = 0; k < 32; k++) {
                            szBufferData[k] =
                                g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                                [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * i + (32 * j) + k];
                        }

                        mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 32);

                        udelay(2000);   // delay about 2ms
                    }
                } else  // i >= 8
                {
                    for (j = 0; j < (nLength / 32); j++)    // 128/32 = 4
                    {
                        for (k = 0; k < 32; k++) {
                            szBufferData[k] =
                                g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                                      1][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * (i -
                                                         8) +
                                     (32 * j) + k];
                        }

                        mstar_isp_burst_write_eflash_do_write(&szBufferData[0], 32);

                        udelay(2000);   // delay about 2ms
                    }
                }
#else // UPDATE FIRMWARE WITH 128 BYTE EACH TIME
                if (i < 8)  // 1 < i < 8
                {
                    for (j = 0; j < nLength; j++) {
                        szBufferData[j] =
                            g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE]
                            [MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * i + j];
                    }
                } else  // i >= 8
                {
                    for (j = 0; j < nLength; j++) {
                        szBufferData[j] =
                            g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                                  1][MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE * (i - 8) + j];
                    }
                }
#endif
            }

#if defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_8_BYTE_EACH_TIME) || defined(CONFIG_ENABLE_UPDATE_FIRMWARE_WITH_32_BYTE_EACH_TIME)
            // Do nothing here
#else
            mstar_isp_burst_write_eflash_do_write(&szBufferData[0], nLength);

            udelay(2000);   // delay about 2ms
#endif
            nIndex += nLength;
        }

        mstar_isp_burst_write_eflash_end();

        // Set write done
        mstar_set_reg_16bit_on(0x1606, BIT2);

        // Check RBB
        nRegData = mstar_get_reg_low_byte(0x160E);
        nRetryTime = 0;

        while ((nRegData & BIT3) != BIT3) {
            mdelay(10);

            nRegData = mstar_get_reg_low_byte(0x160E);

            if (nRetryTime++ > 100) {
                TS_LOG_INFO("Info block page 1~14 can't wait write to done.\n");

                goto ProgramEnd;
            }
        }

        mstar_set_reg_16bit_off(0x1EBE, BIT15);

        // Update page 15 by write mode
        nIndex = 15 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE;
        nWordNum = MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE / MSG28XX_EMEM_SIZE_BYTES_ONE_WORD; // 128/4=32
        nLength = MSG28XX_EMEM_SIZE_BYTES_ONE_WORD;

        for (i = 0; i < nWordNum; i++) {
            if (i == 0) {
                szFirstData[0] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                          1][7 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE];
                szFirstData[1] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                          1][7 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 1];
                szFirstData[2] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                          1][7 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 2];
                szFirstData[3] =
                    g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                          1][7 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + 3];

                mstar_write_eflash_start(nIndex, &szFirstData[0], EMEM_INFO);
            } else {
                for (j = 0; j < nLength; j++) {
                    szFirstData[j] =
                        g_fw_data[MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE +
                              1][7 * MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE + (4 * i) + j];
                }

                mstar_write_eflash_do_write(nIndex, &szFirstData[0]);
            }

            udelay(2000);   // delay about 2ms

            nIndex += nLength;
        }

        mstar_write_eflash_end();

        // Set write done
        mstar_set_reg_16bit_on(0x1606, BIT2);

        // Check RBB
        nRegData = mstar_get_reg_low_byte(0x160E);
        nRetryTime = 0;

        while ((nRegData & BIT3) != BIT3) {
            mdelay(10);

            nRegData = mstar_get_reg_low_byte(0x160E);

            if (nRetryTime++ > 100) {
                TS_LOG_INFO("Info block page 15 can't wait write to done.\n");

                goto ProgramEnd;
            }
        }
    }

ProgramEnd:

    mstar_set_protect_bit();

    // Clear PROGRAM erase password
    mstar_set_reg_16bit(0x1618, 0xA55A);

    TS_LOG_INFO("Program end\n");

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();
}

u16 mstar_get_swid(EmemType_e eEmemType)
{
    u16 nRetVal = 0;
    u16 nReadAddr = 0;
    u8 szTmpData[4] = { 0 };

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    // Stop MCU
    mstar_set_reg_low_byte(0x0FE6, 0x01);

    if (eEmemType == EMEM_MAIN) // Read SW ID from main block
    {
        mstar_read_eflash_start(0x7FFD, EMEM_MAIN);
        nReadAddr = 0x7FFD;
    } else if (eEmemType == EMEM_INFO)  // Read SW ID from info block
    {
        mstar_read_eflash_start(0x81FB, EMEM_INFO);
        nReadAddr = 0x81FB;
    }

    mstar_read_eflash_do_read(nReadAddr, &szTmpData[0]);

    mstar_read_eflash_end();

    /*
       Ex. SW ID in Main Block :
       Major low byte at address 0x7FFD

       SW ID in Info Block :
       Major low byte at address 0x81FB
     */

    nRetVal = (szTmpData[1] << 8);
    nRetVal |= szTmpData[0];

    TS_LOG_INFO("SW ID = 0x%x\n", nRetVal);

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    return nRetVal;
}

static u32 mstar_get_fw_crc_hw(EmemType_e eEmemType)
{
    u32 nRetVal = 0;
    u32 nRetryTime = 0;
    u32 nCrcEndAddr = 0;
    u16 nCrcDown = 0;

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    mstar_dev_hw_reset();

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    mstar_access_eflash_init();

    if (eEmemType == EMEM_MAIN) {
        // Disable cpu read flash
        mstar_set_reg_low_byte(0x1608, 0x20);
        mstar_set_reg_low_byte(0x1606, 0x20);

        // Set read flag
        mstar_set_reg_16bit(0x1610, 0x0001);

        // Mode reset main block
        mstar_set_reg_16bit(0x1606, 0x0000);

        // CRC reset
        mstar_set_reg_16bit(0x1620, 0x0002);

        mstar_set_reg_16bit(0x1620, 0x0000);

        // Set CRC e-flash block start address => Main Block : 0x0000 ~ 0x7FFE
        mstar_set_reg_16bit(0x1600, 0x0000);

        nCrcEndAddr = (MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024) / 4 - 2;

        mstar_set_reg_16bit(0x1622, nCrcEndAddr);

        // Trigger CRC check
        mstar_set_reg_16bit(0x1620, 0x0001);

        nCrcDown = mstar_get_reg_16bit(0x1620);

        nRetryTime = 0;
        while ((nCrcDown >> 15) == 0) {
            mdelay(10);

            nCrcDown = mstar_get_reg_16bit(0x1620);
            nRetryTime++;

            if (nRetryTime > 30) {
                TS_LOG_INFO("Wait main block nCrcDown failed.\n");
                break;
            }
        }

        nRetVal = mstar_get_reg_16bit(0x1626);
        nRetVal = (nRetVal << 16) | mstar_get_reg_16bit(0x1624);
    } else if (eEmemType == EMEM_INFO) {
        // Disable cpu read flash
        mstar_set_reg_low_byte(0x1608, 0x20);
        mstar_set_reg_low_byte(0x1606, 0x20);

        // Set read flag
        mstar_set_reg_16bit(0x1610, 0x0001);

        // Mode reset info block
        mstar_set_reg_16bit(0x1606, 0x0800);

        mstar_set_reg_low_byte(0x1604, 0x01);

        // CRC reset
        mstar_set_reg_16bit(0x1620, 0x0002);

        mstar_set_reg_16bit(0x1620, 0x0000);

        // Set CRC e-flash block start address => Info Block : 0x0020 ~ 0x01FE
        mstar_set_reg_16bit(0x1600, 0x0020);
        mstar_set_reg_16bit(0x1622, 0x01FE);

        // Trigger CRC check
        mstar_set_reg_16bit(0x1620, 0x0001);

        nCrcDown = mstar_get_reg_16bit(0x1620);

        nRetryTime = 0;
        while ((nCrcDown >> 15) == 0) {
            mdelay(10);

            nCrcDown = mstar_get_reg_16bit(0x1620);
            nRetryTime++;

            if (nRetryTime > 30) {
                TS_LOG_INFO("Wait info block nCrcDown failed.\n");
                break;
            }
        }

        nRetVal = mstar_get_reg_16bit(0x1626);
        nRetVal = (nRetVal << 16) | mstar_get_reg_16bit(0x1624);
    }

    TS_LOG_INFO("Hardware CRC = 0x%x\n", nRetVal);

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    return nRetVal;
}

static u32 mstar_get_fw_crc_eflash(EmemType_e eEmemType)
{
    u32 nRetVal = 0;
    u16 nReadAddr = 0;
    u8 szTmpData[4] = { 0 };

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();

    // Stop MCU
    mstar_set_reg_low_byte(0x0FE6, 0x01);

    if (eEmemType == EMEM_MAIN) // Read main block CRC(128KB-4) from main block
    {
        mstar_read_eflash_start(0x7FFF, EMEM_MAIN);
        nReadAddr = 0x7FFF;
    } else if (eEmemType == EMEM_INFO)  // Read info block CRC(2KB-MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE-4) from info block
    {
        mstar_read_eflash_start(0x81FF, EMEM_INFO);
        nReadAddr = 0x81FF;
    }

    mstar_read_eflash_do_read(nReadAddr, &szTmpData[0]);

    TS_LOG_INFO("szTmpData[0] = 0x%x\n", szTmpData[0]); // add for debug
    TS_LOG_INFO("szTmpData[1] = 0x%x\n", szTmpData[1]); // add for debug
    TS_LOG_INFO("szTmpData[2] = 0x%x\n", szTmpData[2]); // add for debug
    TS_LOG_INFO("szTmpData[3] = 0x%x\n", szTmpData[3]); // add for debug

    mstar_read_eflash_end();

    nRetVal = (szTmpData[3] << 24);
    nRetVal |= (szTmpData[2] << 16);
    nRetVal |= (szTmpData[1] << 8);
    nRetVal |= szTmpData[0];

    TS_LOG_INFO("CRC = 0x%x\n", nRetVal);

    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();

    return nRetVal;
}

static u32 mstar_get_fw_crc_bin(u8 ** szTmpBuf, EmemType_e eEmemType)
{
    u32 nRetVal = 0;

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    if (szTmpBuf != NULL) {
        if (eEmemType == EMEM_MAIN) {
            nRetVal = szTmpBuf[127][1023];
            nRetVal = (nRetVal << 8) | szTmpBuf[127][1022];
            nRetVal = (nRetVal << 8) | szTmpBuf[127][1021];
            nRetVal = (nRetVal << 8) | szTmpBuf[127][1020];
        } else if (eEmemType == EMEM_INFO) {
            nRetVal = szTmpBuf[129][1023];
            nRetVal = (nRetVal << 8) | szTmpBuf[129][1022];
            nRetVal = (nRetVal << 8) | szTmpBuf[129][1021];
            nRetVal = (nRetVal << 8) | szTmpBuf[129][1020];
        }
    }

    return nRetVal;
}

static s32 mstar_check_fw_bin_ingerity(u8 ** szFwData)
{
    u32 nCrcMain = 0, nCrcMainBin = 0;
    u32 nCrcInfo = 0, nCrcInfoBin = 0;
    u32 nRetVal = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_convert_fw_two_dimen_to_one(szFwData, g_one_dimen_fw_data);

    /* Calculate main block CRC & info block CRC by device driver itself */
    nCrcMain =
        mstar_calculate_crc(g_one_dimen_fw_data, 0,
                MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);
    nCrcInfo =
        mstar_calculate_crc(g_one_dimen_fw_data,
                MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 + MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE,
                MSG28XX_FIRMWARE_INFO_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE -
                MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);

    /* Read main block CRC & info block CRC from firmware bin file */
    nCrcMainBin = mstar_get_fw_crc_bin(szFwData, EMEM_MAIN);
    nCrcInfoBin = mstar_get_fw_crc_bin(szFwData, EMEM_INFO);

    TS_LOG_INFO("nCrcMain=0x%x, nCrcInfo=0x%x, nCrcMainBin=0x%x, nCrcInfoBin=0x%x\n",
            nCrcMain, nCrcInfo, nCrcMainBin, nCrcInfoBin);

    if ((nCrcMainBin != nCrcMain) || (nCrcInfoBin != nCrcInfo)) {
        TS_LOG_INFO("CHECK FIRMWARE BIN FILE INTEGRITY FAILED. CANCEL UPDATE FIRMWARE.\n");
        nRetVal = -1;
    } else {
        TS_LOG_INFO("CHECK FIRMWARE BIN FILE INTEGRITY SUCCESS. PROCEED UPDATE FIRMWARE.\n");
        nRetVal = 0;
    }

    return nRetVal;
}

static u32 mstar_28xx_fw_update(u8 ** szFwData, EmemType_e eEmemType)
{
    u32 nCrcMain = 0, nCrcMainHardware = 0, nCrcMainEflash = 0;
    u32 nCrcInfo = 0, nCrcInfoHardware = 0, nCrcInfoEflash = 0;

    TS_LOG_INFO("*** %s() eEmemType = %d ***\n", __func__, eEmemType);

    if (mstar_check_fw_bin_ingerity(szFwData) < 0) {
        TS_LOG_ERR("CHECK FIRMWARE BIN FILE INTEGRITY FAILED. CANCEL UPDATE FIRMWARE.\n");

        g_fw_data_cont = 0; // reset g_fw_data_cont to 0

        mstar_dev_hw_reset();

        return -1;
    }

    g_fw_update = 0x01; // Set flag to 0x01 for indicating update firmware is processing

    // Erase
    if (eEmemType == EMEM_ALL) {
        mstar_erase_emem(EMEM_MAIN);
        mstar_erase_emem(EMEM_INFO);
    } else if (eEmemType == EMEM_MAIN) {
        mstar_erase_emem(EMEM_MAIN);
    } else if (eEmemType == EMEM_INFO) {
        mstar_erase_emem(EMEM_INFO);
    }

    TS_LOG_INFO("erase OK\n");

    // Program
    if (eEmemType == EMEM_ALL) {
        mstar_program_emem(EMEM_MAIN);
        mstar_program_emem(EMEM_INFO);
    } else if (eEmemType == EMEM_MAIN) {
        mstar_program_emem(EMEM_MAIN);
    } else if (eEmemType == EMEM_INFO) {
        mstar_program_emem(EMEM_INFO);
    }

    TS_LOG_INFO("program OK\n");

    /* Calculate main block CRC & info block CRC by device driver itself */
    mstar_convert_fw_two_dimen_to_one(szFwData, g_one_dimen_fw_data);

    /* Read main block CRC & info block CRC from TP */
    if (eEmemType == EMEM_ALL) {
        nCrcMain =
            mstar_calculate_crc(g_one_dimen_fw_data, 0,
                    MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);
        nCrcInfo =
            mstar_calculate_crc(g_one_dimen_fw_data,
                    MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 + MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE,
                    MSG28XX_FIRMWARE_INFO_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE -
                    MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);

        nCrcMainHardware = mstar_get_fw_crc_hw(EMEM_MAIN);
        nCrcInfoHardware = mstar_get_fw_crc_hw(EMEM_INFO);

        nCrcMainEflash = mstar_get_fw_crc_eflash(EMEM_MAIN);
        nCrcInfoEflash = mstar_get_fw_crc_eflash(EMEM_INFO);
    } else if (eEmemType == EMEM_MAIN) {
        nCrcMain =
            mstar_calculate_crc(g_one_dimen_fw_data, 0,
                    MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);
        nCrcMainHardware = mstar_get_fw_crc_hw(EMEM_MAIN);
        nCrcMainEflash = mstar_get_fw_crc_eflash(EMEM_MAIN);
    } else if (eEmemType == EMEM_INFO) {
        nCrcInfo =
            mstar_calculate_crc(g_one_dimen_fw_data,
                    MSG28XX_FIRMWARE_MAIN_BLOCK_SIZE * 1024 + MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE,
                    MSG28XX_FIRMWARE_INFO_BLOCK_SIZE * 1024 - MSG28XX_EMEM_SIZE_BYTES_PER_ONE_PAGE -
                    MSG28XX_EMEM_SIZE_BYTES_ONE_WORD);
        nCrcInfoHardware = mstar_get_fw_crc_hw(EMEM_INFO);
        nCrcInfoEflash = mstar_get_fw_crc_eflash(EMEM_INFO);
    }

    TS_LOG_INFO
        ("nCrcMain=0x%x, nCrcInfo=0x%x, nCrcMainHardware=0x%x, nCrcInfoHardware=0x%x, nCrcMainEflash=0x%x, nCrcInfoEflash=0x%x\n",
         nCrcMain, nCrcInfo, nCrcMainHardware, nCrcInfoHardware, nCrcMainEflash, nCrcInfoEflash);

    g_fw_data_cont = 0; // Reset g_fw_data_cont to 0 after update firmware

    mstar_dev_hw_reset();
    mdelay(300);

    g_fw_update = 0x00; // Set flag to 0x00 for indicating update firmware is finished

    if (eEmemType == EMEM_ALL) {
        if ((nCrcMainHardware != nCrcMain) || (nCrcInfoHardware != nCrcInfo) || (nCrcMainEflash != nCrcMain)
            || (nCrcInfoEflash != nCrcInfo)) {
            TS_LOG_INFO("Update FAILED\n");

            return -1;
        }
    } else if (eEmemType == EMEM_MAIN) {
        if ((nCrcMainHardware != nCrcMain) || (nCrcMainEflash != nCrcMain)) {
            TS_LOG_INFO("Update FAILED\n");

            return -1;
        }
    } else if (eEmemType == EMEM_INFO) {
        if ((nCrcInfoHardware != nCrcInfo) || (nCrcInfoEflash != nCrcInfo)) {
            TS_LOG_INFO("Update FAILED\n");

            return -1;
        }
    }

    TS_LOG_INFO("Update SUCCESS\n");

    return 0;
}

u32 mstar_update_fw_cash(u8 ** szFwData, EmemType_e eEmemType)
{
    u16 ic_sw_id = 0x0000;
    u16 fw_sw_id = 0x0000;

    TS_LOG_INFO("%s:g_chip_type = 0x%x\n", __func__, g_chip_type);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2118A)
    {
        TS_LOG_INFO("IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED = %d\n", IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED);

        // force to update firmware, don't check sw id
        if (IS_FORCE_TO_UPDATE_FIRMWARE_ENABLED) {
            return mstar_28xx_fw_update(szFwData, EMEM_MAIN);
        } else {
            fw_sw_id = szFwData[129][1005] << 8 | szFwData[129][1004];
            ic_sw_id = mstar_get_swid(EMEM_INFO);
            TS_LOG_INFO("Firmware sw id = 0x%x, IC sw id = 0x%x\n", fw_sw_id, ic_sw_id);

            if ((ic_sw_id != fw_sw_id) && (ic_sw_id != 0xFFFF)) {
                mstar_dev_hw_reset();   // Reset HW here to avoid touch may be not worked after get sw id.
                TS_LOG_INFO("The sw id of the update firmware file is not equal to sw id on e-flash. Not allow to update\n");
                g_fw_data_cont = 0;     // Reset g_fw_data_cont to 0
                return -1;
            } else {
                return mstar_28xx_fw_update(szFwData, EMEM_MAIN);
            }
        }
    } else {
        TS_LOG_INFO("Unsupported chip type = 0x%x\n", g_chip_type);
        g_fw_data_cont = 0; // Reset g_fw_data_cont to 0
        return -1;
    }
}

s32 mstar_fw_update_sdcard(const char *pFilePath, u8 mode)
{
    s32 ret = -1, i = 0;
    struct file *pfile = NULL;
    struct inode *inode;
    s32 fsize = 0;
    mm_segment_t old_fs;
    loff_t pos;
    u16 eSwId = 0x0000;
    u16 eVendorId = 0x0000;
    const struct firmware *fw = NULL;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    g_fw_data_buf = vmalloc(MSG28XX_FIRMWARE_WHOLE_SIZE * 1024 * sizeof(u8));
    g_one_dimen_fw_data = vmalloc(MSG28XX_FIRMWARE_WHOLE_SIZE * 1024 * sizeof(u8));

    if (ERR_ALLOC_MEM(g_fw_data_buf) || ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        TS_LOG_ERR("Failed to allocate FW buffer \n");
        goto out;
    }

    g_fw_data = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
    if (ERR_ALLOC_MEM(g_fw_data)) {
        TS_LOG_ERR("Failed to allocate FW buffer\n");
        goto out;
    }

    for (i = 0; i < 130; i++) {
        g_fw_data[i] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);

        if (ERR_ALLOC_MEM(g_fw_data[i])) {
            TS_LOG_ERR("Failed to allocate FW buffer\n");
            goto out;
        }
    }

    if (mode == 1) {
        pfile = filp_open(pFilePath, O_RDONLY, 0);
        if (IS_ERR(pfile)) {
            TS_LOG_INFO("Error occurred while opening file %s.\n", pFilePath);
            goto out;
        }
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 0)
        inode = pfile->f_dentry->d_inode;
#else
        inode = pfile->f_path.dentry->d_inode;
#endif

        fsize = inode->i_size;

        TS_LOG_INFO("fsize = %d\n", fsize);

        if (fsize <= 0) {
            filp_close(pfile, NULL);
            goto out;
        }
        // read firmware
        memset(g_fw_data_buf, 0, MSG28XX_FIRMWARE_WHOLE_SIZE * 1024);

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        pos = 0;
        vfs_read(pfile, g_fw_data_buf, fsize, &pos);

        filp_close(pfile, NULL);
        set_fs(old_fs);
    } else {
        ret = request_firmware(&fw, pFilePath, &g_mstar_dev_data->ts_platform_data->ts_dev->dev);
        if (ret) {
            TS_LOG_INFO("[MSTAR] failed to request firmware %d\n", ret);
            goto out;
        }
        if ((int)fw->size < MSG28XX_FIRMWARE_WHOLE_SIZE * 1024) {
            TS_LOG_ERR("request firmware size is less than 130KB\n");
            goto out;
        }
        memcpy(g_fw_data_buf, fw->data, MSG28XX_FIRMWARE_WHOLE_SIZE * 1024);
        fsize = MSG28XX_FIRMWARE_WHOLE_SIZE * 1024;
    }

    mstar_fw_store_data(g_fw_data_buf, fsize);

    mstar_finger_touch_report_disable();

    if ((g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA
         || g_chip_type == CHIP_TYPE_ILI2117A || g_chip_type == CHIP_TYPE_ILI2118A)
        && fsize == 133120) {
        ret = mstar_update_fw_cash(g_fw_data, EMEM_MAIN);   // update main block only, do not update info block.
    } else {
        mstar_dev_hw_reset();
        TS_LOG_INFO("The file size of the update firmware bin file is invalid, fsize = %d\n", fsize);
    }

out:
    g_fw_data_cont = 0; // reset g_fw_data_cont to 0 after update firmware

    mstar_finger_touch_report_enable();

    if (!ERR_ALLOC_MEM(fw)) {
        release_firmware(fw);
        fw = NULL;
    }

    if (!ERR_ALLOC_MEM(g_fw_data_buf)) {
        vfree(g_fw_data_buf);
        g_fw_data_buf = NULL;
    }

    if (!ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        vfree(g_one_dimen_fw_data);
        g_one_dimen_fw_data = NULL;
    }

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

    return ret;
}

static void mstar_read_dq_mem_start(void)
{
    u8 nParCmdSelUseCfg = 0x7F;
    u8 nParCmdAdByteEn0 = 0x50;
    u8 nParCmdAdByteEn1 = 0x51;
    u8 nParCmdDaByteEn0 = 0x54;
    u8 nParCmdUSetSelB0 = 0x80;
    u8 nParCmdUSetSelB1 = 0x82;
    u8 nParCmdSetSelB2 = 0x85;
    u8 nParCmdIicUse = 0x35;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdSelUseCfg, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn0, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn1, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdDaByteEn0, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB0, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB1, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdSetSelB2, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdIicUse, 1);
}

static void mstar_read_dq_mem_end(void)
{
    u8 nParCmdNSelUseCfg = 0x7E;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdNSelUseCfg, 1);
}

u32 mstar_read_dq_mem_value(u16 nAddr)
{
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2118A) {
        u8 tx_data[3] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF };
        u8 rx_data[4] = { 0 };

        TS_LOG_INFO("*** %s() ***\n", __func__);

        TS_LOG_INFO("DQMem Addr = 0x%x\n", nAddr);

        mstar_dbbus_enter_serial_debug();
        mstar_dbbus_stop_mcu();
        mstar_dbbus_iic_use_bus();
        mstar_dbbus_iic_reshape();

        // Stop mcu
        mstar_set_reg_low_byte(0x0FE6, 0x01);   //bank:mheg5, addr:h0073
        mdelay(100);

        mstar_read_dq_mem_start();

        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
        mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, &rx_data[0], 4);

        mstar_read_dq_mem_end();

        // Start mcu
        mstar_set_reg_low_byte(0x0FE6, 0x00);   //bank:mheg5, addr:h0073

        mstar_dbbus_iic_not_use_bus();
        mstar_dbbus_not_stop_mcu();
        mstar_dbbus_exit_serial_debug();

        return (rx_data[3] << 24 | rx_data[2] << 16 | rx_data[1] << 8 | rx_data[0]);
    } else {
        TS_LOG_INFO("*** %s() ***\n", __func__);

        // TODO : not support yet

        return 0;
    }
}

void mstar_write_dq_mem_value(u16 nAddr, u32 nData)
{
    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2118A) {
        u8 szDbBusTxData[7] = { 0 };

        TS_LOG_INFO("*** %s() ***\n", __func__);

        TS_LOG_INFO("DQMem Addr = 0x%x\n", nAddr);

        mstar_dbbus_enter_serial_debug();
        mstar_dbbus_stop_mcu();
        mstar_dbbus_iic_use_bus();
        mstar_dbbus_iic_reshape();

        // Stop mcu
        mstar_set_reg_low_byte(0x0FE6, 0x01);   //bank:mheg5, addr:h0073
        mdelay(100);

        mstar_read_dq_mem_start();

        szDbBusTxData[0] = 0x10;
        szDbBusTxData[1] = ((nAddr >> 8) & 0xff);
        szDbBusTxData[2] = (nAddr & 0xff);
        szDbBusTxData[3] = nData & 0x000000FF;
        szDbBusTxData[4] = ((nData & 0x0000FF00) >> 8);
        szDbBusTxData[5] = ((nData & 0x00FF0000) >> 16);
        szDbBusTxData[6] = ((nData & 0xFF000000) >> 24);
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &szDbBusTxData[0], 7);

        mstar_read_dq_mem_end();

        // Start mcu
        mstar_set_reg_low_byte(0x0FE6, 0x00);   //bank:mheg5, addr:h0073
        mdelay(100);

        mstar_dbbus_iic_not_use_bus();
        mstar_dbbus_not_stop_mcu();
        mstar_dbbus_exit_serial_debug();
    } else {
        TS_LOG_INFO("*** %s() ***\n", __func__);

        // TODO : not support yet
    }
}

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
int mstar_fw_update_swid_entry(void)
{
    int ret = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2118A) {
#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
        mstar_fw_update_swid();
#endif
    }

    return ret;
}

#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
static void mstar_fw_update_swid(void)
{
    u32 nCrcMainA, nCrcMainB;
    u32 i, j, ret = 0;
    u16 nUpdateBinMajor = 0, nUpdateBinMinor = 0;
    u16 nMajor = 0, nMinor = 0;
    u8 nIsSwIdValid = 0;
    u8 *pVersion = NULL;
    char fw_name[128] = { 0 };
    int copy_count = 0;
    int k = 0, l = 0;
    const struct firmware *fw = NULL;
    u16 ic_sw_id = 0;
    u16 fw_sw_id = 0;

    mstar_finger_touch_report_disable();
    nCrcMainA = mstar_get_fw_crc_hw(EMEM_MAIN);
    nCrcMainB = mstar_get_fw_crc_eflash(EMEM_MAIN);
    if (nCrcMainA == nCrcMainB) {
        ic_sw_id = mstar_get_swid(EMEM_MAIN);
    }
    else {
        ic_sw_id = mstar_get_swid(EMEM_INFO);
    }
    TS_LOG_INFO("ic swid = 0x%x\n", ic_sw_id);
    mstar_finger_touch_report_enable();

    if ((!strcmp(mstar_project_id, MSTAR_PROJECT_ID_HLT)) || (ic_sw_id == MSTAR_VENDOR_ID_HLT)) {
        sprintf(fw_name, "%s", MSTAR_FIRMWARE_NAME_HLT);
        TS_LOG_INFO("projectid: %s, vendorid: %d\n", mstar_project_id, ic_sw_id);
    }
    else {
        TS_LOG_ERR("unknow project id and SW ID\n");
        return;
    }

    ret = request_firmware(&fw, fw_name, &g_mstar_dev_data->ts_platform_data->ts_dev->dev);
    if (ret) {
        TS_LOG_ERR("[MSTAR] failed to request firmware %d\n", ret);
        return;
    }

    TS_LOG_INFO("mstar fw->size = %d\n", (int)fw->size);

    if ((int)fw->size < (130 * 1024)) {
        TS_LOG_ERR("The size of firmware is too smaller\n");
        return;
    }

    g_one_dimen_fw_data = vmalloc(MSG28XX_FIRMWARE_WHOLE_SIZE * 1024 * sizeof(u8));
    if (ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        TS_LOG_ERR("Failed to allocate FW buffer\n");
        goto out;
    }

    mstar_finger_touch_report_disable();
    nCrcMainA = mstar_get_fw_crc_hw(EMEM_MAIN);
    nCrcMainB = mstar_get_fw_crc_eflash(EMEM_MAIN);
    TS_LOG_INFO("main_hw_crc = 0x%x, main_eflash_crc = 0x%x\n", nCrcMainA, nCrcMainB);

#ifdef CONFIG_ENABLE_CODE_FOR_DEBUG
    if (nCrcMainA != nCrcMainB) {
        for (i = 0; i < 5; i++) {
            nCrcMainA = mstar_get_fw_crc_hw(EMEM_MAIN);
            nCrcMainB = mstar_get_fw_crc_eflash(EMEM_MAIN);
            TS_LOG_INFO("*** Retry[%d] : main_hw_crc=0x%x, main_eflash_crc=0x%x ***\n", i, nCrcMainA, nCrcMainB);

            if (nCrcMainA == nCrcMainB) {
                break;
            }
            mdelay(50);
        }
    }
#endif

    if (nCrcMainA == nCrcMainB) {

        ic_sw_id = mstar_get_swid(EMEM_MAIN);
        fw_sw_id = fw->data[0x1FFF5] << 8 | fw->data[0x1FFF4];
        TS_LOG_INFO("Main mem ic swid = 0x%x, firmware file swid = 0x%x\n", ic_sw_id, fw_sw_id);

        if (ic_sw_id == fw_sw_id) {

            g_swid_data.pUpdateBin = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
            if (ERR_ALLOC_MEM(g_swid_data.pUpdateBin)) {
                TS_LOG_ERR("Failed to allocate FW buffer\n");
                goto out;
            }

            for (i = 0; i < 130; i++) {
                g_swid_data.pUpdateBin[i] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);

                if (ERR_ALLOC_MEM(g_swid_data.pUpdateBin[i])) {
                    TS_LOG_ERR("Failed to allocate FW buffer\n");
                    goto out;
                }
            }

            for (k = 0; k < 130; k++) {
                for (l = 0; l < 1024; l++) {
                    g_swid_data.pUpdateBin[k][l] = fw->data[copy_count];
                    copy_count++;
                }
            }

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
            nUpdateBinMajor =
                g_swid_data.pUpdateBin[127][1013] << 8 | g_swid_data.pUpdateBin[127][1012];
            nUpdateBinMinor =
                g_swid_data.pUpdateBin[127][1015] << 8 | g_swid_data.pUpdateBin[127][1014];
#else
            nUpdateBinMajor =
                (*(g_swid_data.pUpdateBin + 0x1FFF5)) << 8 |
                (*(g_swid_data.pUpdateBin + 0x1FFF4));
            nUpdateBinMinor =
                (*(g_swid_data.pUpdateBin + 0x1FFF7)) << 8 |
                (*(g_swid_data.pUpdateBin + 0x1FFF6));
#endif
            TS_LOG_INFO("The updated firmware file major=0x%x, minor=0x%x\n", nUpdateBinMajor, nUpdateBinMinor);
            mstar_get_customer_fw_ver_dbbus(EMEM_MAIN, &nMajor, &nMinor, &pVersion);
            TS_LOG_INFO("The IC fw version = 0x%x\n", nMinor);

            if ((nUpdateBinMinor & 0xFF) > (nMinor & 0xFF)) {
                if (g_fw_ver_flag) {
                    TS_LOG_DEBUG
                        ("SwId=0x%x, nMajor=%u, nMinor=%u.%u, nUpdateBinMajor=%u, nUpdateBinMinor=%u.%u\n",
                         ic_sw_id, nMajor, nMinor & 0xFF, (nMinor & 0xFF00) >> 8, nUpdateBinMajor,
                         nUpdateBinMinor & 0xFF, (nUpdateBinMinor & 0xFF00) >> 8);
                } else {
                    TS_LOG_DEBUG
                        ("SwId=0x%x, nMajor=%d, nMinor=%d, nUpdateBinMajor=%u, nUpdateBinMinor=%u\n",
                         ic_sw_id, nMajor, nMinor, nUpdateBinMajor, nUpdateBinMinor);
                }

                g_fw_data = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
                if (ERR_ALLOC_MEM(g_fw_data)) {
                    TS_LOG_ERR("Failed to allocate FW buffer\n");
                    goto out;
                }

                for (i = 0; i < 130; i++) {
                    g_fw_data[i] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);

                    if (ERR_ALLOC_MEM(g_fw_data[i])) {
                        TS_LOG_ERR("Failed to allocate FW buffer\n");
                        goto out;
                    }
                }

                for (i = 0; i < MSG28XX_FIRMWARE_WHOLE_SIZE; i++) {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
                    mstar_fw_store_data(g_swid_data.pUpdateBin[i], 1024);
#else
                    mstar_fw_store_data((g_swid_data.pUpdateBin + i * 1024), 1024);
#endif
                }

                if ((ic_sw_id != 0x0000) && (ic_sw_id != 0xFFFF)) {
                    g_fw_data_cont = 0;             // Reset g_fw_data_cont to 0 after copying update firmware data to temp buffer
                    g_update_retry_cont = UPDATE_FIRMWARE_RETRY_COUNT;
                    g_update_info_block_first = 1;  // Set 1 for indicating main block is complete
                    g_fw_update = 0x11;

                    TS_LOG_INFO("Start to upgrade new firmware process\n");
                    mstar_update_fw_swid_do_work(NULL);

                } else {
                    TS_LOG_INFO("The MAIN_MEM swid is invalid, go to normal boot up process\n");
                }
            } else {
                TS_LOG_INFO("update version is older than or equal to the current version, go to normal boot up process\n");
            }
        } else {
            TS_LOG_INFO("The MAIN_MEM swid is not equal updated fw swid, go to normal boot up process\n");
        }
    } else {

        TS_LOG_INFO("IC firmware is lost, force to upgrade app firmware\n");
        g_swid_data.pUpdateBin = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
        if (ERR_ALLOC_MEM(g_swid_data.pUpdateBin)) {
            TS_LOG_ERR("Failed to allocate FW buffer\n");
            goto out;
        }

        for (i = 0; i < 130; i++) {
            g_swid_data.pUpdateBin[i] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);
            if (ERR_ALLOC_MEM(g_swid_data.pUpdateBin[i])) {
                TS_LOG_ERR("Failed to allocate FW buffer\n");
                goto out;
            }
        }

        g_fw_data = (u8 **) kmalloc(130 * sizeof(u8 *), GFP_KERNEL);
        if (ERR_ALLOC_MEM(g_fw_data)) {
            TS_LOG_ERR("Failed to allocate FW buffer\n");
            goto out;
        }

        for (i = 0; i < 130; i++) {
            g_fw_data[i] = (u8 *) kmalloc(1024 * sizeof(u8), GFP_KERNEL);
            if (ERR_ALLOC_MEM(g_fw_data[i])) {
                TS_LOG_ERR("Failed to allocate FW buffer\n");
                goto out;
            }
        }

        for (k = 0; k < 130; k++) {
            for (l = 0; l < 1024; l++) {
                g_swid_data.pUpdateBin[k][l] = fw->data[copy_count];
                copy_count++;
            }
        }

        for (i = 0; i < MSG28XX_FIRMWARE_WHOLE_SIZE; i++) {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
            mstar_fw_store_data(g_swid_data.pUpdateBin[i], 1024);
#else
            mstar_fw_store_data((g_swid_data.pUpdateBin + i * 1024), 1024);
#endif
        }

        g_fw_data_cont = 0;             // Reset g_fw_data_cont to 0 after copying update firmware data to temp buffer
        g_update_retry_cont = UPDATE_FIRMWARE_RETRY_COUNT;
        g_update_info_block_first = 0;  // Set 0 for indicating main block is broken
        g_fw_update = 0x11;
        mstar_update_fw_swid_do_work(NULL);
    }

out:
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
    if (!ERR_ALLOC_MEM(g_swid_data.pUpdateBin)) {
        for (i = 0; i < 130; i++) {
            if (!ERR_ALLOC_MEM(g_swid_data.pUpdateBin[i])) {
                kfree(g_swid_data.pUpdateBin[i]);
                g_swid_data.pUpdateBin[i] = NULL;
            }
        }

        kfree(g_swid_data.pUpdateBin);
        g_swid_data.pUpdateBin = NULL;
    }
#endif
    if (!ERR_ALLOC_MEM(g_one_dimen_fw_data)) {
        vfree(g_one_dimen_fw_data);
        g_one_dimen_fw_data = NULL;
    }

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
    mstar_dev_hw_reset();
    mstar_finger_touch_report_enable();
}
#endif

static void mstar_update_fw_swid_do_work(struct work_struct *pWork)
{
    s32 nRetVal = -1;

    TS_LOG_INFO("*** %s() g_update_retry_cont = %d ***\n", __func__, g_update_retry_cont);

    if (g_chip_type == CHIP_TYPE_MSG28XX || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
        || g_chip_type == CHIP_TYPE_ILI2118A) {
#ifdef CONFIG_ENABLE_CHIP_TYPE_MSG28XX
        nRetVal = mstar_28xx_fw_update(g_fw_data, EMEM_MAIN);
#endif
    } else {
        TS_LOG_INFO("This chip type (0x%x) does not support update firmware by sw id\n", g_chip_type);

        mstar_dev_hw_reset();

        mstar_finger_touch_report_enable();

        nRetVal = -1;
        return;
    }

    TS_LOG_INFO("*** Update firmware by sw id result = %d ***\n", nRetVal);

    if (nRetVal == 0) {
        TS_LOG_INFO("Update firmware by sw id success\n");

        mstar_dev_hw_reset();

        mstar_finger_touch_report_enable();

        if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX
            || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
            || g_chip_type == CHIP_TYPE_ILI2118A) {
            g_update_info_block_first = 0;
            g_fw_update = 0x00;
        }
    } else {         // nRetVal == -1 for MSG22xx/MSG28xx/MSG58xxa or nRetVal == 2/3/4 for ILI21xx
        g_update_retry_cont--;
        if (g_update_retry_cont > 0) {
            TS_LOG_INFO("g_update_retry_cont = %d\n", g_update_retry_cont);
            mstar_update_fw_swid_do_work(NULL);
        } else {
            TS_LOG_INFO("Update firmware by sw id failed\n");

            mstar_dev_hw_reset();

            mstar_finger_touch_report_enable();

            if (g_chip_type == CHIP_TYPE_MSG22XX || g_chip_type == CHIP_TYPE_MSG28XX
                || g_chip_type == CHIP_TYPE_MSG58XXA || g_chip_type == CHIP_TYPE_ILI2117A
                || g_chip_type == CHIP_TYPE_ILI2118A) {
                g_update_info_block_first = 0;
                g_fw_update = 0x00;
            }
#if defined (CONFIG_HUAWEI_DSM)
            if (!dsm_client_ocuppy(ts_dclient)) {
                dsm_client_record(ts_dclient, "Mstar Update firmware by sw id failed\n");
                dsm_client_notify(ts_dclient,DSM_TP_FWUPDATE_ERROR_NO);
            }
#endif
        }
    }
}
#endif
