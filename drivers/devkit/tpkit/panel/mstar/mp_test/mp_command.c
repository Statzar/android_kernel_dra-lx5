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
 * Author: Dicky Chiang
 * Maintain: Luca Hsu, Tigers Huang
 */

#include "mp_common.h"

int ascii_to_hex(char ch)
{
    char ch_tmp;
    int hex_val = -1;

    ch_tmp = tolower(ch);

    if ((ch_tmp >= '0') && (ch_tmp <= '9')) {
        hex_val = ch_tmp - '0';
    } else if ((ch_tmp >= 'a') && (ch_tmp <= 'f')) {
        hex_val = ch_tmp - 'a' + 10;
    }

    return hex_val;
}

int str_to_hex(char *hex_str)
{
    int i, len;
    int hex_tmp, hex_val;

    len = strlen(hex_str);
    hex_val = 0;
    for (i = 0; i < len; i++) {
        hex_tmp = ascii_to_hex(hex_str[i]);

        if (hex_tmp == -1) {
            return -1;
        }

        hex_val = (hex_val) * 16 + hex_tmp;
    }
    return hex_val;
}

u8 mstar_check_double_value_in_range(s32 nValue, s32 nMax, s32 nMin)
{
    if (nValue <= nMax && nValue >= nMin) {
        return 1;
    } else {
        return 0;
    }
}

u8 mstar_check_value_in_range(s32 nValue, s32 nMax, s32 nMin)
{
    if (nValue <= nMax && nValue >= nMin) {
        return 1;
    } else {
        return 0;
    }
}

static void mstar_read_flash_finale_28xx(void)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);

    // set read done
    mstar_set_reg_16bit(0x1606, 0x02);

    // unset info flag
    mstar_set_reg_low_byte(0x1607, 0x00);

    // clear addr
    mstar_set_reg_16bit(0x1600, 0x00);
}

static void mstar_read_flash_init_28xx(u16 cayenne_address, int nBlockType)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);

    //wriu 0x1608 0x20
    mstar_set_reg_16bit(0x1608, 0x20);
    //wriu 0x1606 0x20
    mstar_set_reg_16bit(0x1606, 0x20);

    //wriu read done
    mstar_set_reg_16bit(0x1606, 0x02);

    //set address
    mstar_set_reg_16bit(0x1600, cayenne_address);

    if (nBlockType == EMEM_TYPE_INFO_BLOCK) {
        //set Info Block
        mstar_set_reg_low_byte((uint) 0x1607, (uint) 0x08);

        //set Info Double Buffer
        mstar_set_reg_low_byte((uint) 0x1604, (uint) 0x01);
    } else {
        //set Main Block
        mstar_set_reg_low_byte((uint) 0x1607, (uint) 0x00);

        //set Main Double Buffer
        mstar_set_reg_low_byte((uint) 0x1604, (uint) 0x01);
    }
    // set FPGA flag
    mstar_set_reg_low_byte(0x1610, 0x01);

    // set mode trigger
    if (nBlockType == EMEM_TYPE_INFO_BLOCK) {
        // set eflash mode to read mode
        // set info flag
        mstar_set_reg_16bit(0x1606, 0x0801);
    } else {
        // set eflash mode to read mode
        mstar_set_reg_16bit(0x1606, 0x0001);
    }
}

static int mstar_read_flash_RIU_28xx(u32 nAddr, int nBlockType, int nLength, u8 * pFlashData)
{
    uint read_16_addr_a = 0, read_16_addr_c = 0;

    TS_LOG_DEBUG("*** %s() nAddr:0x%x***\n", __func__, nAddr);

    //set read address
    mstar_set_reg_16bit(0x1600, nAddr);

    //read 16+16 bits
    read_16_addr_a = mstar_get_reg_16bit(0x160a);
    read_16_addr_c = mstar_get_reg_16bit(0x160c);

    //DEBUG("*** %s() read_16_addr_a:%x read_16_addr_c:%x***\n",__func__,read_16_addr_a,read_16_addr_c);

    pFlashData[0] = (u8) (read_16_addr_a & 0xff);
    pFlashData[1] = (u8) ((read_16_addr_a >> 8) & 0xff);
    pFlashData[2] = (u8) (read_16_addr_c & 0xff);
    pFlashData[3] = (u8) ((read_16_addr_c >> 8) & 0xff);
    //DEBUG("*** %s() pFlashData[0]:0x%x pFlashData[1]:0x%x pFlashData[2]:0x%x pFlashData[3]:0x%x***\n",
    //          __func__,pFlashData[0],pFlashData[1],pFlashData[2],pFlashData[3]);
    return 0;
}

static int mstar_read_flash_28xx(u32 nAddr, int nBlockType, int nLength, u8 * pFlashData)
{
    u16 _28xx_addr = nAddr / 4;
    u32 addr_star, addr_end, addr_step;
    u32 read_byte = 0;

    addr_star = nAddr;
    addr_end = nAddr + nLength;

    if ((addr_star >= EMEM_SIZE_MSG28XX) || (addr_end > EMEM_SIZE_MSG28XX)) {
        TS_LOG_DEBUG("*** %s : addr_start = 0x%x , addr_end = 0x%x *** \n", __func__, addr_star, addr_end);
        return -1;
    }

    addr_step = 4;

    mstar_read_flash_init_28xx(_28xx_addr, nBlockType);

    for (addr_star = nAddr; addr_star < addr_end; addr_star += addr_step) {
        _28xx_addr = addr_star / 4;

        TS_LOG_DEBUG("*** %s() _28xx_addr:0x%x addr_star:0x%x addr_end:%x nLength:%d pFlashData:%p***\n",
                 __func__, _28xx_addr, addr_star, addr_end, nLength, pFlashData);
        mstar_read_flash_RIU_28xx(_28xx_addr, nBlockType, nLength, (pFlashData + read_byte));
        TS_LOG_DEBUG("*** %s() pFlashData[%x]: %02x %02x %02x %02x read_byte:%d \n", __func__, addr_star,
                 pFlashData[read_byte], pFlashData[read_byte + 1], pFlashData[read_byte + 2],
                 pFlashData[read_byte + 3], read_byte);
        //pFlashData+=addr_step;
        read_byte += 4;
    }

    mstar_read_flash_finale_28xx();

    return 0;
}

// Start mcu
void mstar_mp_start_mcu(void)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);
    // Start mcu
    mstar_set_reg_low_byte(0x0FE6, 0x00);   //bank:mheg5, addr:h0073
}

// Stop mcu
void mstar_mp_stop_mcu(void)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);
    mstar_set_reg_low_byte(0x0FE6, 0x01);   //bank:mheg5, addr:h0073
}

void mstar_mp_exit_dbbus(void)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);
    mstar_dbbus_iic_not_use_bus();
    mstar_dbbus_not_stop_mcu();
    mstar_dbbus_exit_serial_debug();
}

void mstar_mp_dbbus_enter(void)
{
    TS_LOG_DEBUG("*** %s *** \n", __func__);
    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_stop_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
}

// s32 mstar_mp_check_switch_status(void)
// {
    // u32 nRegData = 0;
    // int nTimeOut = 280;
    // int nT = 0;

    // TS_LOG_DEBUG("*** %s() ***\n", __func__);
    // do
    // {
    // nRegData = mstar_get_reg_16bit(0x1402);
    // //udelay(20000);
    // mdelay(20);
    // nT++;
    // if (nT > nTimeOut)
    // {
        // return -1;
    // }
    // //DEBUG("nRegData = %x", nRegData);

    // } while (nRegData != 0x7447);

    // return 0;
// }

s32 mstar_enter_mp_mode(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_mp_stop_mcu();
    mdelay(100);
    mstar_set_reg_16bit(0X3C60, 0xAA55);    // disable watch dog
    mstar_set_reg_16bit(0X3D08, 0xFFFF);    // clear interrupt status
    mstar_set_reg_16bit(0X3D18, 0xFFFF);    // clear interrupt status

    mstar_set_reg_16bit(0x1402, 0x7474);    // enter mp mode

    mstar_set_reg_16bit(0x1E06, 0x0000);
    mstar_set_reg_16bit(0x1E06, 0x0001);
    mstar_mp_start_mcu();
    //udelay(300000);
    mdelay(300);

    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg28xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }
    return 0;
}

int mstar_mp_read_flash(u8 nChipType, u32 nAddr, int nBlockType, int nLength, u8 * pFlashData)
{
    int ret = 0;

    TS_LOG_DEBUG("*** %s()  nChipType 0x%x***\n", __func__, nChipType);

    switch (nChipType) {

    case CHIP_TYPE_MSG28XX:
    case CHIP_TYPE_MSG28XXA:
        ret = mstar_read_flash_28xx(nAddr, nBlockType, nLength, pFlashData);
        break;

    default:
        break;
    }

    return ret;
}

void mstar_reg_get_16bit_byte_value_buf(u16 nAddr, u8 * pBuf, u16 nLen)
{
    u16 i;
    u8 tx_data[3] = { 0x10, (nAddr >> 8) & 0xFF, nAddr & 0xFF };
    u8 *rx_data = (u8 *) kcalloc(nLen, sizeof(u8), GFP_KERNEL);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
    mstar_iic_read_data(SLAVE_I2C_ID_DBBUS, rx_data, nLen);

    for (i = 0; i < nLen; i++)
        pBuf[i] = rx_data[i];
    kfree(rx_data);
}

void mstar_mp_debug_show_array2(void *pBuf, u16 nLen, int nDataType, int nCarry, int nChangeLine)
{
    u8 *pU8Buf = NULL;
    s8 *pS8Buf = NULL;
    u16 *pU16Buf = NULL;
    s16 *pS16Buf = NULL;
    u32 *pU32Buf = NULL;
    s32 *pS32Buf = NULL;
    char *szStrBuf = NULL;
    char szStrTmp[10] = { 0 };

    int i;

    TS_LOG_DEBUG(" %s \n", __func__);

    szStrBuf = kcalloc(1024, sizeof(char), GFP_KERNEL);
    if (ERR_ALLOC_MEM(szStrBuf)) {
        TS_LOG_ERR("Failed to allocate szStrBuf mem\n");
        return;
    }

    if (nDataType == 8)
        pU8Buf = (u8 *) pBuf;
    else if (nDataType == -8)
        pS8Buf = (s8 *) pBuf;
    else if (nDataType == 16)
        pU16Buf = (u16 *) pBuf;
    else if (nDataType == -16)
        pS16Buf = (s16 *) pBuf;
    else if (nDataType == 32)
        pU32Buf = (u32 *) pBuf;
    else if (nDataType == -32)
        pS32Buf = (s32 *) pBuf;

    for (i = 0; i < nLen; i++) {
        if (nCarry == 16) {
            if (nDataType == 8)
                sprintf(szStrTmp, "%02X ", pU8Buf[i]);
            else if (nDataType == -8)
                sprintf(szStrTmp, "%02X ", pS8Buf[i]);
            else if (nDataType == 16)
                sprintf(szStrTmp, "%04X ", pU16Buf[i]);
            else if (nDataType == -16)
                sprintf(szStrTmp, "%04X ", pS16Buf[i]);
            else if (nDataType == 32)
                sprintf(szStrTmp, "%08X ", pU32Buf[i]);
            else if (nDataType == -32)
                sprintf(szStrTmp, "%08X ", pS32Buf[i]);
        } else if (nCarry == 10) {
            if (nDataType == 8)
                sprintf(szStrTmp, "%6u ", pU8Buf[i]);
            else if (nDataType == -8)
                sprintf(szStrTmp, "%6d ", pS8Buf[i]);
            else if (nDataType == 16)
                sprintf(szStrTmp, "%6u ", pU16Buf[i]);
            else if (nDataType == -16)
                sprintf(szStrTmp, "%6d ", pS16Buf[i]);
            else if (nDataType == 32)
                sprintf(szStrTmp, "%6u ", pU32Buf[i]);
            else if (nDataType == -32)
                sprintf(szStrTmp, "%6d ", pS32Buf[i]);
        }

        strcat(szStrBuf, szStrTmp);
        memset(szStrTmp, 0, 10);
        if (i % nChangeLine == nChangeLine - 1) {
            TS_LOG_DEBUG("%s\n", szStrBuf);
            memset(szStrBuf, 0, 1024);
        }
    }

    TS_LOG_DEBUG("\n");
    kfree(szStrBuf);
}

void mstar_mp_dbbus_re_enter(void)
{
    u8 nParCmdSelUseCfg = 0x7F;
    u8 nParCmdAdByteEn1 = 0x51;
    u8 nPar_N_CmdUSetSelB0 = 0x80;
    u8 nPar_N_CmdUSetSelB1 = 0x82;
    u8 nPar_N_CmdSetSelB2 = 0x84;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdSelUseCfg, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn1, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nPar_N_CmdUSetSelB0, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nPar_N_CmdUSetSelB1, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nPar_N_CmdSetSelB2, 1);
}

void mstar_mp_ddbus_read_dq_mem_start(void)
{
    u8 nParCmdSelUseCfg = 0x7F;
    u8 nParCmdAdByteEn0 = 0x50;
    u8 nParCmdAdByteEn2 = 0x52;
    u8 nParCmdAdByteEn1 = 0x51;
    u8 nParCmdDaByteEn0 = 0x54;
    u8 nParCmdUSetSelB0 = 0x80;
    u8 nParCmdUSetSelB1 = 0x82;
    u8 nParCmdSetSelB2 = 0x85;
    u8 nParCmdIicUse = 0x35;
    //u8 nParCmdWr        = 0x10;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_set_reg_16bit_off(0x0F50, BIT1);

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdSelUseCfg, 1);

    if(mp_test_info.mode == 2) {
        //mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn0, 1);
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn2, 1);
        //mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdDaByteEn0, 1);
    } else {
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn0, 1);
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdAdByteEn1, 1);
        mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdDaByteEn0, 1);
    }

    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB0, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdUSetSelB1, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdSetSelB2, 1);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdIicUse, 1);
}

void mstar_mp_dbbus_read_dq_mem_end(void)
{
    u8 nParCmdNSelUseCfg = 0x7E;

    if(mp_test_info.mode == 2)
        nParCmdNSelUseCfg = 0x7F;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, &nParCmdNSelUseCfg, 1);
    mstar_set_reg_16bit_on(0x0F50, BIT1);
}

void mstar_mp_enable_adc_one_shot(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_set_reg_16bit_on(0x100a, BIT0);

    return;
}

void mstar_mp_ana_change_cd_time(u16 T1, u16 T2)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_set_reg_16bit_by_addr(0x1013, T1, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1018, T2, ADDRESS_MODE_16BIT);
}

s32 mstar_mp_trigger_mutual_one_shot(s16 * pResultData, u16 * pSenNum, u16 * pDrvNum, u16 drv_mode)
{
    u8 nShotData[476] = { 0 };  //14*17*2
    u32 nAddr = 0, nAddrNextSF = 0;
    u16 nSF = 0, nAfeOpening = 0, nDriOpening = 0;
    u16 nMaxDataNumOfOneSF = 0;
    u16 nDriMode = 0;
    u32 nDataShift = -1;
    u16 nRegData = 0, nSwcap = 0;
    s16 *pShotDataAll = NULL;
    u16 i, j, k;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_dbbus_iic_use_bus();

    nRegData = mstar_get_reg_16bit_by_addr(0x1305, ADDRESS_MODE_16BIT);
    nSF = (nRegData & (BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13)) >> 8;
    nAfeOpening = nRegData & (BIT0 | BIT1 | BIT2 | BIT3);

    if (nSF == 0)
        return -1;

    nRegData = mstar_get_reg_16bit_by_addr(0x1005, ADDRESS_MODE_16BIT);
    nDriMode = ((nRegData & BIT9) >> 8);

    nRegData = mstar_get_reg_low_byte(0x100B);
    nDriMode = nRegData;

    nDriOpening = msg30xx_get_drv_opening();

    TS_LOG_DEBUG
        ("*** Msg30xx MP Test# TriggerMutualOneShot nSF=%d, nAfeOpening=%d, drv_mode=%d, nDriOpening=%d. ***\n",
         nSF, nAfeOpening, drv_mode, nDriOpening);

    nMaxDataNumOfOneSF = nAfeOpening * nDriOpening;
    nAddrNextSF = nMaxDataNumOfOneSF * 2;
    nAddr = g_fout_base_addr;

    if (nMaxDataNumOfOneSF == 0) {
        TS_LOG_ERR("nMaxDataNumOfOneSF == 0");
        return -1;
    }

    mstar_set_reg_16bit_off(0x3D08, BIT8);  ///FIQ_E_FRAME_READY_MASK
    mstar_reg_mask_16bit(0x0F28, BIT1, BIT1, ADDRESS_MODE_16BIT);

    nRegData = mstar_get_reg_16bit_by_addr(0x1100, ADDRESS_MODE_16BIT);
    nSwcap = (nRegData & BIT15);
    if (nSwcap)
        mstar_mp_enable_adc_one_shot();
    else            // sine mode
        mstar_reg_mask_16bit(0x1110, BIT1, BIT1, ADDRESS_MODE_16BIT);

    ///polling frame-ready interrupt status
    while (0x0000 == (nRegData & BIT8)) {
        nRegData = mstar_get_reg_16bit(0x3D18);
    }

    if (nAfeOpening % 2 == 0)
        nDataShift = -1;
    else
        nDataShift = 0; //special case

    if ((nAddrNextSF % 4) != 0)
        nAddrNextSF = ((nAddrNextSF + 4) / 4) * 4;

    if (drv_mode == 2)  // for short test
    {
        u32 data_length = nAddrNextSF;
        data_length += (nDataShift == 0) ? 2 : 0;
        pShotDataAll = (s16 *) kmalloc(sizeof(s16) * nSF * nMaxDataNumOfOneSF, GFP_KERNEL);

        /// get ALL raw data, combine and handle datashift.
        for (i = 0; i < nSF; i++) {
            memset(nShotData, 0, sizeof(nShotData));
            mstar_mp_ddbus_read_dq_mem_start();
            //RegGetXByteValue(nShotData, nAddr + i * nAddrNextSF, data_length);
            mstar_get_reg_xbit(nAddr + i * nAddrNextSF, nShotData, data_length,
                       MAX_I2C_TRANSACTION_LENGTH_LIMIT);
            mstar_mp_dbbus_read_dq_mem_end();

            for (j = 0; j < nMaxDataNumOfOneSF; j++) {
                pShotDataAll[i * nMaxDataNumOfOneSF + j] =
                    (s16) (nShotData[2 * j] | nShotData[2 * j + 1] << 8);

                if (nDataShift == 0 && j == (nMaxDataNumOfOneSF - 1))
                    pShotDataAll[i * nMaxDataNumOfOneSF + j + 1] =
                        (s16) (nShotData[2 * (j + 1)] | nShotData[2 * (j + 1) + 1] << 8);
                //TS_LOG_DEBUG( "pShotDataAll[%d] = %d", i*nMaxDataNumOfOneSF+j, pShotDataAll[i*nMaxDataNumOfOneSF+j]);
            }
        }

        //problem here
        for (k = 0; k < nSF; k++) {
            for (i = k * nAfeOpening; i < nAfeOpening * (k + 1); i++)   //Sen
            {
                pResultData[i] = pShotDataAll[k * nMaxDataNumOfOneSF + ((i - nAfeOpening * k) * nDriOpening)];  //resultData[Sen, Dri]
                //TS_LOG_DEBUG("pResultData[%d] = %d", i, pResultData[i]);
            }
        }

        *pSenNum = nSF * nAfeOpening;
        *pDrvNum = nDriOpening;
        kfree(pShotDataAll);
    } else          // for open test
    {
        u16 frame_multiply = (g_two_dac_enable == ENABLE) ? 2 : 1;
        u32 data_length = nAddrNextSF;
        data_length += (nDataShift == 0) ? 2 : 0;
        pShotDataAll = (s16 *) kmalloc(sizeof(s16) * nSF * frame_multiply * nMaxDataNumOfOneSF, GFP_KERNEL);

        /// get ALL raw data, combine and handle datashift.
        for (i = 0; i < (nSF * frame_multiply); i++) {
            memset(nShotData, 0, sizeof(nShotData));
            mstar_mp_ddbus_read_dq_mem_start();
            //RegGetXByteValue(nShotData, nAddr + i * nAddrNextSF, data_length);
            mstar_get_reg_xbit(nAddr + i * nAddrNextSF, nShotData, data_length,
                       MAX_I2C_TRANSACTION_LENGTH_LIMIT);
            mstar_mp_dbbus_read_dq_mem_end();

            for (j = 0; j < nMaxDataNumOfOneSF; j++) {
                pShotDataAll[i * nMaxDataNumOfOneSF + j] =
                    (s16) (nShotData[2 * j] | nShotData[2 * j + 1] << 8);

                if (nDataShift == 0 && j == (nMaxDataNumOfOneSF - 1))
                    pShotDataAll[i * nMaxDataNumOfOneSF + j + 1] =
                        (s16) (nShotData[2 * (j + 1)] | nShotData[2 * (j + 1) + 1] << 8);
                //TS_LOG_DEBUG( "pShotDataAll[%d] = %d", i*nMaxDataNumOfOneSF+j, pShotDataAll[i*nMaxDataNumOfOneSF+j]);
            }
        }

        //problem here
        for (k = 0; k < (nSF * frame_multiply); k++) {
            for (i = k * nAfeOpening; i < nAfeOpening * (k + 1); i++)   //Sen
            {
                for (j = 0; j < nDriOpening; j++)   //Dri
                {
                    pResultData[i * MAX_CHANNEL_DRV + j] = pShotDataAll[k * nMaxDataNumOfOneSF + (j + (i - nAfeOpening * k) * nDriOpening)];    //resultData[Sen, Dri]
                }
            }
        }

        *pSenNum = nSF * nAfeOpening;
        *pDrvNum = nDriOpening;
        kfree(pShotDataAll);
    }
    mstar_set_reg_16bit_on(0x3D08, BIT8);   ///FIQ_E_FRAME_READY_MASK
    mstar_set_reg_16bit_on(0x3D08, BIT4);   ///FIQ_E_TIMER0_MASK
    mstar_reg_mask_16bit(0x3D0C, BIT8, BIT8, ADDRESS_MODE_16BIT);   //clear frame ready status
    return 0;
}

s32 mstar_mp_get_mutual_one_shot_raw_iir(s16 * nResultData, u16 * pSenNum, u16 * pDrvNum, u16 drv_mode)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    return mstar_mp_trigger_mutual_one_shot(nResultData, pSenNum, pDrvNum, drv_mode);
}

s32 mstar_mp_check_switch_status(void)
{
    u16 nRegData = 0;
    int nTimeOut = 280;
    int nT = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    do {
        nRegData = mstar_get_reg_16bit_by_addr(0x1401, ADDRESS_MODE_16BIT);
        mdelay(20);
        nT++;
        if (nT > nTimeOut) {
            return -1;
        }
        TS_LOG_DEBUG("nT = %d, nRegData = 0x%04x\n", nT, nRegData);
    } while (nRegData != 0x7447);

    return 0;
}

s32 mstar_mp_switch_fw_mode(u16 * nFMode, u16 * deep_standby)   //201703xx
{
    u16 nRegData = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_stop_mcu();
    mdelay(100);
    mstar_set_reg_16bit(0X3C60, 0xAA55);    // disable watch dog

    mstar_set_reg_16bit(0X3D08, 0xFFFF);    // clear interrupt status
    mstar_set_reg_16bit(0X3D18, 0xFFFF);    // clear interrupt status

    mstar_set_reg_16bit(0x1402, 0x7474);    // enter mp mode

    mstar_set_reg_16bit(0x1E06, 0x0000);
    mstar_set_reg_16bit(0x1E06, 0x0001);
    mstar_mp_start_mcu();
    mdelay(300);

    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg30xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }
    //deep standby mode
    if (*deep_standby == 1) {
        TS_LOG_DEBUG("*** Msg30xx MP Test# enter deep standby! ***\n");
        mstar_set_reg_16bit(0x1402, 0x6179);
        TS_LOG_DEBUG("deep_standby_timeout = %d", mutual_mp_test_data->deep_standby_timeout);
        mdelay(mutual_mp_test_data->deep_standby_timeout);  // depend on ini, default = 450ms

        mstar_mp_dbbus_enter();

        if (mstar_mp_check_switch_status() < 0) {
            *deep_standby = 0;
            TS_LOG_ERR("*** Msg30xx MP Test# enter deep standby FAILED! ***\n");
            return -1;
        }
    }

    g_scan_mode = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
    g_code_type = mstar_get_reg_16bit_by_addr(0x1362, ADDRESS_MODE_16BIT);
    g_two_dac_enable = g_code_type & 0x00FF;
    TS_LOG_DEBUG("nFMode = %x\n", *nFMode);

    switch (*nFMode) {
    case MUTUAL_MODE:
        mstar_set_reg_16bit(0x1402, 0x5705);
        break;

    case MUTUAL_SINE:
        mstar_set_reg_16bit(0x1402, 0x5706);
        break;

    case SELF:
        mstar_set_reg_16bit(0x1402, 0x6278);
        break;

    case MUTUAL_SINGLE_DRIVE:
        mstar_set_reg_16bit(0x1402, 0x0158);
        break;

    case SINE_PHASE:
        mstar_set_reg_16bit(0x1402, 0xECFA);
        break;

    default:
        return -1;
    }
    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg30xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }

    switch (*nFMode) {
    case MUTUAL_MODE:
    case MUTUAL_SINE:
    case MUTUAL_KEY:
    case MUTUAL_SINGLE_DRIVE:
    case MUTUAL_SINE_KEY:
        nRegData = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
        g_fout_base_addr = (u16) (nRegData << 2);
        TS_LOG_DEBUG("g_fout_base_addr = 0x%04x\n", g_fout_base_addr);
        break;
    default:
        return -1;
    }

    mstar_mp_stop_mcu();
    mstar_set_reg_16bit(0x3D08, 0xFEFF);    //open timer

    return 0;
}

void mstar_mp_ana_sw_reset(void)    //301703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_set_reg_16bit_on(0x1001, (BIT0 | BIT1 | BIT2 | BIT3));    ///reg_tgen_soft_rst: 1 to reset
    mstar_set_reg_16bit_off(0x1001, (BIT0 | BIT1 | BIT2 | BIT3));

    /// delay
    mdelay(20);
}

void mstar_mp_ana_fix_prs(u16 option)   //201703xx
{
//    u16 regData = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_stop_mcu();
    mstar_reg_mask_16bit(0x1004, BIT1 | BIT2, option << 1, ADDRESS_MODE_16BIT);
}

s32 mstar_enter_mp_mode_30xx()  //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_stop_mcu();
    mdelay(100);
    mstar_set_reg_16bit(0X3C60, 0xAA55);    // disable watch dog
    mstar_set_reg_16bit(0X3D08, 0xFFFF);    // clear interrupt status
    mstar_set_reg_16bit(0X3D18, 0xFFFF);    // clear interrupt status

    mstar_set_reg_16bit(0x1402, 0x7474);    // enter mp mode

    mstar_set_reg_16bit(0x1E06, 0x0000);
    mstar_set_reg_16bit(0x1E06, 0x0001);
    mstar_mp_start_mcu();
    mdelay(300);

    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg30xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }

    g_scan_mode = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
    g_code_type = mstar_get_reg_16bit_by_addr(0x1362, ADDRESS_MODE_16BIT);
    g_two_dac_enable = g_code_type & 0x00FF;

    switch (mutual_mp_test_data->Open_mode) {
    case 0:
        mstar_set_reg_16bit(0x1402, MUTUAL_MODE);
        break;
    case 1:
        mstar_set_reg_16bit(0x1402, MUTUAL_SINE);
        break;
    }

    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg30xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }

    return 0;
}

u16 msg30xx_get_drv_opening(void)   //201703xx
{
    u16 dri_opening = 0;
    u16 nRegData = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    nRegData = mstar_get_reg_16bit_by_addr(0x1306, ADDRESS_MODE_16BIT);
    dri_opening = nRegData & (BIT0 | BIT1 | BIT2 | BIT3 | BIT4);

    return dri_opening;
}

void msg30xx_scan_dac_setting(u16 nTwoDAC)  //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    msg30xx_Tgen_Ovwr_dac_enable(ENABLE, eDAC_0);

    if (nTwoDAC == ENABLE) {
        msg30xx_Tgen_Ovwr_dac_enable(ENABLE, eDAC_1);
    } else {
        msg30xx_Tgen_Ovwr_dac_enable(DISABLE, eDAC_1);
    }
}

void msg30xx_Tgen_Ovwr_dac_enable(u16 nEnable, u16 eDACSelect)  //201703xx
{
    //BIT0 : DAC enable or not
    //BIT1 : DAC LDO enable or not
    u16 nAddr = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (eDACSelect == eDAC_0) {
        nAddr = 0x1537;
    } else {
        nAddr = 0x153F;
    }

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(nAddr, BIT0 | BIT1, BIT0 | BIT1, ADDRESS_MODE_16BIT);
    } else {
        mstar_reg_mask_16bit(nAddr, BIT0 | BIT1, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_reg_en(u16 nEnable)  //201703xx
{
    //reg_en_afe_refbuf : bit0
    //reg_en_bg : bit1
    //reg_en_csub_refbuf : bit2
    //reg_en_lvdrvg_refbuf : bit3
    //reg_en_lvdrvm_refbuf : bit4
    //reg_en_lvdrvp_refbuf : bit5
    //reg_en_ref : bit6

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x152A, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6,
                     BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6, ADDRESS_MODE_16BIT);
    } else {
        mstar_reg_mask_16bit(0x152A, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_invert_mode(u16 nEnable, u16 nTwoDAC)    //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x1532, BIT0, BIT0, ADDRESS_MODE_16BIT);
        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x153A, BIT0, BIT0, ADDRESS_MODE_16BIT);
        } else {
            mstar_reg_mask_16bit(0x153A, BIT0, 0, ADDRESS_MODE_16BIT);
        }
    } else {
        mstar_reg_mask_16bit(0x1532, BIT0, 0, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x153A, BIT0, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_DrvL_Buf_cfg_setting(u16 nEnable, u16 nTwoDAC)   //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x1534, BIT0 | BIT1 | BIT4 | BIT5, BIT1 | BIT5, ADDRESS_MODE_16BIT);
        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x153C, BIT0 | BIT1 | BIT4 | BIT5, BIT1 | BIT5, ADDRESS_MODE_16BIT);
        } else {
            mstar_reg_mask_16bit(0x153C, BIT0 | BIT1 | BIT4 | BIT5, 0, ADDRESS_MODE_16BIT);
        }
    } else {
        mstar_reg_mask_16bit(0x1534, BIT0 | BIT1 | BIT4 | BIT5, BIT0 | BIT1 | BIT4 | BIT5, ADDRESS_MODE_16BIT);
        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x153C, BIT0 | BIT1 | BIT4 | BIT5, BIT0 | BIT1 | BIT4 | BIT5,
                         ADDRESS_MODE_16BIT);
        } else {
            mstar_reg_mask_16bit(0x153C, BIT0 | BIT1 | BIT4 | BIT5, 0, ADDRESS_MODE_16BIT);
        }
    }
}

void msg30xx_Tgen_Ovwr_DrvL_Buf_gain_setting(u16 nEnable, u16 nTwoDAC)  //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x1534, BIT8 | BIT9 | BIT12 | BIT13, 0, ADDRESS_MODE_16BIT);

        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x153C, BIT8 | BIT9 | BIT12 | BIT13, 0, ADDRESS_MODE_16BIT);
        } else {
            mstar_reg_mask_16bit(0x153C, BIT9 | BIT13, BIT9 | BIT13, ADDRESS_MODE_16BIT);
        }
    } else {
        mstar_reg_mask_16bit(0x1534, BIT9 | BIT13, BIT9 | BIT13, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x153C, BIT9 | BIT13, BIT9 | BIT13, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_DrvH_comp_setting(u16 nEnable)   //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x1531, BIT8 | BIT10, BIT8 | BIT10, ADDRESS_MODE_16BIT);
    } else {
        mstar_reg_mask_16bit(0x1531, BIT8 | BIT10, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_Ross_select(u16 nEnable, u16 nTwoDAC)    //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        mstar_reg_mask_16bit(0x1531, BIT6 | BIT7, 0, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1531, BIT14 | BIT15, 0, ADDRESS_MODE_16BIT);

        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x1539, BIT6 | BIT7, 0, ADDRESS_MODE_16BIT);
            mstar_reg_mask_16bit(0x1539, BIT14 | BIT15, 0, ADDRESS_MODE_16BIT);
        }
    } else {
        mstar_reg_mask_16bit(0x1531, BIT6 | BIT7, BIT6 | BIT7, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1531, BIT14 | BIT15, BIT14 | BIT15, ADDRESS_MODE_16BIT);

        if (nTwoDAC == ENABLE) {
            mstar_reg_mask_16bit(0x1539, BIT6 | BIT7, BIT6 | BIT7, ADDRESS_MODE_16BIT);
            mstar_reg_mask_16bit(0x1539, BIT14 | BIT15, BIT14 | BIT15, ADDRESS_MODE_16BIT);
        }
    }
}

void msg30xx_Tgen_Ovwr_DrvH_setting(u16 nEnable)    //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == DISABLE) //Hv
    {
        mstar_reg_mask_16bit(0x1530, BIT0 | BIT1, BIT0 | BIT1, ADDRESS_MODE_16BIT);
    } else {
        mstar_reg_mask_16bit(0x1530, BIT0 | BIT1, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_Tgen_Ovwr_charge_pump_setting(u16 nEnable_charge_pump) //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable_charge_pump == DISABLE) {
        mstar_reg_mask_16bit(0x1433, BIT0 | BIT1, 0, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1540, BIT10, 0, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x152F, BIT8, BIT8, ADDRESS_MODE_16BIT);
    } else {
        mstar_reg_mask_16bit(0x1433, BIT0 | BIT1, BIT1, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1540, BIT10, BIT10, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x152F, BIT8, 0, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_ana_enable_charge_pump(u16 nEnable_charge_pump)    //201703xx
{

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    msg30xx_Tgen_Ovwr_charge_pump_setting(nEnable_charge_pump);
}

void msg30xx_gain_setting(void) //201703xx
{
    u8 Cfb = mutual_mp_test_data->Open_test_cfb;
    u16 decode_out = 0, dri_opening = 0, regdata = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    regdata = mstar_get_reg_16bit_by_addr(0x1306, ADDRESS_MODE_16BIT);  //get dri num
    dri_opening = regdata & (BIT0 | BIT1 | BIT2 | BIT3 | BIT4);

    mstar_set_reg_16bit_by_addr(0x1019, 0x083f, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101a, 0x0029, ADDRESS_MODE_16BIT);    //post idle time in sub0
    mstar_set_reg_16bit_by_addr(0x101b, 0x0029, ADDRESS_MODE_16BIT);    //post idle time in sub1
    mstar_set_reg_16bit_by_addr(0x101c, 0x0029, ADDRESS_MODE_16BIT);    //post idle time in sub2
    mstar_set_reg_16bit_by_addr(0x101d, 0x0029, ADDRESS_MODE_16BIT);    //post idle time in sub3

    // fw default setting
    msg30xx_scan_set_sample_num(26);
    mstar_set_reg_16bit_by_addr(0x136B, 0x9D89, ADDRESS_MODE_16BIT);

    ///
    switch (dri_opening) {
    case 1:
        decode_out = 0;
        break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 12:
    case 13:
    case 14:
    case 16:
        decode_out = 2;
        break;
    case 11:
    case 15:
    case 17:
        decode_out = 3;
        break;
    }
    // set shift decode out
    mstar_reg_mask_16bit(0x130D, BIT0 | BIT1 | BIT2 | BIT3, decode_out, ADDRESS_MODE_16BIT);
    // set shift all coef
    mstar_reg_mask_16bit(0x130D, BIT12 | BIT13 | BIT14 | BIT15, 0, ADDRESS_MODE_16BIT);
    // set shift fir out
    mstar_reg_mask_16bit(0x130D, BIT4 | BIT5 | BIT6 | BIT7, BIT6, ADDRESS_MODE_16BIT);

    /// all AFE Cfb use by autosettings.ini setting, defalt (50p)
    mstar_set_reg_16bit_by_addr(0x1504, 0x3FFF, ADDRESS_MODE_16BIT);    // all 14 AFE Cfb: SW control

    switch (Cfb) {
    case 0:
    case 2:
        Msg30xxSetCfb(_Msg30xx30p);
        break;
    case 1:
        Msg30xxSetCfb(_Msg30xx25p);
        break;
    case 3:
        Msg30xxSetCfb(_Msg30xx35p);
        break;
    case 4:
        Msg30xxSetCfb(_Msg30xx45p);
        break;
    case 5:
        Msg30xxSetCfb(_Msg30xx50p);
        break;
    }
    ///ADC: AFE Gain bypass
    mstar_set_reg_16bit_by_addr(0x1230, 0x3FFF, ADDRESS_MODE_16BIT);
}

void msg30xx_set_sensor_pad_state(u16 state)
{
    u16 value = 0;
    int i;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    for (i = 0; i < 8; i++)
        value |= (u16) (state << (i * 2));

    for (i = 0; i < 7; i++) {
        if (i == 6)
            value = (u16) (state & (BIT0 | BIT1));
        mstar_set_reg_16bit_by_addr(0x1514 + i, value, ADDRESS_MODE_16BIT);
    }

    for (i = 0; i < 4; i++) {
        if (i == 3)
            value = 0x0001;
        else
            value = 0xFFFF;
        mstar_set_reg_16bit_by_addr(0x1510 + i, value, ADDRESS_MODE_16BIT);
    }
}

void msg30xx_scan_set_sensor_num(u16 nSensorNum)    //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_reg_mask_16bit(0x1305, BIT0 | BIT1 | BIT2 | BIT3, nSensorNum, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x1102, BIT0 | BIT1 | BIT2 | BIT3, nSensorNum - 1, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x1006, BIT0 | BIT1 | BIT2 | BIT3, nSensorNum - 1, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x1017, BIT11 | BIT12 | BIT13 | BIT14 | BIT15,
                 (u16) (0x0C - abs(MAX_AFE_NUM_30XX - nSensorNum)) << 11, ADDRESS_MODE_16BIT);
}

void msg30xx_scan_set_subframe_num(u16 nSubframeNum)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_reg_mask_16bit(0x1305, BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13, nSubframeNum << 8, ADDRESS_MODE_16BIT);   // bit8-13=SF_NUM,bit0-3=SENSOR_NUM
    mstar_reg_mask_16bit(0x1101, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, nSubframeNum, ADDRESS_MODE_16BIT);    // bit0-5=6,6 subframe per scan
    mstar_reg_mask_16bit(0x100B, BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6, nSubframeNum << 1, ADDRESS_MODE_16BIT);   // bit1-6= subframe
}

void msg30xx_scan_set_sample_num(u16 nSampleNum)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_reg_mask_16bit(0x1304, 0x07FF, nSampleNum, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x1103, 0x1FFF, nSampleNum, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x100D, 0x07FF, nSampleNum, ADDRESS_MODE_16BIT);
    mstar_reg_mask_16bit(0x1B04, 0x07FF, nSampleNum, ADDRESS_MODE_16BIT);
}

void msg30xx_set_chip_swap_enable(u16 nSwapEnable)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    if (nSwapEnable == ENABLE)
        mstar_reg_mask_16bit(0x111F, BIT2, BIT2, ADDRESS_MODE_16BIT);
    else
        mstar_reg_mask_16bit(0x111F, BIT2, 0, ADDRESS_MODE_16BIT);
}

void msg30xx_adc_desp_invert_enable(u16 nEnable)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    if (nEnable == ENABLE)
        mstar_reg_mask_16bit(0x1210, BIT13, BIT13, ADDRESS_MODE_16BIT);
    else
        mstar_reg_mask_16bit(0x1210, BIT13, 0, ADDRESS_MODE_16BIT);
}

void msg30xx_set_mutual_csub_via_dbbus(s16 nCSub)   //201703xx
{
    u8 nBaseLen = 6;
    u16 nFilter = 0x3F;
    u16 nLastFilter = 0x000F;   // 6x 14 % 16
    u8 nBasePattern = 0;
    u8 nPattern;
    u16 n16BitsPattern;
    u16 nCSub16Bits[6] = { 0 };
    int i;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    for (i = 0; i < 6; i++) {
        if (i == 0)
            nPattern = nBasePattern;    //Patn => Pattern

        n16BitsPattern = (u16) ((nPattern & 0xF) << nBaseLen * 2) | (nPattern << nBaseLen) | nPattern;

        if (i == 5) {
            nCSub16Bits[i] = (u16) (n16BitsPattern & nLastFilter);
        } else {
            nCSub16Bits[i] = n16BitsPattern;
        }
        nPattern = (u8) ((n16BitsPattern >> 4) & nFilter);
    }

    mstar_set_reg_16bit_by_addr(0x211F, 0x3FFF, ADDRESS_MODE_16BIT);

    for (i = 0; i < 6; i++) {
        mstar_set_reg_16bit_by_addr(0x2148 + i, nCSub16Bits[i], ADDRESS_MODE_16BIT);
        mstar_set_reg_16bit_by_addr(0x214A + i, nCSub16Bits[i], ADDRESS_MODE_16BIT);
    }
}
