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

int g_deltac_buf[MAX_MUTUAL_NUM] = { 0 };
int g_deltac_va_buf[MAX_MUTUAL_NUM] = { 0 };

int *g_result_buf;
int g_sense_num = 0;
int g_drive_num = 0;
s16 g_raw_data_overlap_done[90][90] = { {0} };

extern const u8 *g_map_va_mutual;

void Msg30xxSetCfb(u8 Cfb)  //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    /// Setting Cfb
    switch (Cfb) {
    case _Msg30xx50p:   /// Cfb = 50p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0000, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, (u16) 0x0000, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x0, Rfb: 180kohm
        break;
    case _Msg30xx45p:   /// Cfb = 45p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0010, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, BIT8, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x1, Rfb: 225kohm
        break;
    case _Msg30xx35p:   /// Cfb = 35p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0020, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, BIT9, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;
    case _Msg30xx30p:   /// Cfb = 30p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0040, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, BIT9, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;
    case _Msg30xx25p:   /// Cfb = 25p
        mstar_reg_mask_16bit(0x1528, (u16) 0x0070, (u16) 0x0050, ADDRESS_MODE_16BIT);
        mstar_reg_mask_16bit(0x1523, (u16) 0x0700, BIT9, ADDRESS_MODE_16BIT);   /// 0x1523[10:8] = 0x2, Rfb: 300kohm
        break;
    default:
        break;
    }

}

void mstar_mp_calibrate_mutual_csub(s16 nCSub)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    msg30xx_set_mutual_csub_via_dbbus(nCSub);
}

u16 mstar_mp_ana_get_mutual_channel_num(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    return (mstar_get_reg_16bit(0x102E) & (BIT0 | BIT1 | BIT2 | BIT3));
}

u16 mstar_mp_ana_get_mutual_subframe_num(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    return ((mstar_get_reg_16bit(0x1216) & (BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13 | BIT14 | BIT15)) >> 8) + 1;
}

void mstar_mp_update_ana_charge_dump_setting(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    mstar_set_reg_16bit_by_addr(0x1018, 0x001F, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1019, 0x003f, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101a, 0x0028, ADDRESS_MODE_16BIT);    //post idle time in sub0
    mstar_set_reg_16bit_by_addr(0x101b, 0x0028, ADDRESS_MODE_16BIT);    //post idle time in sub1
    mstar_set_reg_16bit_by_addr(0x101c, 0x0028, ADDRESS_MODE_16BIT);    //post idle time in sub2
    mstar_set_reg_16bit_by_addr(0x101d, 0x0028, ADDRESS_MODE_16BIT);    //post idle time in sub3
    mstar_set_reg_16bit_by_addr(0x101e, 0x0028, ADDRESS_MODE_16BIT);    //post idle time in sub4
    mstar_set_reg_16bit_by_addr(0x101f, 0x0000, ADDRESS_MODE_16BIT);

    mstar_set_reg_16bit_by_addr(0x100d, 0x0020, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1103, 0x0020, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1104, 0x0020, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1302, 0x0020, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x136b, 0x10000 / 0x20, ADDRESS_MODE_16BIT);    // AFE_coef, set value by 0x10000 dividing register 0x100d value
    mstar_set_reg_16bit_by_addr(0x1b30, 0x0020, ADDRESS_MODE_16BIT);
}

s32 mstar_mp_get_deltac(s32 * pDeltaC, u16 drv_mode)
{
    s16 *pRawData = NULL;
    u16 nDrvPos = 0, nSenPos = 0, nShift = 0;
    u16 nSenNumBak = 0;
    u16 nDrvNumBak = 0;
    s16 i, j;
    u16 switch_dac = (g_two_dac_enable == ENABLE) ? 2 : 1;

    TS_LOG_INFO("*** %s(): switch_dac = %d ***\n", __func__, switch_dac);

    pRawData = (s16 *) kmalloc(sizeof(s16) * MAX_CHANNEL_SEN * 2 * MAX_CHANNEL_DRV * 8, GFP_KERNEL);
    if (ERR_ALLOC_MEM(pRawData)) {
        TS_LOG_ERR("Failed to allocate pRawData mem \n");
        return -1;
    }

    memset(pRawData, 0, sizeof(s16) * MAX_CHANNEL_SEN * 2 * MAX_CHANNEL_DRV * 8);
    memset(g_raw_data_overlap_done, 0, 90 * 90 * sizeof(s16));

    if (mstar_mp_get_mutual_one_shot_raw_iir(pRawData, &nSenNumBak, &nDrvNumBak, drv_mode) < 0) {
        TS_LOG_INFO("*** Msg30xx Open Test# GetMutualOneShotRawIIR failed! ***\n");
        return -1;
    }

    TS_LOG_DEBUG("*** Msg30xx Open Test# nSenNumBak=%d nDrvNumBak=%d ***\n", nSenNumBak, nDrvNumBak);

    for (i = 0; i < g_sense_num; i++) {
        for (j = 0; j < g_drive_num; j++) {
            g_raw_data_overlap_done[i][j] = UN_USE_SENSOR;
        }
    }

    for (i = 0; i < nSenNumBak * switch_dac; i++) {
        for (j = 0; j < nDrvNumBak; j++) {
            nShift = (u16) (i * nDrvNumBak + j);

            nDrvPos = *(g_map_va_mutual + nShift * 2 + 1);
            nSenPos = *(g_map_va_mutual + nShift * 2 + 0);

            if (nDrvPos >= g_drive_num || nSenPos >= g_sense_num)
                continue;

            if (nDrvPos != 0xFF && nSenPos != 0xFF) {
                g_raw_data_overlap_done[nSenPos][nDrvPos] = pRawData[i * MAX_CHANNEL_DRV + j];
            }
        }
    }

    for (i = 0; i < g_sense_num; i++) {
        for (j = 0; j < g_drive_num; j++) {
            nShift = (u16) (i * g_drive_num + j);
            pDeltaC[nShift] = (s32) g_raw_data_overlap_done[i][j];
        }
    }

    TS_LOG_INFO("*** Msg30xx Open Test# gDeltaC ***\n");
    mstar_mp_debug_show_array2(pDeltaC, g_sense_num * g_drive_num, -32, 10, g_sense_num);
    kfree(pRawData);
    return 0;
}

void mstar_mp_halsrame_enter_access_mode(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    // change to R2 mode
    mstar_reg_mask_16bit(0x2140, BIT8, BIT8, ADDRESS_MODE_16BIT);
    // SRAM using MCU clock
    mstar_reg_mask_16bit(0x1E11, BIT13, BIT13, ADDRESS_MODE_16BIT);
}

void mstar_halsram_exit_access_mode(void)
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);
    // change to R2 mode
    mstar_reg_mask_16bit(0x2140, BIT8, 0, ADDRESS_MODE_16BIT);
    // SRAM using MCU clock
    mstar_reg_mask_16bit(0x1E11, BIT13, 0, ADDRESS_MODE_16BIT);
}

void mstar_mp_write_dq_mem_8bit(u16 addr, u8 data)
{
    u8 read_buf[4] = { 0 };
    u16 high_16, low_16 = 0;
    u16 read_addr = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    read_addr = addr - (addr % 4);

    mstar_reg_get_16bit_byte_value_buf(read_addr, read_buf, sizeof(read_buf) / sizeof(read_buf[0]));

    read_buf[addr % 4] = data;
    TS_LOG_DEBUG("read_buf:0x%02x 0x%02x 0x%02x  0x%02x", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
    low_16 = (((u16) read_buf[1] << 8) | read_buf[0]);
    high_16 = (((u16) read_buf[3] << 8) | read_buf[2]);

    mstar_set_reg_16bit(read_addr + 2, low_16);
    mstar_set_reg_16bit(read_addr, high_16);
}

void mstar_mp_write_dq_mem_32bits(u16 addr, u32 data)
{
    u8 read_buf[4] = { 0 };
    u16 high_16, low_16 = 0;
    u16 read_addr = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    read_addr = addr - (addr % 4);

    mstar_reg_get_16bit_byte_value_buf(read_addr, read_buf, sizeof(read_buf) / sizeof(read_buf[0]));
    TS_LOG_DEBUG("read_buf[0] = 0x%02X, read_buf[1] = 0x%02X, read_buf[2] = 0x%02X, read_buf[3] = 0x%02X",
             read_buf[0], read_buf[1], read_buf[2], read_buf[3]);

    low_16 = (data & 0xFFFF);
    high_16 = (data >> 16);
    mstar_set_reg_16bit(read_addr + 2, low_16);
    mstar_set_reg_16bit(read_addr, high_16);
    mstar_reg_get_16bit_byte_value_buf(read_addr, read_buf, sizeof(read_buf) / sizeof(read_buf[0]));
    TS_LOG_DEBUG("read_buf[0] = 0x%02X, read_buf[1] = 0x%02X, read_buf[2] = 0x%02X, read_buf[3] = 0x%02X",
             read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
}

s32 mstar_obtain_open_value_keys_fw_v1007(s32 * pkeyarray)
{
    s32 k;
    u16 drv_mode = 0;
    u16 numKey = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (mstar_mp_get_deltac(g_deltac_va_buf, drv_mode) < 0) {
        TS_LOG_INFO("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
        return -1;
    }

    numKey = mutual_mp_test_data->sensorInfo.numKey;

    if (MAX(numKey, 3) > 3)
        numKey = 3;

    for (k = 0; k < numKey; k++) {
        pkeyarray[k] = g_deltac_va_buf[k];
        TS_LOG_INFO("+++ Dicky: pkeyarray[%d] = %d,  g_deltac_va_buf[%d] = %d \n", k, pkeyarray[k], k,
                g_deltac_va_buf[k]);
    }
    return 0;
}

s32 mstar_obtain_open_value_keys(s32 * pkeyarray)
{
    s32 k;
    u16 drv_mode = 0, numKey, drvNum, senNum;
    u32 shift = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (mstar_mp_get_deltac(g_deltac_va_buf, drv_mode) < 0) {
        TS_LOG_INFO("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
        return -1;
    }

    numKey = mutual_mp_test_data->sensorInfo.numKey;
    drvNum = mutual_mp_test_data->sensorInfo.numDrv;
    senNum = mutual_mp_test_data->sensorInfo.numSen;

    if (MAX(numKey, 3) > 3)
        numKey = 3;

    for (k = 0; k < numKey; k++) {
        shift =
            (mutual_mp_test_data->KeySen[k] - 1) * mutual_mp_test_data->sensorInfo.KeyDrv_o +
            mutual_mp_test_data->sensorInfo.KeyDrv_o - 1;
        pkeyarray[k] = g_deltac_va_buf[shift];
    }
    return 0;
}

s32 mstar_obtain_open_value_va_fw_v1007(void)
{
    u16 drv_mode = 0;
    if (mstar_mp_get_deltac(g_deltac_buf, drv_mode) < 0) {
        TS_LOG_ERR("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
        return -1;
    }
    return 0;
}

s32 mstar_obtain_open_value_va(void)
{
    s32 baseLen = 6, byteLen = 8, index = 0;
    u8 u8DrvData = 0;
    u32 uRegData32bits[4] = { 0 };
    u8 u8ShotData[16] = { 0 };
    u16 i, isf, nSf = 0, afe_opening = 0, dri_opening = 0, nDataLen = 0;
    u16 keySen = mutual_mp_test_data->PAD2Drive[mutual_mp_test_data->sensorInfo.KeyDrv_o - 1];
    u16 drv_mode = 0, regdata = 0;
    s32 addr = 0;       /// base address of FW mutual drive mapping.

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    regdata = mstar_get_reg_16bit_by_addr(0x1305, ADDRESS_MODE_16BIT);
    nSf = (regdata & (BIT8 | BIT9 | BIT10 | BIT11 | BIT12 | BIT13) >> 8);
    afe_opening = regdata & (BIT0 | BIT1 | BIT2 | BIT3);

    dri_opening = msg30xx_get_drv_opening();
    mstar_mp_halsrame_enter_access_mode();

    for (isf = 0; isf < nSf; isf++) {
        memset(u8ShotData, 0, sizeof(u8ShotData));
        if (dri_opening == 17) {
            addr = 0x7000;
            nDataLen = 16;
            index = nDataLen - 1;
        } else {
            addr = 0x7010;
            nDataLen = dri_opening;
            index = 0;
        }
        mstar_mp_ddbus_read_dq_mem_start();
        //RegGetXByteValue(u8ShotData, add + 0x20 * isf, sizeof(u8ShotData) / sizeof(u8ShotData[0]));
        mstar_get_reg_xbit(addr + 0x20 * isf, u8ShotData, sizeof(u8ShotData) / sizeof(u8ShotData[0]),
                   MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        mstar_mp_debug_show_array2(u8ShotData, 16, 16, 16, 16);
        // Find key index, then replace key sensor by assign sensor 56(0x38).
        for (i = 0; i < nDataLen; i++) {
            uRegData32bits[i / 4] |= (u32) u8ShotData[i] << (byteLen * (i % 4));
        }
        TS_LOG_DEBUG("Check KeySen : %d\n", keySen);
        for (i = index; i < nDataLen; i++) {
            u8DrvData = (u8) ((uRegData32bits[i / 4] >> (baseLen * (i % 4))) & 0x3F);
            TS_LOG_DEBUG("Driving PAD[%d] = %d\n", i, u8DrvData);
            if (u8DrvData == keySen) {
                uRegData32bits[i / 4] &= (u32) ~ (0x3F << (baseLen * (i % 4)));
                uRegData32bits[i / 4] |= (u32) (0x05 << (baseLen * (i % 4)));   //need discuss fout appear abnormal if overwrite key drive, 0x05 pad is temporary, it must be changed
                mstar_mp_write_dq_mem_32bits(addr + isf * 0x20 + i, uRegData32bits[i / 4]);
            }
        }

        mstar_mp_dbbus_read_dq_mem_end();
        mstar_mp_debug_show_array2(u8ShotData, 16, 16, 16, 16);
    }

    mstar_halsram_exit_access_mode();
    mstar_mp_ana_sw_reset();

    if (mstar_mp_get_deltac(g_deltac_buf, drv_mode) < 0) {
        TS_LOG_ERR("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
        return -1;
    }

    TS_LOG_DEBUG("*** ObtainOpenValue_VA End ***\n");
    return 0;
}

s32 mstar_mp_reenter_mutual_mode(u16 nFMode)
{
    u16 nRegData = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_start_mcu();
    mstar_mp_dbbus_re_enter();
    mdelay(50);
    mstar_set_reg_16bit(0x1402, nFMode);

    TS_LOG_DEBUG("nFMode = %x", nFMode);

    if (mstar_mp_check_switch_status() < 0) {
        TS_LOG_ERR("*** Msg30xx MP Test# CheckSwitchStatus failed! ***\n");
        return -1;
    }

    nRegData = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
    g_fout_base_addr = (u16) (nRegData << 2);

    TS_LOG_INFO("g_fout_base_addr = 0x%04x\n", g_fout_base_addr);
    mstar_mp_stop_mcu();
    mstar_set_reg_16bit(0x3D08, 0xFEFF);    //open timer
    return 0;
}

void mstar_mp_open_swcap_mode_setting(void)
{
    u16 chargeT = mutual_mp_test_data->OPEN_Charge, dumpT = mutual_mp_test_data->OPEN_Dump;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_stop_mcu();

    if (chargeT == 0 || dumpT == 0) {
        chargeT = 0x18;
        dumpT = 0x16;
    }

    mstar_mp_calibrate_mutual_csub(mutual_mp_test_data->Open_test_csub);
    msg30xx_gain_setting();
    mstar_mp_ana_change_cd_time(chargeT, dumpT);
    msg30xx_ana_enable_charge_pump(mutual_mp_test_data->Open_test_chargepump);
    msg30xx_Tgen_Ovwr_Ross_select(mutual_mp_test_data->Open_test_chargepump, ONE_DAC_ENABLE);
    msg30xx_Tgen_Ovwr_DrvL_Buf_gain_setting(mutual_mp_test_data->Open_test_chargepump, ONE_DAC_ENABLE);
    msg30xx_Tgen_Ovwr_DrvL_Buf_cfg_setting(mutual_mp_test_data->Open_test_chargepump, ONE_DAC_ENABLE);
    msg30xx_Tgen_Ovwr_invert_mode(mutual_mp_test_data->inverter_mode, ONE_DAC_ENABLE);
    msg30xx_scan_dac_setting(ONE_DAC_ENABLE);
    mstar_mp_ana_sw_reset();
}

void mstar_mp_open_sine_mode_setting(void)
{
    u8 nCfb = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    msg30xx_Tgen_Ovwr_reg_en(ENABLE);
    msg30xx_ana_enable_charge_pump(mutual_mp_test_data->Open_test_chargepump);
    msg30xx_Tgen_Ovwr_Ross_select(mutual_mp_test_data->Open_test_chargepump, g_two_dac_enable);
    msg30xx_Tgen_Ovwr_DrvL_Buf_gain_setting(mutual_mp_test_data->Open_test_chargepump, g_two_dac_enable);
    msg30xx_Tgen_Ovwr_DrvL_Buf_cfg_setting(mutual_mp_test_data->Open_test_chargepump, g_two_dac_enable);
    msg30xx_Tgen_Ovwr_invert_mode(mutual_mp_test_data->inverter_mode, g_two_dac_enable);

    if (!mutual_mp_test_data->inverter_mode)    // inverter mode == off must set DAC enable, otherwise it can't fetch fout
        msg30xx_scan_dac_setting(g_two_dac_enable);

    TS_LOG_INFO("Msg30xxSetCfb : %d, TwoDACEnable : %d", mutual_mp_test_data->Open_test_cfb, g_two_dac_enable);

    nCfb = mutual_mp_test_data->Open_test_cfb;
    switch (mutual_mp_test_data->Open_test_cfb) {
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
}

s32 mstar_mp_open_latter_fw_v1007(u16 nFMode)
{
    int *keyArray = NULL, k;
    u16 shift = 0;
    u16 drv_mode = 0;
    int ret = 0, numKey = 0;

    // Stop mcu
    mstar_mp_stop_mcu();
    TS_LOG_DEBUG("%s: nFMode = %x\n", __func__, nFMode);

    if (nFMode == MUTUAL_SINE)
        mstar_mp_open_sine_mode_setting();
    else
        mstar_mp_open_swcap_mode_setting();

    numKey = mutual_mp_test_data->sensorInfo.numKey;

    if (MAX(numKey, 3) > 3)
        numKey = 3;

    if (mutual_mp_test_data->Mutual_Key != 0) {

        u16 fmodeKey;
        if (nFMode == MUTUAL_SINE)
            fmodeKey = MUTUAL_SINE_KEY;
        else
            fmodeKey = MUTUAL_KEY;

        keyArray = (int *)kcalloc(numKey, sizeof(int), GFP_KERNEL);
        if (mstar_obtain_open_value_va_fw_v1007() < 0) {
            TS_LOG_ERR("*** ObtainOpenValue_VA failed ***\n");
            ret = -1;
            goto out;
        }

        if (mutual_mp_test_data->Open_KeySettingByFW == 0)  // Use our MP setting.
        {

            if (nFMode == MUTUAL_SINE)
                mstar_mp_open_sine_mode_setting();
            else
                mstar_mp_open_swcap_mode_setting();

            // We will setting Cfb = 50p and disable ChargePump when obtain BG of key by one shot.
            Msg30xxSetCfb(_Msg30xx50p);
            msg30xx_ana_enable_charge_pump(DISABLE);
        }

        if (mstar_mp_reenter_mutual_mode(fmodeKey) < 0) {
            TS_LOG_ERR("*** mstar_mp_reenter_mutual_mode failed ***\n");
            ret = -1;
            goto out;
        }

        if (mstar_obtain_open_value_keys_fw_v1007(keyArray) < 0) {
            TS_LOG_ERR("*** mstar_obtain_open_value_keys failed ***\n");
            ret = -1;
            goto out;
        }

        for (k = 0; k < numKey; k++) {
            shift =
                (mutual_mp_test_data->KeySen[k] - 1) * mutual_mp_test_data->sensorInfo.KeyDrv_o +
                mutual_mp_test_data->sensorInfo.KeyDrv_o - 1;
            g_deltac_buf[shift] = keyArray[k];
        }
    } else {
        if (mstar_mp_get_deltac(g_deltac_buf, drv_mode) < 0) {
            TS_LOG_ERR("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
            ret = -1;
            goto out;
        }
    }

out:
    if (keyArray != NULL) {
        TS_LOG_DEBUG("*** free keyArray ***\n");
        kfree(keyArray);
    }
    return ret;
}

s32 mstar_mp_previous_fw_v1007(u16 nFMode)
{
    s32 *keyArray = NULL, k;
    u16 shift = 0;
    u16 drv_mode = 0;
    int ret = 0, numKey = 0;

    TS_LOG_DEBUG("*** %s(): Mode = %x ***\n", __func__, nFMode);

    mstar_mp_stop_mcu();

    numKey = mutual_mp_test_data->sensorInfo.numKey;

    if (MAX(numKey, 3) > 3)
        numKey = 3;

    if (mutual_mp_test_data->Mutual_Key != 0) {

        if (mutual_mp_test_data->Open_mode == 2) {
            if (mstar_mp_open_latter_fw_v1007(nFMode) < 0) {
                TS_LOG_ERR("*** Msg30xx Open Test# OpenTest failed! ***\n");
                ret = -1;
                goto out;
            }
        } else {
            if ((mutual_mp_test_data->Pattern_type == 5) && (mutual_mp_test_data->Pattern_model == 1)) {
                if (mutual_mp_test_data->Open_KeySettingByFW) {
                    if (nFMode == MUTUAL_SINE)
                        mstar_mp_open_sine_mode_setting();
                    else
                        mstar_mp_open_swcap_mode_setting();

                    Msg30xxSetCfb(_Msg30xx50p);
                    msg30xx_ana_enable_charge_pump(DISABLE);
                }

                keyArray = (s32 *) kcalloc(numKey, sizeof(s32), GFP_KERNEL);
                if (mstar_obtain_open_value_keys(keyArray) < 0) {
                    TS_LOG_ERR("*** mstar_obtain_open_value_keys failed ***\n");
                    ret = -1;
                    goto out;
                }

                if (mstar_mp_reenter_mutual_mode(nFMode) < 0) {
                    TS_LOG_ERR("*** mstar_mp_reenter_mutual_mode failed ***\n");
                    ret = -1;
                    goto out;
                }

                if (nFMode == MUTUAL_SINE)
                    mstar_mp_open_sine_mode_setting();
                else
                    mstar_mp_open_swcap_mode_setting();

                if (mstar_obtain_open_value_va() < 0) {
                    TS_LOG_ERR("*** ObtainOpenValue_VA failed ***\n");
                    ret = -1;
                    goto out;
                }

                for (k = 0; k < numKey; k++) {
                    shift =
                        (mutual_mp_test_data->KeySen[k] -
                         1) * mutual_mp_test_data->sensorInfo.KeyDrv_o +
                        mutual_mp_test_data->sensorInfo.KeyDrv_o - 1;
                    g_deltac_buf[shift] = keyArray[k];
                }
            } else {
                keyArray = (s32 *) kcalloc(numKey, sizeof(s32), GFP_KERNEL);

                /* first shot - Get Keys    */
                if (mstar_mp_reenter_mutual_mode(0x6733) < 0) {
                    TS_LOG_ERR("*** mstar_mp_reenter_mutual_mode failed ***\n");
                    ret = -1;
                    goto out;
                }

                if (nFMode == MUTUAL_SINE)
                    mstar_mp_open_sine_mode_setting();
                else
                    mstar_mp_open_swcap_mode_setting();

                Msg30xxSetCfb(_Msg30xx50p);
                msg30xx_ana_enable_charge_pump(DISABLE);

                if (mstar_obtain_open_value_keys_fw_v1007(keyArray) < 0) {
                    TS_LOG_ERR("*** mstar_obtain_open_value_keys failed ***\n");
                    ret = -1;
                    goto out;
                }

                /* second shot - Get VA */
                if (mstar_mp_reenter_mutual_mode(nFMode) < 0) {
                    TS_LOG_ERR("*** mstar_mp_reenter_mutual_mode failed ***\n");
                    ret = -1;
                    goto out;
                }

                if (nFMode == MUTUAL_SINE)
                    mstar_mp_open_sine_mode_setting();
                else
                    mstar_mp_open_swcap_mode_setting();

                if (mstar_obtain_open_value_va_fw_v1007() < 0) {
                    TS_LOG_ERR("*** mstar_obtain_open_value_keys failed ***\n");
                    ret = -1;
                    goto out;
                }

                for (k = 0; k < numKey; k++) {
                    if (strcmp(mutual_mp_test_data->sensorInfo.key_type, "SENSE") == 0) {
                        shift =
                            (mutual_mp_test_data->sensorInfo.numSen -
                             1) * mutual_mp_test_data->sensorInfo.numDrv +
                            mutual_mp_test_data->KeySen[k] - 1;
                    } else if (strcmp(mutual_mp_test_data->sensorInfo.key_type, "DRIVE") == 0) {
                        shift =
                            (mutual_mp_test_data->KeySen[k] -
                             1) * mutual_mp_test_data->sensorInfo.KeyDrv_o +
                            mutual_mp_test_data->sensorInfo.KeyDrv_o - 1;
                    } else {
                        shift =
                            (mutual_mp_test_data->KeySen[k] -
                             1) * mutual_mp_test_data->sensorInfo.KeyDrv_o +
                            mutual_mp_test_data->sensorInfo.KeyDrv_o - 1;
                    }

                    g_deltac_buf[shift] = keyArray[k];
                }
            }
        }
    } else {
        if (nFMode == MUTUAL_SINE)
            mstar_mp_open_sine_mode_setting();
        else
            mstar_mp_open_swcap_mode_setting();

        if (mstar_mp_get_deltac(g_deltac_buf, drv_mode) < 0) {
            TS_LOG_ERR("*** Msg30xx Open Test# GetDeltaC failed! ***\n");
            ret = -1;
            goto out;
        }
    }

out:
    if (keyArray != NULL) {
        TS_LOG_DEBUG("*** free keyArray ***\n");
        kfree(keyArray);
    }
    return 0;
}

u16 g_normal_test_fail_short_check_deltac[MAX_MUTUAL_NUM];
u16 g_normal_test_fail_short_check_ratio[MAX_MUTUAL_NUM];
int g_ratio_board[MAX_MUTUAL_NUM];
int g_ratio_move[MAX_MUTUAL_NUM];
int g_ratio_border_move[MAX_MUTUAL_NUM];
int g_ratio[MAX_MUTUAL_NUM];

s32 mstar_mp_open_judge(u16 nItemID, s8 * pNormalTestResult, u16 * pNormalTestResultCheck)
{
    s32 nRetVal = 0;
    u16 nCSub = mutual_mp_test_data->Open_test_csub;
    u16 nRowNum = 0, nColumnNum = 0;
    u16 i, j, k;
    u16 nCfb;
    s32 bg_per_csub;
    int ratioAvg = 0, ratioAvg_max = 0, ratioAvg_min = 0, passCount = 0;
    int ratioAvg_border = 0, ratioAvg_border_max = 0, ratioAvg_border_min = 0, passCount1 = 0;
    int ratioAvg_move = 0, ratioAvg_border_move = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if ((mutual_mp_test_data->Open_mode == 1) || (mutual_mp_test_data->Open_mode == 2)) // if open mode = 1 (sine mode), Csub must be zero.
        nCSub = 0;

    if (!mutual_mp_test_data->Open_test_cfb)
        nCfb = 2;
    else
        nCfb = mutual_mp_test_data->Open_test_cfb;

    bg_per_csub = (int)(92012 / (11 * nCfb));

    for (i = 0; i < g_sense_num * g_drive_num; i++) {
        if (g_deltac_buf[i] > 31000) {
            return -1;
        }

    if(mp_test_info.mode == 2)//open test flow in fw
    {
        g_result_buf[i] = bg_per_csub * nCSub - g_deltac_buf[i];
    }
    else
    {
        if (g_deltac_buf[i] != UN_USE_SENSOR) {
            if (g_sqrt_en)
                g_result_buf[i] = g_deltac_buf[i];
            else
                g_result_buf[i] = bg_per_csub * nCSub - g_deltac_buf[i];
        } else
            g_result_buf[i] = NULL_DATA;
    }

        // For mutual key, last column if not be used, show number "one".
        if ((mutual_mp_test_data->Mutual_Key == 1 || mutual_mp_test_data->Mutual_Key == 2)
            && (mutual_mp_test_data->sensorInfo.numKey != 0)) {
            if (mutual_mp_test_data->Pattern_type == 5) {
                // KEY_CH = 1, it mean keys in same drive. Current one key project only KEY_CH = 1 type.
                if (mutual_mp_test_data->sensorInfo.KEY_CH != mutual_mp_test_data->sensorInfo.numKey) {
                    if (!((i + 1) % mutual_mp_test_data->sensorInfo.numDrv)) {
                        g_result_buf[i] = NULL_DATA;
                        for (k = 0; k < mutual_mp_test_data->sensorInfo.numKey; k++) {
                            if ((i + 1) / g_drive_num == mutual_mp_test_data->KeySen[k]) {
                                if (g_sqrt_en)
                                    g_result_buf[i] = g_deltac_buf[i];
                                else
                                    g_result_buf[i] =
                                        bg_per_csub * nCSub - g_deltac_buf[i];
                            }
                        }
                    }
                } else {
                    if (i >
                        ((mutual_mp_test_data->sensorInfo.numSen -
                          1) * mutual_mp_test_data->sensorInfo.numDrv - 1)) {
                        g_result_buf[i] = NULL_DATA;
                        for (k = 0; k < mutual_mp_test_data->sensorInfo.numKey; k++) {
                            if (((i + 1) - (g_sense_num - 1) * g_drive_num) ==
                                mutual_mp_test_data->KeySen[k]) {
                                if (g_sqrt_en)
                                    g_result_buf[i] = g_deltac_buf[i];
                                else
                                    g_result_buf[i] =
                                        bg_per_csub * nCSub - g_deltac_buf[i];
                            }
                        }
                    }
                }
            } else {
                if ((g_sense_num < g_drive_num) && ((i + 1) % g_drive_num == 0)) {
                    g_result_buf[i] = NULL_DATA;
                    for (k = 0; k < mutual_mp_test_data->sensorInfo.numKey; k++)
                        if ((i + 1) / g_drive_num == mutual_mp_test_data->KeySen[k]) {
                            if (g_sqrt_en)
                                g_result_buf[i] = g_deltac_buf[i];
                            else
                                g_result_buf[i] = bg_per_csub * nCSub - g_deltac_buf[i];
                        }
                }

                if ((g_sense_num > g_drive_num) && (i > (g_sense_num - 1) * g_drive_num - 1)) {
                    g_result_buf[i] = NULL_DATA;
                    for (k = 0; k < mutual_mp_test_data->sensorInfo.numKey; k++) {
                        if (((i + 1) - (g_sense_num - 1) * g_drive_num) ==
                            mutual_mp_test_data->KeySen[k]) {
                            if (g_sqrt_en)
                                g_result_buf[i] = g_deltac_buf[i];
                            else
                                g_result_buf[i] = bg_per_csub * nCSub - g_deltac_buf[i];
                        }
                    }
                }
            }
        }
    }

    memset(g_normal_test_fail_short_check_deltac, 0xFFFF, sizeof(g_normal_test_fail_short_check_deltac));
    memset(g_normal_test_fail_short_check_ratio, 0xFFFF, sizeof(g_normal_test_fail_short_check_ratio));
    memset(g_ratio, 0, sizeof(g_ratio));
    memset(g_ratio_board, 0, sizeof(g_ratio_board));
    memset(g_ratio_move, 0, sizeof(g_ratio_move));
    memset(g_ratio_border_move, 0, sizeof(g_ratio_move));

    nRowNum = g_drive_num;
    nColumnNum = g_sense_num;

    TS_LOG_DEBUG("*** Msg30xx Open Test# Show _gResult ***\n");
    mstar_mp_debug_show_array2(g_result_buf, nRowNum * nColumnNum, -32, 10, nColumnNum);
    TS_LOG_DEBUG("*** Msg30xx Open Test# Show Goldensample ***\n");
    mstar_mp_debug_show_array2(mutual_mp_test_data->Goldensample_CH_0, nRowNum * nColumnNum, -32, 10, nColumnNum);

    for (k = 0; k < (sizeof(g_deltac_buf) / sizeof(g_deltac_buf[0])); k++) {
        if (0 == mutual_mp_test_data->Goldensample_CH_0[k]) {
            if (k == 0)
                pNormalTestResult[0] = 1;   // no golden sample
            break;
        }

        if (g_result_buf[k] != NULL_DATA) {
            g_ratio[k] = (g_result_buf[k] * 1000) / mutual_mp_test_data->Goldensample_CH_0[k];

            if (0 ==
                mstar_check_value_in_range(g_result_buf[k], mutual_mp_test_data->Goldensample_CH_0_Max[k],
                               mutual_mp_test_data->Goldensample_CH_0_Min[k])) {
                pNormalTestResult[0] = 1;
                pNormalTestResultCheck[k] =
                    (u16) (((k / g_drive_num) + 1) * 100 + ((k % g_drive_num) + 1));
            } else {
                pNormalTestResultCheck[k] = PIN_NO_ERROR;
                if ((mutual_mp_test_data->Pattern_type == 3)
                    && (mutual_mp_test_data->sensorInfo.numKey == 0) && ((k % g_drive_num == 0)
                                             || ((k + 1) % g_drive_num ==
                                                 0))) {
                    ratioAvg_border += g_ratio[k];
                    passCount1 += 1;
                } else if ((mutual_mp_test_data->Pattern_type == 3)
                       && (mutual_mp_test_data->sensorInfo.numKey != 0) && ((k % g_drive_num == 0)
                                                || ((k + 2) %
                                                    g_drive_num ==
                                                    0))) {
                    ratioAvg_border += g_ratio[k];
                    passCount1 += 1;
                } else {
                    ratioAvg += g_ratio[k];
                    passCount += 1;
                }
            }
        } else {
            pNormalTestResultCheck[k] = PIN_NO_ERROR;
        }
        g_normal_test_fail_short_check_deltac[k] = pNormalTestResultCheck[k];
    }

    TS_LOG_DEBUG("*** Msg30xx Open Test# normalTestFail_check_Deltac Channel ***\n");

    ratioAvg_max =
        (int)(100000 + (mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000) +
          (mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_min = (int)(100000 - (mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio * 1000)) / 100;

    ratioAvg_border_max =
        (int)(100000 + (mutual_mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000) +
          (mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio_up * 1000)) / 100;
    ratioAvg_border_min = (int)(100000 - (mutual_mp_test_data->ToastInfo.persentDC_Border_Ratio * 1000)) / 100;

    if (passCount != 0) {
        if (passCount1 != 0) {
            ratioAvg_border_move = ratioAvg_border / passCount1;

            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(g_ratio) / sizeof(g_ratio[0]); i++) {
                if ((mutual_mp_test_data->sensorInfo.numKey == 0)
                    && ((i % g_drive_num == 0) || ((i + 1) % g_drive_num == 0))) {
                    g_ratio_move[i] = g_ratio[i] - ratioAvg_border_move + 1000;
                } else if ((mutual_mp_test_data->sensorInfo.numKey != 0)
                       && ((i % g_drive_num == 0) || ((i + 2) % g_drive_num == 0))) {
                    g_ratio_move[i] = g_ratio[i] - ratioAvg_border_move + 1000;
                } else {
                    g_ratio_move[i] = g_ratio[i] - ratioAvg_move + 1000;
                }

            }
        } else {
            ratioAvg_move = ratioAvg / passCount;

            for (i = 0; i < sizeof(g_ratio) / sizeof(g_ratio[0]); i++) {
                g_ratio_move[i] = g_ratio[i] - ratioAvg_move + 1000;
            }
        }
    } else {
        memcpy(g_ratio, g_ratio_move, sizeof(g_ratio));
    }

    for (j = 0; j < (sizeof(g_deltac_buf) / sizeof(g_deltac_buf[0])); j++) {
        if (0 == mutual_mp_test_data->Goldensample_CH_0[j]) {
            if (j == 0)
                pNormalTestResult[1] = 1;   // no golden sample
            break;
        }

        if (PIN_NO_ERROR == pNormalTestResultCheck[j]) {
            if (g_result_buf[j] != NULL_DATA) {
                if ((mutual_mp_test_data->Pattern_type == 3)
                    && (mutual_mp_test_data->sensorInfo.numKey == 0) && ((j % g_drive_num == 0)
                                             || ((j + 1) % g_drive_num ==
                                                 0))) {
                    if (0 ==
                        mstar_check_double_value_in_range(g_ratio_move[j], ratioAvg_border_max,
                                          ratioAvg_border_min)) {
                        pNormalTestResult[1] = 1;
                        pNormalTestResultCheck[j] =
                            (u16) (((j / g_drive_num) + 1) * 100 + ((j % g_drive_num) + 1));
                    } else {
                        pNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                } else if ((mutual_mp_test_data->Pattern_type == 3)
                       && (mutual_mp_test_data->sensorInfo.numKey != 0) && ((j % g_drive_num == 0)
                                                || ((j + 2) %
                                                    g_drive_num ==
                                                    0))) {
                    if (0 ==
                        mstar_check_double_value_in_range(g_ratio_move[j], ratioAvg_border_max,
                                          ratioAvg_border_min)) {
                        pNormalTestResult[1] = 1;
                        pNormalTestResultCheck[j] =
                            (u16) (((j / g_drive_num) + 1) * 100 + ((j % g_drive_num) + 1));
                    } else {
                        pNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                } else {
                    if (0 ==
                        mstar_check_double_value_in_range(g_ratio_move[j], ratioAvg_max,
                                          ratioAvg_min)) {
                        pNormalTestResult[1] = 1;
                        pNormalTestResultCheck[j] =
                            (u16) (((j / g_drive_num) + 1) * 100 + ((j % g_drive_num) + 1));
                    } else {
                        pNormalTestResultCheck[j] = PIN_NO_ERROR;
                    }
                }
            } else {
                pNormalTestResultCheck[j] = PIN_NO_ERROR;
            }
        } else {
            g_normal_test_fail_short_check_ratio[j] = pNormalTestResultCheck[j];
            continue;
        }
        g_normal_test_fail_short_check_ratio[j] = pNormalTestResultCheck[j];
    }

    TS_LOG_INFO("*** Msg30xx Open Test# normalTestFail_check_Ratio Channel ***\n");

    for (k = 0; k < MAX_MUTUAL_NUM; k++) {
        if (0 == mutual_mp_test_data->Goldensample_CH_0[k]) {
            pNormalTestResultCheck[k] = PIN_NO_ERROR;
            g_normal_test_fail_short_check_deltac[k] = PIN_NO_ERROR;
            g_normal_test_fail_short_check_ratio[k] = PIN_NO_ERROR;
        }
    }

    if ((pNormalTestResult[0] != 0) || (pNormalTestResult[1] != 0))
        nRetVal = -1;

    for (i = 0; i < 2; i++) {
        mutual_mp_test_result->pCheck_Fail[i] = pNormalTestResult[i];
    }
    for (i = 0; i < sizeof(g_normal_test_fail_short_check_deltac) / sizeof(g_normal_test_fail_short_check_deltac[0]); i++)  // reduce memory operation instead of memcpy
    {
        mutual_mp_test_result->pOpenFailChannel[i] = g_normal_test_fail_short_check_deltac[i];
        mutual_mp_test_result->pOpenRatioFailChannel[i] = g_normal_test_fail_short_check_ratio[i];
        mutual_mp_test_result->pGolden_CH_Max_Avg[i] = g_ratio_move[i];
    }

    return nRetVal;
}

int mstar_mp_fix_carrier_freq(u16 fmode, u8 Freq, u8 Freq1, u16 code_type)  //201703xx
{
    u8 nFreq = Freq, nFreq1 = Freq1, nDAC = 1;
    int i, nRet = 1, curFreq = 0, setFreq = 0;
    u16 nRegData = 0;
    u8 cmd[4] = { 0 };

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    nFreq = MAX(nFreq, 0);
    nFreq = MIN(nFreq, 255);
    cmd[0] = 0x0B;
    cmd[1] = 0x01;
    cmd[2] = nFreq;

    if (code_type == TWO_DAC_ENABLE) {
        nFreq1 = MAX(nFreq1, 0);
        nFreq1 = MIN(nFreq1, 255);
        cmd[3] = nFreq1;
        nDAC = 2;
    }

    mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &cmd[0], sizeof(cmd) / sizeof(u8));
    mstar_mp_reenter_mutual_mode(fmode);

    for (i = 0; i < nDAC; i++) {
        nRegData = mstar_get_reg_16bit_by_addr(0x2003 + i * 0x40, ADDRESS_MODE_16BIT);
        curFreq = (int)(nRegData * (13000000 / 16384) / 1000);  //khz
        setFreq = (i == 0) ? nFreq : nFreq1;

        if (abs(curFreq - setFreq) >= 2) {
            TS_LOG_INFO
                ("Fixed carrier DAC%d failed, current frequency = %d khz, need fixed frequency = %d khz", i,
                 curFreq, setFreq);
            return 0;
        }
    }
    return nRet;
}

u16 sine_method_detection(void)
{
    u16 nRegdata = 0;

    nRegdata = mstar_get_reg_16bit_by_addr(0x1301, ADDRESS_MODE_16BIT);
    return (((nRegdata & BIT7) == BIT7) ? 1 : 0);
}

u16 g_normal_test_result_check[MAX_MUTUAL_NUM] = { 0 }; //6:max subframe    13:max afe

int mstar_mp_open_test_entry(u16 fw_ver)
{
    u16 fmode = MUTUAL_MODE, i;
    s32 nRetVal = 0;
    s8 nNormalTestResult[2] = { 0 };    //0:golden    1:ratio
    u16 deep_standby = mutual_mp_test_data->deep_standby;

    TS_LOG_INFO("*** %s: MP Mode = %d ***\n", __func__, mp_test_info.mode);

    g_sense_num = mutual_mp_test_data->sensorInfo.numSen;
    g_drive_num = mutual_mp_test_data->sensorInfo.numDrv;
    g_result_buf = mutual_mp_test_result->pOpenResultData;

    /* Running New MP Test Flow if mode is 2, otherwise older one */
    if (mp_test_info.mode == 2) {
        nRetVal = new_flow_main(0);
        goto ITO_TEST_END;
    }

    mstar_finger_touch_report_disable();
    mstar_dev_hw_reset();
    mstar_mp_dbbus_enter();
    mdelay(100);

    // Stop mcu
    mstar_mp_stop_mcu();

    switch (mutual_mp_test_data->Open_mode) {
    case 0:
        fmode = MUTUAL_MODE;
        break;
    case 1:
    case 2:
        fmode = MUTUAL_SINE;
        break;
    }

    TS_LOG_INFO("fmode = %x\n", fmode);

    if (mstar_mp_switch_fw_mode(&fmode, &deep_standby) < 0) {
        TS_LOG_ERR("*** Msg30xx Open Test# SwitchFwMode failed! ***\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }

    if (fmode == MUTUAL_SINE) {
        if (!mstar_mp_fix_carrier_freq
            (fmode, mutual_mp_test_data->Open_fixed_carrier, mutual_mp_test_data->Open_fixed_carrier1,
             g_two_dac_enable)) {
            TS_LOG_ERR("*** Msg30xx Open Test# Set Fixed Carrier failed! ***\n");
            nRetVal = -1;
            goto ITO_TEST_END;
        }
    }

    g_sqrt_en = sine_method_detection();

    if (fw_ver == 0x0007) {
        if ((g_scan_mode != KEY_SEPERATE) && (g_scan_mode != KEY_COMBINE))
            g_scan_mode = KEY_SEPERATE;
    } else if (fw_ver < 0x0007) {
        g_scan_mode = KEY_COMBINE;
    }

    if (g_scan_mode == KEY_SEPERATE) {
        if (mstar_mp_open_latter_fw_v1007(fmode) < 0) {
            TS_LOG_ERR("*** Msg30xx Open Test# OpenTest failed! ***\n");
            nRetVal = -1;
            goto ITO_TEST_END;
        }
    } else {
        if (mstar_mp_previous_fw_v1007(fmode) < 0) {
            TS_LOG_ERR("*** Msg30xx Open Test# OpenTest failed! ***\n");
            nRetVal = -1;
            goto ITO_TEST_END;
        }
    }

    mdelay(10);

    memset(g_normal_test_result_check, 0, sizeof(g_normal_test_result_check));
    nRetVal = mstar_mp_open_judge(0, nNormalTestResult, g_normal_test_result_check);
    TS_LOG_INFO("*** Msg30xx Open Test# OpenTestOpenJudge return value = %d ***\n", nRetVal);

    mstar_mp_exit_dbbus();

ITO_TEST_END:

    for (i = 0; i < 2; i++) {
        mutual_mp_test_result->pCheck_Fail[i] = 1;
    }

    for (i = 0; i < MAX_MUTUAL_NUM; i++) {
        mutual_mp_test_result->pOpenFailChannel[i] = 0xFFFF;
        mutual_mp_test_result->pOpenRatioFailChannel[i] = 0xFFFF;
    }

    mstar_dev_hw_reset();
    mdelay(300);
    mstar_finger_touch_report_enable();
    return nRetVal;
}

int mstar_mp_open_test(u16 fw_ver)
{
    int nRetVal = 0;
    int nRet = 0;

    nRetVal = mstar_mp_open_test_entry(fw_ver);
    if (nRetVal == 0) {
        nRet = ITO_TEST_OK; //PASS
        TS_LOG_INFO("Msg30xx Open Test# MP test success\n");
    } else {
        if (nRetVal == -1) {
            nRet = ITO_TEST_FAIL;
        } else if (nRetVal == -2) {
            nRet = ITO_TEST_GET_TP_TYPE_ERROR;
        } else {
            nRet = ITO_TEST_UNDEFINED_ERROR;
        }

        TS_LOG_ERR("Msg30xx Open Test# MP test failed\n");
    }

    return nRet;
}
