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
#include "mp_setting.h"

#define FILENAME_MAX 4096

extern u8 *g_platform_fw_inter_ver; // internal use firmware version for MStar
extern u8 *g_fw_cust_ver;   // customer firmware version
extern struct mutex g_mutex;
extern u32 g_mp_test;

MutualMpTest_t *mutual_mp_test_data;
MutualMpTestResult_t *mutual_mp_test_result;
const u8 *g_map_va_mutual = NULL;
u16 g_scan_mode = 0;
u16 g_code_type = 0;
u16 g_fout_base_addr = 0;
u16 g_two_dac_enable = 0;
u16 g_sqrt_en = 0;
struct mstar_tp_info tpinfo;

int mstar_mp_get_fw_ver_on_flash(void)
{
    uint8_t nChipType = 0;
    uint8_t *main_fw_ver = kmalloc(4, GFP_KERNEL);
    uint8_t *info_fw_ver = kmalloc(4, GFP_KERNEL);
    uint8_t *ret = kmalloc(21, GFP_KERNEL); //0001.0001 / 0001.0001
    uint16_t main_major, main_minor;
    uint16_t info_major, info_minor;
#ifdef BUILD_FUNCTION_APK
    jbyteArray result;
#else
    uint8_t result = 0;
#endif

    memset(main_fw_ver, 0, 4);
    memset(info_fw_ver, 0, 4);
    memset(ret, 0, 21);
    memset(&tpinfo, 0, sizeof(struct mstar_tp_info));

    TS_LOG_DEBUG("*** %s *** \n", __func__);

    mstar_mp_dbbus_enter();
    mstar_mp_stop_mcu();

    nChipType = mstar_get_reg_16bit(0x1ECC) & 0xFF;
    TS_LOG_INFO(":%s: ==== nChipType = 0x%x \n", __func__, nChipType);

    if (nChipType != CHIP_TYPE_MSG28XX && nChipType != CHIP_TYPE_MSG28XXA &&
            nChipType != CHIP_TYPE_ILI2117A && nChipType != CHIP_TYPE_ILI2118A)
    {
        nChipType = 0;
    }

    switch (nChipType) {

    case CHIP_TYPE_MSG28XX:
    case CHIP_TYPE_MSG28XXA:
        mstar_mp_read_flash(nChipType, 0x1fff4, EMEM_TYPE_MAIN_BLOCK, 4, main_fw_ver);
        mstar_mp_read_flash(nChipType, 0x07ec, EMEM_TYPE_INFO_BLOCK, 4, info_fw_ver);
        break;

    default:
        ret[0] = '5';
        ret[1] = 'A';
        ret[2] = 'A';
        ret[3] = '5';
        break;
    }

    mstar_mp_start_mcu();
    mstar_mp_exit_dbbus();

    main_major = (main_fw_ver[0] | main_fw_ver[1] << 8);
    main_minor = (main_fw_ver[2] | main_fw_ver[3] << 8);

    info_major = (info_fw_ver[0] | info_fw_ver[1] << 8);
    info_minor = (info_fw_ver[2] | info_fw_ver[3] << 8);

    TS_LOG_INFO("%d.%03d/%d.%03d\n", main_major, main_minor, info_major, info_minor);
    sprintf(ret, "%d.%03d/%d.%03d", main_major, main_minor, info_major, info_minor);
    TS_LOG_INFO("%d.%03d/%d.%03d\n", main_major, main_minor, info_major, info_minor);

#ifdef BUILD_FUNCTION_APK
    result = (*env)->NewByteArray(env, 21);
    (*env)->SetByteArrayRegion(env, result, 0, 21, ret);
#else
    tpinfo.ChipType = nChipType;
    sprintf(tpinfo.MainBlockFWVersion, "%d.%03d", main_major, main_minor);
    sprintf(tpinfo.InfoBlockFWVersion, "%d.%03d", info_major, info_minor);
    sprintf(tpinfo.PlatformVersion, "%s", g_platform_fw_inter_ver);
    sprintf(tpinfo.FwVersion, "%s", g_fw_cust_ver);
#endif

    kfree(main_fw_ver);
    kfree(info_fw_ver);
    kfree(ret);
    return result;
}

int mstar_mp_load_ini(char *pFilePath)
{
    int nSize = 0;
    int res;

    mutual_mp_test_data = (MutualMpTest_t *) kcalloc(1, sizeof(MutualMpTest_t), GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_data)) {
        TS_LOG_ERR("Failed to allocate mutual_mp_test_data mem \n");
        return -1;
    }

    mutual_mp_test_result = (MutualMpTestResult_t *) kcalloc(1, sizeof(MutualMpTestResult_t), GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_result)) {
        TS_LOG_ERR("Failed to allocate mutual_mp_test_result mem \n");
        return -1;
    }

    mutual_mp_test_data->UIConfig.bOpenTest = bOpenTest;
    mutual_mp_test_data->UIConfig.bShortTest = bShortTest;
    //mutual_mp_test_data->UIConfig.bWpTest = bWpTest;

    mutual_mp_test_data->sensorInfo.numDrv = numDrv;
    mutual_mp_test_data->sensorInfo.numSen = numSen;
    mutual_mp_test_data->sensorInfo.numKey = numKey;
    mutual_mp_test_data->sensorInfo.numKeyLine = numKeyLine;
    mutual_mp_test_data->sensorInfo.numGr = numGr;
    mutual_mp_test_data->Open_test_csub = Open_test_csub;
    mutual_mp_test_data->Open_test_cfb = Open_test_cfb;
    mutual_mp_test_data->Open_mode = Open_mode;
    mutual_mp_test_data->Open_fixed_carrier = Open_fixed_carrier;
    mutual_mp_test_data->Open_fixed_carrier1 = Open_fixed_carrier1;
    mutual_mp_test_data->Open_test_chargepump = Open_test_chargepump;
    mutual_mp_test_data->Mutual_Key = Mutual_Key;
    mutual_mp_test_data->Pattern_type = Pattern_type;
    mutual_mp_test_data->Pattern_model = Pattern_model;
    mutual_mp_test_data->ToastInfo.persentDC_VA_Range = persentDC_VA_Range;
    mutual_mp_test_data->ToastInfo.persentDC_VA_Range_up = persentDC_VA_Range_up;
    mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio = persentDC_VA_Ratio;
    mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio_up = persentDC_VA_Ratio_up;
    mutual_mp_test_data->ToastInfo.persentDC_Border_Ratio = persentDC_Border_Ratio;
    mutual_mp_test_data->deep_standby = deep_standby;
    mutual_mp_test_data->deep_standby_timeout = deep_standby_timeout;

    if ((mutual_mp_test_data->Mutual_Key == 1) || (mutual_mp_test_data->Mutual_Key == 2)) {
        mutual_mp_test_data->sensorInfo.KEY_CH = KEY_CH;
    }

    mutual_mp_test_data->Open_KeySettingByFW = Open_KeySettingByFW;
    mutual_mp_test_data->inverter_mode = inverter_mode;
    mutual_mp_test_data->OPEN_Charge = OPEN_Charge;
    mutual_mp_test_data->OPEN_Dump = OPEN_Dump;
    mutual_mp_test_data->SHORT_Charge = SHORT_Charge;
    mutual_mp_test_data->SHORT_Dump1 = SHORT_Dump1;

    mutual_mp_test_data->UIConfig.sSupportIC = kzalloc(256, GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_data->UIConfig.sSupportIC)) {
        TS_LOG_ERR("Failed to allocate mutual_mp_test_data->UIConfig.sSupportIC mem\n");
        return -1;
    }
    //memset(mutual_mp_test_data->UIConfig.sSupportIC, 0, FILENAME_MAX * sizeof(char));

    if (strlen(SupportIC) != 0)
        memcpy(mutual_mp_test_data->UIConfig.sSupportIC, SupportIC, sizeof(SupportIC));

    TS_LOG_INFO("SupportIC:      [%s]\n", mutual_mp_test_data->UIConfig.sSupportIC);

    mutual_mp_test_data->project_name = kzalloc(256, GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_data->project_name)) {
        TS_LOG_ERR("Failed to allocate mutual_mp_test_data->project_name mem\n");
        return -1;
    }
    //memset(mutual_mp_test_data->project_name, 0, FILENAME_MAX * sizeof(char));
    if (strlen(project_name) != 0)
        memcpy(mutual_mp_test_data->project_name, project_name, sizeof(project_name));

    TS_LOG_INFO("PROJECT:      [%s]\n", mutual_mp_test_data->project_name);

    mutual_mp_test_data->Goldensample_CH_0 = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_data->Goldensample_CH_0)) {
        TS_LOG_ERR("Failed to allocate mutual_mp_test_data->Goldensample_CH_0 mem\n");
        return -1;
    }
    memcpy(mutual_mp_test_data->Goldensample_CH_0, Goldensample_CH_0, sizeof(Goldensample_CH_0));

    mutual_mp_test_data->Goldensample_CH_0_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_data->Goldensample_CH_0_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_data->Goldensample_CH_0_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_data->Goldensample_CH_0_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_data->Goldensample_CH_0_Max)
        || ERR_ALLOC_MEM(mutual_mp_test_data->Goldensample_CH_0_Max_Avg)
        || ERR_ALLOC_MEM(mutual_mp_test_data->Goldensample_CH_0_Min)
        || ERR_ALLOC_MEM(mutual_mp_test_data->Goldensample_CH_0_Min_Avg)) {
        TS_LOG_ERR("Failed to allocate Goldensample mem\n");
        return -1;
    }

    if (mutual_mp_test_data->sensorInfo.numDrv && mutual_mp_test_data->sensorInfo.numSen) {
        mutual_mp_test_data->PAD2Drive =
            kcalloc(mutual_mp_test_data->sensorInfo.numDrv, sizeof(u16), GFP_KERNEL);
        mutual_mp_test_data->PAD2Sense =
            (u16 *) kcalloc(mutual_mp_test_data->sensorInfo.numSen, sizeof(u16), GFP_KERNEL);
        if (ERR_ALLOC_MEM(mutual_mp_test_data->PAD2Sense) || ERR_ALLOC_MEM(mutual_mp_test_data->PAD2Drive)) {
            TS_LOG_ERR("Failed to allocate PAD2Sense/PAD2Drive mem\n");
            return -1;
        }

        memcpy(mutual_mp_test_data->PAD2Drive, PAD2Drive, sizeof(PAD2Drive));
        memcpy(mutual_mp_test_data->PAD2Sense, PAD2Sense, sizeof(PAD2Sense));
    }

    if (mutual_mp_test_data->sensorInfo.numGr) {
        mutual_mp_test_data->PAD2GR =
            (u16 *) kmalloc(mutual_mp_test_data->sensorInfo.numGr * sizeof(u16), GFP_KERNEL);
        if (ERR_ALLOC_MEM(mutual_mp_test_data->PAD2GR)) {
            TS_LOG_ERR("Failed to allocate PAD2GR mem\n");
            return -1;
        }

        memcpy(mutual_mp_test_data->PAD2GR, PAD2GR, sizeof(PAD2GR));
    }

    mutual_mp_test_data->sensorInfo.thrsShort = thrsShort;
    TS_LOG_INFO("SHORTVALUE:      [%d]\n", mutual_mp_test_data->sensorInfo.thrsShort);

    mutual_mp_test_data->sensorInfo.thrsICpin = thrsICpin;
    TS_LOG_INFO("ICPINSHORT:      [%d]\n", mutual_mp_test_data->sensorInfo.thrsICpin);

    return 0;
}

void mstar_mp_calc_golden_range(int *goldenTbl, u16 weight, u16 weight_up, int *maxTbl, int *minTbl, int length)
{
    int i;
    int value = 0, value_up = 0;

    for (i = 0; i < length; i++) {
        value = (int)weight *abs(goldenTbl[i]) / 100;
        value_up = (int)weight_up *abs(goldenTbl[i]) / 100;

        maxTbl[i] = goldenTbl[i] + value + value_up;
        minTbl[i] = goldenTbl[i] - value;
    }
}

int mstar_mp_test_result(int nResultType, int *pResult)
{
    int nResultSize = 0;

    switch (nResultType) {

    case MPTEST_RESULT:
        pResult[OPEN_TEST_RESULT] = mutual_mp_test_result->nOpenResult;
        pResult[SHORT_TEST_RESULT] = mutual_mp_test_result->nShortResult;
        nResultSize = 3;
        break;

    case MPTEST_SCOPE:
        pResult[0] = mutual_mp_test_data->sensorInfo.numSen;
        pResult[1] = mutual_mp_test_data->sensorInfo.numDrv;
        pResult[2] = mutual_mp_test_data->sensorInfo.numKey;
        nResultSize = 3;
        break;

    case OPEN_TEST_DATA:
        pResult[0] = mutual_mp_test_data->sensorInfo.numDrv * mutual_mp_test_data->sensorInfo.numSen;
        memcpy(&pResult[1], mutual_mp_test_result->pOpenResultData, sizeof(int) * pResult[0]);
        nResultSize = pResult[0] + 1;
        break;

    case OPEN_TEST_FAIL_CHANNEL:
        pResult[0] = MAX_MUTUAL_NUM;
        memcpy(&pResult[1], mutual_mp_test_result->pOpenFailChannel, sizeof(int) * pResult[0]);
        nResultSize = pResult[0] + 1;
        break;

    case SHORT_TEST_DATA:
        break;

    case SHORT_TEST_FAIL_CHANNEL:
        break;

    default:
        break;
    }

    return nResultSize;
}

void mstar_mp_test_end(void)
{
    TS_LOG_INFO("*** %s() ***\n", __func__);

    if (mutual_mp_test_data->KeySen != NULL) {
        kfree(mutual_mp_test_data->KeySen);
    }

    if (mutual_mp_test_data->sensorInfo.key_type != NULL) {
        kfree(mutual_mp_test_data->sensorInfo.key_type);
    }

    if (mutual_mp_test_data->UIConfig.sSupportIC != NULL) {
        kfree(mutual_mp_test_data->UIConfig.sSupportIC);
    }

    if (mutual_mp_test_data->project_name != NULL) {
        kfree(mutual_mp_test_data->project_name);
    }

    if (mutual_mp_test_data->sensorInfo.key_type != NULL) {
        kfree(mutual_mp_test_data->sensorInfo.key_type);
    }

    if (mutual_mp_test_data->Goldensample_CH_0 != NULL) {
        kfree(mutual_mp_test_data->Goldensample_CH_0);
    }

    if (mutual_mp_test_data->Goldensample_CH_0_Max != NULL) {
        kfree(mutual_mp_test_data->Goldensample_CH_0_Max);
    }

    if (mutual_mp_test_data->Goldensample_CH_0_Max_Avg != NULL) {
        kfree(mutual_mp_test_data->Goldensample_CH_0_Max_Avg);
    }

    if (mutual_mp_test_data->Goldensample_CH_0_Min != NULL) {
        kfree(mutual_mp_test_data->Goldensample_CH_0_Min);
    }

    if (mutual_mp_test_data->Goldensample_CH_0_Min_Avg != NULL) {
        kfree(mutual_mp_test_data->Goldensample_CH_0_Min_Avg);
    }

    if (mutual_mp_test_data->PAD2Drive != NULL) {
        kfree(mutual_mp_test_data->PAD2Drive);
    }

    if (mutual_mp_test_data->PAD2Sense != NULL) {
        kfree(mutual_mp_test_data->PAD2Sense);
    }

    if (mutual_mp_test_data->PAD2GR != NULL) {
        kfree(mutual_mp_test_data->PAD2GR);
    }

    if (mutual_mp_test_result->pCheck_Fail != NULL) {
        kfree(mutual_mp_test_result->pCheck_Fail);
    }

    if (mutual_mp_test_result->pOpenResultData != NULL) {
        kfree(mutual_mp_test_result->pOpenResultData);
    }

    if (mutual_mp_test_result->pOpenFailChannel != NULL) {
        kfree(mutual_mp_test_result->pOpenFailChannel);
    }

    if (mutual_mp_test_result->pOpenRatioFailChannel != NULL) {
        kfree(mutual_mp_test_result->pOpenRatioFailChannel);
    }

    if (mutual_mp_test_result->pGolden_CH != NULL) {
        kfree(mutual_mp_test_result->pGolden_CH);
    }

    if (mutual_mp_test_result->pGolden_CH_Max != NULL) {
        kfree(mutual_mp_test_result->pGolden_CH_Max);
    }

    if (mutual_mp_test_result->pGolden_CH_Max_Avg != NULL) {
        kfree(mutual_mp_test_result->pGolden_CH_Max_Avg);
    }

    if (mutual_mp_test_result->pGolden_CH_Min != NULL) {
        kfree(mutual_mp_test_result->pGolden_CH_Min);
    }

    if (mutual_mp_test_result->pGolden_CH_Min_Avg != NULL) {
        kfree(mutual_mp_test_result->pGolden_CH_Min_Avg);
    }

    if (mutual_mp_test_result->pICPinChannel != NULL) {
        kfree(mutual_mp_test_result->pICPinChannel);
    }

    if (mutual_mp_test_result->pShortFailChannel != NULL) {
        kfree(mutual_mp_test_result->pShortFailChannel);
    }

    if (mutual_mp_test_result->pICPinShortFailChannel != NULL) {
        kfree(mutual_mp_test_result->pICPinShortFailChannel);
    }

    if (mutual_mp_test_result->pShortResultData != NULL) {
        kfree(mutual_mp_test_result->pShortResultData);
    }

    if (mutual_mp_test_result->pShortRData) {
        kfree(mutual_mp_test_result->pShortRData);
    }

    if (mutual_mp_test_result->pICPinShortResultData != NULL) {
        kfree(mutual_mp_test_result->pICPinShortResultData);
    }

    if (mutual_mp_test_result->pICPinShortRData != NULL) {
        kfree(mutual_mp_test_result->pICPinShortRData);
    }

    if (mutual_mp_test_data != NULL) {
        kfree(mutual_mp_test_data);
        mutual_mp_test_data = NULL;
    }

    if (mutual_mp_test_result != NULL) {
        kfree(mutual_mp_test_result);
        mutual_mp_test_result = NULL;
    }
}

u16 mstar_mp_get_fw_offical_ver_on_flash(void)
{
    u8 *info_fw_ver;
    int official_ver = 0;
    u8 nChipType = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    info_fw_ver = (u8 *) kcalloc(1, 4, GFP_KERNEL);
    //mstar_mp_dbbus_enter();
    mstar_mp_stop_mcu();

    nChipType = mstar_get_reg_16bit(0x1ECC) & 0xFF;
    mstar_mp_read_flash(nChipType, 0x7f8, EMEM_TYPE_INFO_BLOCK, 4, info_fw_ver);

    TS_LOG_INFO
        ("info_fw_ver[0] = 0x%02x, info_fw_ver[1] = 0x%02x, info_fw_ver[2] = 0x%02x, info_fw_ver[3] = 0x%02x\n",
         info_fw_ver[0], info_fw_ver[1], info_fw_ver[2], info_fw_ver[3]);

    info_fw_ver[1] = 0;

    official_ver = str_to_hex((char *)info_fw_ver);
    kfree(info_fw_ver);
    mstar_mp_start_mcu();
    return (u16) official_ver;
}

s16 mstar_mp_get_va_map(u16 nScanMode)
{
    s16 nRetVal = 0;
    u16 nDrvOpening = 0;
    nDrvOpening = msg30xx_get_drv_opening();

    TS_LOG_INFO("nDrvOpening = %d,g_two_dac_enable =%d\n", nDrvOpening, g_two_dac_enable);

    if (g_two_dac_enable == ENABLE) {
        switch (nDrvOpening) {
        case 4:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker4_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker4_Key_X[0][0];
            }
            break;
        case 5:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker5_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker5_Key_X[0][0];
            }
            break;
        case 6:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker6_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker6_Key_X[0][0];
            }
            break;
        case 7:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker7_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker7_Key_X[0][0];
            }
            break;
        case 8:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker8_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Two_Dac_Barker8_Key_X[0][0];
            }
            break;
        default:
            nRetVal = -1;
            break;
        }
    } else {
        switch (nDrvOpening) {
        case 4:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker4_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker4_Key_X[0][0];
            }
            break;
        case 5:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker5_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker5_Key_X[0][0];
            }
            break;
        case 6:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker6_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker6_Key_X[0][0];
            }
            break;
        case 7:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker7_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker7_Key_X[0][0];
            }
            break;
        case 8:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker8_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker8_Key_X[0][0];
            }
            break;
        case 9:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker9_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker9_Key_X[0][0];
            }
            break;
        case 10:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker10_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker10_Key_X[0][0];
            }
            break;
        case 11:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker11_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker11_Key_X[0][0];
            }
            break;
        case 12:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker12_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker12_Key_X[0][0];
            }
            break;
        case 13:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker13_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker13_Key_X[0][0];
            }
            break;
        case 14:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker14_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker14_Key_X[0][0];
            }
            break;
        case 15:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker15_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker15_Key_X[0][0];
            }
            break;
        case 16:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker16_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker16_Key_X[0][0];
            }
            break;
        case 17:
            if (nScanMode == KEY_SEPERATE) {
                g_map_va_mutual = &g_MapVaMutual_Barker17_X[0][0];
            } else {
                g_map_va_mutual = &g_MapVaMutual_Barker17_Key_X[0][0];
            }
            break;

        default:
            nRetVal = -1;
            break;
        }
    }
    return nRetVal;
}

int mstar_mp_init_vars(void)
{
    int i;
    int nMapSize;

    TS_LOG_INFO(" ------------%s--------\n", __func__);

    mstar_finger_touch_report_disable();
    if (mp_test_info.mode == 1) {
        mstar_dev_hw_reset();
        mstar_mp_dbbus_enter();

        // to get key combine or key seperate scan mode
        if (!mstar_enter_mp_mode_30xx()) {
            mdelay(100);
            mstar_mp_stop_mcu();
        } else {
            TS_LOG_INFO("Failed to enter MP mode \n");
            return -1;
        }
        TS_LOG_INFO("ScanMode = 0x%04X,, CodeType = 0x%04X,, TwoDAC = 0x%04X,\n", g_scan_mode, g_code_type, g_two_dac_enable);

        if (mstar_mp_get_va_map(g_scan_mode) < 0) {
            TS_LOG_INFO("ChooseVAMapping Fail \n");
            return -1;
        }
    }
    mstar_mp_calc_golden_range(mutual_mp_test_data->Goldensample_CH_0,
                   mutual_mp_test_data->ToastInfo.persentDC_VA_Range,
                   mutual_mp_test_data->ToastInfo.persentDC_VA_Range_up,
                   mutual_mp_test_data->Goldensample_CH_0_Max,
                   mutual_mp_test_data->Goldensample_CH_0_Min, MAX_MUTUAL_NUM);

    mutual_mp_test_result->pCheck_Fail = kcalloc(TEST_ITEM_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pOpenResultData = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pOpenFailChannel = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pOpenRatioFailChannel = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);

    if (ERR_ALLOC_MEM(mutual_mp_test_result->pCheck_Fail) || ERR_ALLOC_MEM(mutual_mp_test_result->pOpenResultData)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pOpenFailChannel)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pOpenRatioFailChannel)) {
        TS_LOG_ERR("Failed to allocate channels' mem\n");
        return -1;
    }

    mutual_mp_test_result->pGolden_CH = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pGolden_CH_Max = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pGolden_CH_Max_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pGolden_CH_Min = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pGolden_CH_Min_Avg = kcalloc(MAX_MUTUAL_NUM, sizeof(int), GFP_KERNEL);
    if (ERR_ALLOC_MEM(mutual_mp_test_result->pGolden_CH) || ERR_ALLOC_MEM(mutual_mp_test_result->pGolden_CH_Max) ||
        ERR_ALLOC_MEM(mutual_mp_test_result->pGolden_CH_Max_Avg)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pGolden_CH_Min)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pGolden_CH_Min_Avg)) {
        TS_LOG_ERR("Failed to allocate pGolden_CH' mem\n");
        return -1;
    }

    mutual_mp_test_result->nRatioAvg_max =
        (int)(100 + mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio +
          mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio_up) / 100;
    mutual_mp_test_result->nRatioAvg_min = (int)(100 - mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio) / 100;
    mutual_mp_test_result->nBorder_RatioAvg_max =
        (int)(100 + mutual_mp_test_data->ToastInfo.persentDC_Border_Ratio +
          mutual_mp_test_data->ToastInfo.persentDC_VA_Ratio_up) / 100;
    mutual_mp_test_result->nBorder_RatioAvg_min =
        (int)(100 - mutual_mp_test_data->ToastInfo.persentDC_Border_Ratio) / 100;

    mutual_mp_test_result->pShortFailChannel = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pShortResultData = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pICPinChannel = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pICPinShortFailChannel = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pICPinShortResultData = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);

    if (ERR_ALLOC_MEM(mutual_mp_test_result->pShortFailChannel)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pShortResultData)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pICPinChannel)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pICPinShortFailChannel)
        || ERR_ALLOC_MEM(mutual_mp_test_result->pICPinShortResultData)) {
        TS_LOG_ERR("Failed to allocate pShortFailChannel' mem\n");
        return -1;
    }

    mutual_mp_test_result->pICPinShortRData = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);
    mutual_mp_test_result->pShortRData = (int *)kcalloc(MAX_CHANNEL_NUM_30XX, sizeof(int), GFP_KERNEL);

    if (ERR_ALLOC_MEM(mutual_mp_test_result->pICPinShortRData) || ERR_ALLOC_MEM(mutual_mp_test_result->pShortRData)) {
        TS_LOG_ERR("Failed to allocate pICPinShortRData' mem\n");
        return -1;
    }

    return 0;
}

int mstar_mp_test_start(void)
{
    int i, res = 0;
    u16 fw_ver = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    mstar_finger_touch_report_disable();
    if (mp_test_info.mode == 1) {
        mstar_dev_hw_reset();
        mstar_mp_dbbus_enter();

        fw_ver = mstar_mp_get_fw_offical_ver_on_flash();
        if (fw_ver == 0xFF)
            fw_ver = 0;

        TS_LOG_INFO("info fw_ver = %x\n", fw_ver);
    }
    if (mstar_mp_init_vars() < 0) {
        res = -1;
        TS_LOG_INFO("mstar_mp_init_vars fail ***\n");
        goto out;
    }

    /* Open Test */
    if (mutual_mp_test_data->UIConfig.bOpenTest == 1) {
        mutual_mp_test_result->nOpenResult = mstar_mp_open_test(fw_ver);

        for (i = 0; i < MAX_MUTUAL_NUM; i++) {
            mutual_mp_test_result->pGolden_CH[i] = mutual_mp_test_data->Goldensample_CH_0[i];
            mutual_mp_test_result->pGolden_CH_Max[i] = mutual_mp_test_data->Goldensample_CH_0_Max[i];
            mutual_mp_test_result->pGolden_CH_Min[i] = mutual_mp_test_data->Goldensample_CH_0_Min[i];
        }
    } else {
        mutual_mp_test_result->nOpenResult = ITO_NO_TEST;
    }

    /* Short Test */
    if (mutual_mp_test_data->UIConfig.bShortTest == 1) {
        mutual_mp_test_result->nShortResult = mstar_mp_short_test_start();
    } else {
        mutual_mp_test_result->nShortResult = ITO_NO_TEST;
    }

    /* Return final result */
    if (mutual_mp_test_result->nOpenResult == ITO_NO_TEST) {
        res = mutual_mp_test_result->nShortResult;
    } else if (mutual_mp_test_result->nShortResult == ITO_NO_TEST) {
        res = mutual_mp_test_result->nOpenResult;
    } else {
        if (mutual_mp_test_result->nShortResult == ITO_TEST_OK &&
            mutual_mp_test_result->nOpenResult == ITO_TEST_OK)
            res = ITO_TEST_OK;
        else
            res = ITO_TEST_FAIL;
    }

    TS_LOG_INFO("Result(%d): Short = %d, Open = %d \n", res, mutual_mp_test_result->nShortResult,
            mutual_mp_test_result->nOpenResult);
out:
    mstar_dev_hw_reset();
    mdelay(300);
    mstar_finger_touch_report_enable();
    return res;
}

int mstar_mp_test_entry(int nChipType, char *pFilePath)
{
    int res = 0;

    TS_LOG_DEBUG("*** %s : Start running MP Test *** \n", __func__);

    mutex_lock(&g_mutex);
    g_mp_test = 1;
    mutex_unlock(&g_mutex);

    if (nChipType == CHIP_TYPE_ILI2117A || nChipType == CHIP_TYPE_ILI2118A || nChipType == CHIP_TYPE_MSG28XXA) {
        if (mstar_mp_load_ini(pFilePath) < 0) {
            TS_LOG_DEBUG("*** %s : 2117A/2117B failed to load ini *** \n", __func__);
            res = -1;
            goto out;
        }
        res = mstar_mp_test_start();
    } else {
        TS_LOG_DEBUG("*** %s : It doesn't support the MP test for this chip *** \n", __func__);
        res = -1;
        goto out;
    }

out:
    mutex_lock(&g_mutex);
    g_mp_test = 0;
    mutex_unlock(&g_mutex);
    return res;

}

void mstar_mp_test_entry_end(int nChipType)
{
    if (nChipType == CHIP_TYPE_ILI2117A || nChipType == CHIP_TYPE_ILI2118A || nChipType == CHIP_TYPE_MSG28XXA) {
        mstar_mp_test_end();
    }
}
