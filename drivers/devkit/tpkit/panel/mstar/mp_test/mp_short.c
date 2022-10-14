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

extern int g_deltac_buf[MAX_MUTUAL_NUM];
extern int g_sense_num;
extern int g_drive_num;
int g_gr_num = 0;
int g_gr_test_pings[14] = { 0 };

u16 g_gr_pins[14];

u16 g_mux_mem_20_3E_0_settings[16] = { 0 };
u16 g_mux_mem_20_3E_1_settings[16] = { 0 };
u16 g_mux_mem_20_3E_2_settings[16] = { 0 };
u16 g_mux_mem_20_3E_3_settings[16] = { 0 };
u16 g_mux_mem_20_3E_4_settings[16] = { 0 };
u16 g_mux_mem_20_3E_5_settings[16] = { 0 };
u16 g_mux_mem_20_3E_6_settings[16] = { 0 };

            //-----------------0------4------8-----12-----16-----20-----24-----28-----32-----36-----40-----44-----48-----52-----56-----60,41pins
u16 MUX_MEM_3036_1_Settings[16] =
    { 0x0001, 0x0000, 0x0020, 0x0300, 0x4000, 0x0000, 0x0050, 0x0600, 0x0000, 0x0007, 0x0080, 0x0000, 0x0009, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3036_2_Settings[16] =
    { 0x0010, 0x0000, 0x0200, 0x3000, 0x0000, 0x0004, 0x0500, 0x0000, 0x0006, 0x0070, 0x0800, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3036_3_Settings[16] =
    { 0x0100, 0x0000, 0x2000, 0x0000, 0x0003, 0x0400, 0x5000, 0x0000, 0x0060, 0x0700, 0x8000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3036_4_Settings[16] =
    { 0x1000, 0x0000, 0x0000, 0x0002, 0x0030, 0x4000, 0x0000, 0x0005, 0x0600, 0x7000, 0x0000, 0x0008, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3036_5_Settings[16] =
    { 0x0000, 0x1000, 0x0000, 0x0020, 0x0300, 0x0000, 0x0004, 0x0050, 0x6000, 0x0000, 0x0007, 0x8000, 0x0000, 0x0000,
0x0000, 0x0000 };
            //-----------------0------4------8-----12-----16-----20-----24-----28-----32-----36-----40-----44-----48-----52-----56-----60,49pins
u16 MUX_MEM_3056_1_Settings[16] =
    { 0x0001, 0x0020, 0x0300, 0x4000, 0x0000, 0x0005, 0x0060, 0x0700, 0x8000, 0x0000, 0x0009, 0x00A0, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3056_2_Settings[16] =
    { 0x0010, 0x0200, 0x3000, 0x0000, 0x0004, 0x0050, 0x0600, 0x7000, 0x0000, 0x0008, 0x0090, 0x0A00, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3056_3_Settings[16] =
    { 0x0100, 0x2000, 0x0000, 0x0003, 0x0040, 0x0500, 0x6000, 0x0000, 0x0007, 0x0080, 0x0900, 0xA000, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3056_4_Settings[16] =
    { 0x1000, 0x0000, 0x0002, 0x0030, 0x0400, 0x5000, 0x0000, 0x0006, 0x0070, 0x0800, 0x9000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000 };
u16 MUX_MEM_3056_5_Settings[16] =
    { 0x0000, 0x0001, 0x0020, 0x0300, 0x4000, 0x0000, 0x0005, 0x0060, 0x0700, 0x8000, 0x0000, 0x0009, 0x000A, 0x0000,
0x0000, 0x0000 };

            //--------------0--1--2--3--4--5--6--7--8--9-10-11-12-13-14-15-16-17-18-19-20-21-22-23-24-25-26-27-28-29
u16 sensepad_pin_mapping_3036[60] =
    { 8, 9, 10, 11, 0, 0, 0, 12, 0, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 0, 25, 26, 27, 28, 29, 30, 31, 32,
    //-------------30-31-32-33-34-35-36-37-38-39-40-41-42-43-44-45-46-47-48-49-50-51-52-53-54-55-56-57-58-59
    33, 0, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 0, 0, 47, 48, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

            //--------------0--1--2--3--4--5--6--7--8--9-10-11-12-13-14-15-16-17-18-19-20-21-22-23-24-25-26-27-28-29
u16 sensepad_pin_mapping_3056[60] =
    { 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
37,
    //-------------30-31-32-33-34-35-36-37-38-39-40-41-42-43-44-45-46-47-48-49-50-51-52-53-54-55-56-57-58-59
    38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

void mstar_mp_set_noise_sensor_mode(u8 nEnable) //201703xx
{
    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (nEnable == ENABLE) {
        //mstar_set_reg_16bit_by_addr(0x1523, BIT4, ADDRESS_MODE_16BIT);
        msg30xx_set_mutual_csub_via_dbbus(0);
    }
}

void mstar_mp_change_ana_setting(void)  //201703xx
{
    int i, nMappingItem;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    for (nMappingItem = 0; nMappingItem < 6; nMappingItem++) {
        /// sensor mux sram read/write base address / write length
        mstar_set_reg_16bit_by_addr(0x2140, 0x00, ADDRESS_MODE_16BIT);  //select sram pad mux
        mstar_set_reg_16bit_by_addr(0x2148, 0x00, ADDRESS_MODE_16BIT);  //disable sensor mux cen overwrite
        mstar_set_reg_16bit_by_addr(0x2101, BIT0, ADDRESS_MODE_16BIT);  //reg_mem_soft_rst
        mstar_set_reg_16bit_by_addr(0x2101, 0x00, ADDRESS_MODE_16BIT);
        mstar_set_reg_16bit_by_addr(0x2142, 0x07, ADDRESS_MODE_16BIT);  //sensor mux sram write length
        mstar_set_reg_16bit_by_addr(0x2141, 0x07 * nMappingItem, ADDRESS_MODE_16BIT);
        mstar_set_reg_16bit_by_addr(0x2143, BIT0, ADDRESS_MODE_16BIT);  //sram write start

        for (i = 0; i < 7; i++) {
            if (nMappingItem == 0) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_1_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_1_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_1_settings[%d] = 0x%04x, g_mux_mem_20_3E_1_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_1_settings[2 * i], 2*i+1, g_mux_mem_20_3E_1_settings[2 * i + 1]);
            }
            if (nMappingItem == 1) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_2_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_2_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_2_settings[%d] = 0x%04x, g_mux_mem_20_3E_2_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_2_settings[2 * i], 2*i+1, g_mux_mem_20_3E_2_settings[2 * i + 1]);
            }
            if (nMappingItem == 2) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_3_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_3_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_3_settings[%d] = 0x%04x, g_mux_mem_20_3E_3_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_3_settings[2 * i], 2*i+1, g_mux_mem_20_3E_3_settings[2 * i + 1]);
            }
            if (nMappingItem == 3) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_4_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_4_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_4_settings[%d] = 0x%04x, g_mux_mem_20_3E_4_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_4_settings[2 * i], 2*i+1, g_mux_mem_20_3E_4_settings[2 * i + 1]);
            }
            if (nMappingItem == 4) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_5_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_5_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_5_settings[%d] = 0x%04x, g_mux_mem_20_3E_5_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_5_settings[2 * i], 2*i+1, g_mux_mem_20_3E_5_settings[2 * i + 1]);
            }
            if (nMappingItem == 5) {
                mstar_set_reg_16bit_by_addr(0x2144, g_mux_mem_20_3E_6_settings[2 * i],
                                ADDRESS_MODE_16BIT);
                mstar_set_reg_16bit_by_addr(0x2145, g_mux_mem_20_3E_6_settings[2 * i + 1],
                                ADDRESS_MODE_16BIT);
                //TS_LOG_DEBUG("g_mux_mem_20_3E_6_settings[%d] = 0x%04x, g_mux_mem_20_3E_6_settings[%d] = 0x%04x", 2*i, g_mux_mem_20_3E_6_settings[2 * i], 2*i+1, g_mux_mem_20_3E_6_settings[2 * i + 1]);
            }
        }
    }
}

void mstar_mp_ana_load_setting(void)
{
    // Stop mcu
    mstar_mp_stop_mcu();
    mstar_mp_change_ana_setting();
}

void mstar_mp_read_settings(void)
{
    int i, nSize, j;
    //char str[512] = { 0 };

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    for (i = 1; i < 6; i++) {
        nSize = 0;
        if (i == 1) {
            memcpy(g_mux_mem_20_3E_1_settings, N1_MUX_MEM_20_3E, sizeof(N1_MUX_MEM_20_3E));
            for (j = 0; j < 16; j++)
                TS_LOG_DEBUG("0x%04x,", g_mux_mem_20_3E_1_settings[j]);
            TS_LOG_DEBUG("SHORT_TEST_N1\n");
            mstar_mp_debug_show_array2(g_mux_mem_20_3E_1_settings, 16, 16, 16, 16);
        } else if (i == 2) {
            memcpy(g_mux_mem_20_3E_1_settings, N2_MUX_MEM_20_3E, sizeof(N2_MUX_MEM_20_3E));
            TS_LOG_DEBUG("SHORT_TEST_N2\n");
            mstar_mp_debug_show_array2(g_mux_mem_20_3E_2_settings, 16, 16, 16, 16);
        } else if (i == 3) {
            memcpy(g_mux_mem_20_3E_1_settings, S1_MUX_MEM_20_3E, sizeof(S1_MUX_MEM_20_3E));
            TS_LOG_DEBUG("SHORT_TEST_S1\n");
            mstar_mp_debug_show_array2(g_mux_mem_20_3E_3_settings, 16, 16, 16, 16);
        } else if (i == 4) {
            memcpy(g_mux_mem_20_3E_1_settings, S2_MUX_MEM_20_3E, sizeof(S2_MUX_MEM_20_3E));
            TS_LOG_DEBUG("SHORT_TEST_S2\n");
            mstar_mp_debug_show_array2(g_mux_mem_20_3E_4_settings, 16, 16, 16, 16);
        } else if (i == 5) {
            // if (IniGetU16Array("SHORT_TEST_N3","MUX_MEM_20_3E", g_mux_mem_20_3E_5_settings)) { ///shortN-3
            //      //TS_LOG_DEBUG("SHORT_TEST_N3");
            //      //mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
            //      return;
            // }
            // if (IniGetU16Array("SHORT_TEST_S3","MUX_MEM_20_3E", g_mux_mem_20_3E_5_settings)) { ///shortS-3
            //      //TS_LOG_DEBUG("SHORT_TEST_S3");
            //      //mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
            //      return;
            // }
            // if (IniGetU16Array("SHORT_TEST_GR","MUX_MEM_20_3E", g_mux_mem_20_3E_5_settings)) {
            //      //TS_LOG_DEBUG("SHORT_TEST_GR");
            //      //mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
            //      return;
            // }
#if 0
            if (ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
                if (ms_ini_split_u16_array(str, g_mux_mem_20_3E_5_settings)) {  ///shortN-3
                    TS_LOG_DEBUG("SHORT_TEST_N3\n");
                    mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
                    return;
                }
            }

            if (ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
                if (ms_ini_split_u16_array(str, g_mux_mem_20_3E_5_settings)) {  ///shortS-3
                    TS_LOG_DEBUG("SHORT_TEST_S3\n");
                    mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
                    return;
                }
            }

            if (ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
                if (ms_ini_split_u16_array(str, g_mux_mem_20_3E_5_settings)) {
                    TS_LOG_DEBUG("SHORT_TEST_GR\n");
                    mstar_mp_debug_show_array2(g_mux_mem_20_3E_5_settings, 16, 16, 16, 16);
                    TS_LOG_DEBUG("SHORT_TEST_GR end \n");
                    return;
                }
            }
#endif
        }
    }
}

s32 mstar_mp_get_valueR(s32 * pTarget, u16 drv_mode)    //201703xx
{
    s16 *pRawData = NULL;
    u16 nSenNumBak = 0;
    u16 nDrvNumBak = 0;
    u16 nShift = 0;
    s16 i, j;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    if (pTarget == NULL) {
        TS_LOG_ERR("pTarget's mem is null \n");
        return -1;
    }

    pRawData = (s16 *) kcalloc(MAX_MUTUAL_NUM, sizeof(s16), GFP_KERNEL);
    if (ERR_ALLOC_MEM(pRawData)) {
        TS_LOG_ERR("Failed to allocate pRawData mem \n");
        return -1;
    }

    if (mstar_mp_get_mutual_one_shot_raw_iir(pRawData, &nSenNumBak, &nDrvNumBak, drv_mode) < 0) {
        TS_LOG_ERR("*** Msg30xx Short Test# GetMutualOneShotRawIIR failed! ***\n");
        return -1;
    }

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 14; j++) {
            nShift = (u16) (j + 14 * i);
            pTarget[nShift] = pRawData[j + 14 * i];
        }
    }
    kfree(pRawData);
    return 0;
}

u16 mstar_mp_read_test_pins(u16 itemID, int *testPins)  //201703xx
{
    u16 nSize = 0;
    int i = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    switch (itemID) {
    case 0:
    case 10:
        //nSize = IniGetIntArray("SHORT_TEST_N1", "TEST_PIN", testPins);
        //ms_getInidata("SHORT_TEST_N1","TEST_PIN",str);
        //nSize = ms_ini_split_int_array(N1_TEST_PIN, testPins);
        memcpy(testPins, N1_TEST_PIN, sizeof(N1_TEST_PIN));
        nSize = sizeof(N1_TEST_PIN) / sizeof(int);
        TS_LOG_DEBUG("SHORT_TEST_N1 nSize = %d\n", nSize);
        for (i = 0; i < 13; i++) {
            TS_LOG_DEBUG("%d,", testPins[i]);
        }
        TS_LOG_DEBUG("\n");
        break;
    case 1:
    case 11:
        nSize = sizeof(N2_TEST_PIN) / sizeof(int);
        memcpy(testPins, N2_TEST_PIN, sizeof(N2_TEST_PIN));
        TS_LOG_DEBUG("SHORT_TEST_N2 nSize = %d\n", nSize);
        for (i = 0; i < 12; i++) {
            TS_LOG_DEBUG("%d,", testPins[i]);
        }
        TS_LOG_DEBUG("\n");
        break;
    case 2:
    case 12:
        nSize = sizeof(S1_TEST_PIN) / sizeof(int);
        memcpy(testPins, S1_TEST_PIN, sizeof(S1_TEST_PIN));
        for (i = 0; i < 7; i++) {
            TS_LOG_DEBUG("%d,", testPins[i]);
        }
        TS_LOG_DEBUG("\n");
        break;
    case 3:
    case 13:
        nSize = sizeof(S2_TEST_PIN) / sizeof(int);
        memcpy(testPins, S2_TEST_PIN, sizeof(S2_TEST_PIN));
        TS_LOG_DEBUG("SHORT_TEST_S2 nSize = %d\n", nSize);
        for (i = 0; i < 6; i++) {
            TS_LOG_DEBUG("%d,", testPins[i]);
        }
        TS_LOG_DEBUG("\n");
        break;

    case 4:
    case 14:
#if 0
        if (ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str) != 0) {
            ms_getInidata("SHORT_TEST_N3", "TEST_PIN", str);
            nSize = ms_ini_split_int_array(str, testPins);
            TS_LOG_DEBUG("SHORT_TEST_N3 nSize = %d\n", nSize);
        } else if (ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str) != 0) {
            ms_getInidata("SHORT_TEST_S3", "TEST_PIN", str);
            nSize = ms_ini_split_int_array(str, testPins);
            TS_LOG_DEBUG("SHORT_TEST_S3 nSize = %d\n", nSize);
        } else if (ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str) != 0) {
            if (mutual_mp_test_data->sensorInfo.numGr == 0)
                nSize = 0;
            else
                nSize = _gMsg30xxGRSize;

            for (i = 0; i < sizeof(g_gr_test_pings) / sizeof(g_gr_test_pings[0]); i++)
                testPins[i] = g_gr_test_pings[i];
            TS_LOG_DEBUG("SHORT_TEST_GR nSize = %d\n", nSize);
        }
#endif
        break;

    default:
        return 0;
    }

    for (i = nSize; i < MAX_CHANNEL_NUM_30XX; i++) {
        testPins[i] = PIN_NO_ERROR;
    }

    return nSize;
}

s32 mstar_mp_short_test_judge(u8 nItemID, s8 * TestFail, u16 TestFail_check[][MAX_MUTUAL_NUM])  //201703xx
{
    int nRet = 1, i, count_test_pin = 0;
    int testPins[MAX_CHANNEL_NUM_30XX] = { 0 };

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_read_test_pins(nItemID, testPins);

    for (i = 0; i < sizeof(testPins) / sizeof(testPins[0]); i++) {
        if (testPins[i] != PIN_NO_ERROR)
            count_test_pin++;
    }

    for (i = 0; i < count_test_pin; i++) {
        TestFail_check[nItemID][i] = testPins[i];
        TS_LOG_DEBUG("testPins[%d]=%d\n", i, testPins[i]);
        if (0 ==
            mstar_check_value_in_range(g_deltac_buf[i + nItemID * 14],
                           mutual_mp_test_data->sensorInfo.thrsShort,
                           -mutual_mp_test_data->sensorInfo.thrsShort)) {
            TestFail[nItemID] = 1;
            nRet = 0;
        }
    }

    return nRet;
}

int mstar_mp_convert_R_value(int dump_time, s32 deltaR)
{
    s32 result = 0;

    if (deltaR >= IIR_MAX)
        return 0;
    if (deltaR == 0)
        deltaR = 1;

    if(deltaR > -1000) {
        result = (int)(73318 * dump_time) / (55 * (abs(deltaR) - 0));
    } else {
        result = (int)(53248 * dump_time) / (55 * ((deltaR) - 0));
    }

    return result;
}

void mstar_mp_scan_shift_code_init_rx_code(u16 codeLength, u16 scantag)
{
    u16 i;
    u16 RegData = 0;

    u16 RxShiftCodeCoef_m4[] = { 0x1000, 0x1000, 0x1000, 0x3000, 0x1000, 0x1000, 0x1000, 0x1000,    // codelength 4
        0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m5[] = { 0x0e8b, 0x02e8, 0x1a2e, 0x2ba2, 0x08ba, 0x1000, 0x1000, 0x1000,    // codelength 5
        0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m6[] = { 0x3b6d, 0x1249, 0x16db, 0x2492, 0x0db6, 0x0924, 0x1000, 0x1000,    // codelength 6
        0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m7[] = { 0x1000, 0x1000, 0x1000, 0x0000, 0x0000, 0x1000, 0x0000, 0x1000,    // codelength 7
        0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m8[] = { 0x071c, 0x1555, 0x071c, 0x071c, 0x38e3, 0x1555, 0x38e3, 0x0e38,    // codelength 8
        0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m9[] = { 0x0dc7, 0x0c72, 0x0ea4, 0x3e67, 0x3e1f, 0x109d, 0x3cf5, 0x07b7,    // codelength 9
        0x0550, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m10[] = { 0x064f, 0x13d4, 0x0736, 0x02b4, 0x38c9, 0x13d4, 0x39b0, 0x0ad1,   // codelength 10
        0x0000, 0x0ad1, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m11[] = { 0x1555, 0x1555, 0x1555, 0x0000, 0x1555, 0x1555, 0x0000, 0x1555,   // codelength 11
        0x0000, 0x0000, 0x0000, 0x1000, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m12[] = { 0x06c6, 0x1046, 0x00c0, 0x036a, 0x06c6, 0x0fb2, 0x3939, 0x0d43,   // codelength 12
        0x3f3f, 0x066d, 0x3939, 0x08eb, 0x1000, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m13[] = { 0x1127, 0x0d8b, 0x02a3, 0x370a, 0x0b88, 0x117a, 0x3ed5, 0x02a8,   // codelength 13
        0x3fb8, 0x09f0, 0x0179, 0x3da6, 0x0053, 0x1000, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m14[] = { 0x05c1, 0x114f, 0x02a9, 0x3c4f, 0x3ff6, 0x10b5, 0x01fe, 0x0e1f,   // codelength 14
        0x381b, 0x022f, 0x3fec, 0x0ae5, 0x3d98, 0x0676, 0x1000, 0x1000
    };
    u16 RxShiftCodeCoef_m15[] = { 0x1000, 0x1000, 0x1000, 0x1000, 0x0000, 0x0000, 0x0000, 0x1000,   // codelength 15
        0x0000, 0x0000, 0x1000, 0x1000, 0x0000, 0x1000, 0x0000, 0x1000
    };
    u16 RxShiftCodeCoef_m16[] = { 0x0b65, 0x3f86, 0x121f, 0x3cb5, 0x0870, 0x3901, 0x052d, 0x079c,   // codelength 16
        0x3bcb, 0x057f, 0x3ccc, 0x11d6, 0x383f, 0x0dc9, 0x3d9f, 0x046d
    };
    u16 RxShiftCodeCoef_m17[] = { 0x0d27, 0x1175, 0x0f91, 0x0ccc, 0x3a71, 0x02b4, 0x01dd, 0x10aa,   // codelength 17
        0x0180, 0x3691, 0x0be3, 0x0c41, 0x024b, 0x1343, 0x3dad, 0x0b0c
    };

    if (codeLength == 17)
        mstar_set_reg_16bit_by_addr(0x132F, 0x06d6, ADDRESS_MODE_16BIT);
    else
        mstar_set_reg_16bit_by_addr(0x132F, 0x1000, ADDRESS_MODE_16BIT);    //reg_shift_code_coef_16

    if (scantag == MULTI) {
        u16 coefMatrix[16] = { 0 };
        switch (codeLength) {
        case 4:
            memcpy(coefMatrix, RxShiftCodeCoef_m4, sizeof(coefMatrix));
            break;
        case 5:
            memcpy(coefMatrix, RxShiftCodeCoef_m5, sizeof(coefMatrix));
            break;
        case 6:
            memcpy(coefMatrix, RxShiftCodeCoef_m6, sizeof(coefMatrix));
            break;
        case 7:
            memcpy(coefMatrix, RxShiftCodeCoef_m7, sizeof(coefMatrix));
            break;
        case 8:
            memcpy(coefMatrix, RxShiftCodeCoef_m8, sizeof(coefMatrix));
            break;
        case 9:
            memcpy(coefMatrix, RxShiftCodeCoef_m9, sizeof(coefMatrix));
            break;
        case 10:
            memcpy(coefMatrix, RxShiftCodeCoef_m10, sizeof(coefMatrix));
            break;
        case 11:
            memcpy(coefMatrix, RxShiftCodeCoef_m11, sizeof(coefMatrix));
            break;
        case 12:
            memcpy(coefMatrix, RxShiftCodeCoef_m12, sizeof(coefMatrix));
            break;
        case 13:
            memcpy(coefMatrix, RxShiftCodeCoef_m13, sizeof(coefMatrix));
            break;
        case 14:
            memcpy(coefMatrix, RxShiftCodeCoef_m14, sizeof(coefMatrix));
            break;
        case 15:
            memcpy(coefMatrix, RxShiftCodeCoef_m15, sizeof(coefMatrix));
            break;
        case 16:
            memcpy(coefMatrix, RxShiftCodeCoef_m16, sizeof(coefMatrix));
            break;
        case 17:
            memcpy(coefMatrix, RxShiftCodeCoef_m17, sizeof(coefMatrix));
            break;
        }

        for (i = 0; i < 16; i++) {
            RegData = coefMatrix[i];
            mstar_set_reg_16bit_by_addr(0x1330 + i, RegData, ADDRESS_MODE_16BIT);
        }
    } else if (scantag == SINGLE) {
        for (i = 0; i < 16; i++) {
            if (i < (codeLength - 1))
                RegData = 0;
            else
                RegData = 0x1000;
            mstar_set_reg_16bit_by_addr(0x1330 + i, RegData, ADDRESS_MODE_16BIT);
        }
    }
}

void mstar_mp_prepare_ana_for_short(void)
{
    u16 SH_ChT = (u16) mutual_mp_test_data->SHORT_Charge;
    u16 SH_DuT1 = (u16) mutual_mp_test_data->SHORT_Dump1;
    u16 scantag = SINGLE;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    // Stop mcu
    mstar_mp_stop_mcu();

    //set Subframe = 6 ; Sensor = 14
    msg30xx_scan_set_sensor_num(MAX_AFE_NUM_30XX);
    msg30xx_scan_set_subframe_num(6);

    // single drive decodeout should be set to 0 => 1
    mstar_reg_mask_16bit(0x130D, BIT0 | BIT1 | BIT2 | BIT3, 0, ADDRESS_MODE_16BIT);
    /// set shift all coef
    mstar_reg_mask_16bit(0x130D, BIT12 | BIT13 | BIT14 | BIT15, 0, ADDRESS_MODE_16BIT);
    /// adc analog+digital pipe delay, 60= 14 AFE. need advise, cayenne = 13 afe, camaro = 14 afe
    mstar_set_reg_16bit_by_addr(0x1029, 0x60, ADDRESS_MODE_16BIT);
    ///trim: Fout 52M &  1.2V
    mstar_set_reg_16bit_by_addr(0x1410, 0xA55A, ADDRESS_MODE_16BIT);    //password
    mstar_set_reg_16bit_by_addr(0x1414, 0xA55A, ADDRESS_MODE_16BIT);    //password
    mstar_set_reg_16bit_by_addr(0x1411, 0x877C, ADDRESS_MODE_16BIT);    //go
    /// reg_drv0_sel_mode : mutual mode
    msg30xx_Tgen_Ovwr_invert_mode(DISABLE, DISABLE);
    /// reg_dac0_en, reg_dac0_1p2ldo_en
    mstar_set_reg_16bit_by_addr(0x1537, BIT0 | BIT1, ADDRESS_MODE_16BIT);
    /// reg_dac_clk_force_on = DAC clock always enabled
    mstar_set_reg_16bit_by_addr(0x2000, BIT0 | BIT1 | BIT2 | BIT3 | BIT4, ADDRESS_MODE_16BIT);
    mstar_mp_set_noise_sensor_mode(ENABLE);
    mstar_mp_ana_fix_prs(3);    //prs1
    if (SH_ChT == 0 || SH_DuT1 == 0) {
        SH_ChT = 0x007E;
        SH_DuT1 = 0x000C;
    }
    mstar_mp_ana_change_cd_time(SH_ChT, SH_DuT1);
    /// all AFE Cfb use by autosettings.ini setting, defalt (50p)
    mstar_set_reg_16bit_by_addr(0x1504, 0x3FFF, ADDRESS_MODE_16BIT);    // all AFE Cfb: SW control
    Msg30xxSetCfb(_Msg30xx50p); // all AFE Cfb use defalt (50p)

    /// reg_afe_icmp disable
    mstar_set_reg_16bit_by_addr(0x1529, 0x0000, ADDRESS_MODE_16BIT);
    msg30xx_ana_enable_charge_pump(ENABLE);
    ///ADC: AFE Gain bypass
    mstar_set_reg_16bit_by_addr(0x1230, 0x3FFF, ADDRESS_MODE_16BIT);
    ///reg_chip_swap_pn_en
    msg30xx_set_chip_swap_enable(DISABLE);
    ///reg_adc_desp_invert disable
    msg30xx_adc_desp_invert_enable(DISABLE);
    //mstar_set_reg_16bit_by_addr(0x1012, 0x0680, ADDRESS_MODE_16BIT); // same as Cobra
    mstar_set_reg_16bit_by_addr(0x1022, 0x0000, ADDRESS_MODE_16BIT);    // Cobra : 4T, Camaro : 1T
    mstar_set_reg_16bit_by_addr(0x110A, 0x0104, ADDRESS_MODE_16BIT);    // default value
    mstar_set_reg_16bit_by_addr(0x1310, 0x093F, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1311, 0x093F, ADDRESS_MODE_16BIT);

    mstar_set_reg_16bit_by_addr(0x1019, 0x083f, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101a, 0x0000, ADDRESS_MODE_16BIT);    //post idle time in sub0
    mstar_set_reg_16bit_by_addr(0x101b, 0x0000, ADDRESS_MODE_16BIT);    //post idle time in sub1
    mstar_set_reg_16bit_by_addr(0x101c, 0x0000, ADDRESS_MODE_16BIT);    //post idle time in sub2
    mstar_set_reg_16bit_by_addr(0x101d, 0x0000, ADDRESS_MODE_16BIT);    //post idle time in sub3
    // fix shift code
    mstar_mp_scan_shift_code_init_rx_code(msg30xx_get_drv_opening(), scantag);  // set shift code by drive mode
    // re-set sample and coefficient
    //20170111, if short test fout is floating huge, then change sample number try this to keep fout static state.
    msg30xx_scan_set_sample_num(0x20);
    mstar_set_reg_16bit_by_addr(0x136B, 0x8000, ADDRESS_MODE_16BIT);
    /// set shift fir out -> (1) multi-drive shift code : 1/8 fout (fw : 1/16 -> mp : 1/128) (2) single-drive shift code : 1/16 fout (fw : 1 -> mp : 1/16)
    if (scantag == SINGLE)
        mstar_reg_mask_16bit(0x130D, BIT4 | BIT5 | BIT6 | BIT7, BIT6, ADDRESS_MODE_16BIT);  // 2 dac fw -> mp setting
    else
        mstar_reg_mask_16bit(0x130D, BIT4 | BIT5 | BIT6 | BIT7, BIT4 | BIT5 | BIT6, ADDRESS_MODE_16BIT);

    msg30xx_Tgen_Ovwr_DrvL_Buf_gain_setting(ENABLE, DISABLE);
    msg30xx_Tgen_Ovwr_DrvL_Buf_cfg_setting(ENABLE, DISABLE);
}

void mstar_mp_fw_ana_setting_for_short(void)
{
    int i = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    // overwrite sensor PAD , restore to default state
    for (i = 0; i < 8; i++)
        mstar_set_reg_16bit_by_addr(0x1e33 + i, 0x0000, ADDRESS_MODE_16BIT);
    // overwrite PAD gpio , restore to default state
    mstar_set_reg_16bit_by_addr(0x1e30, 0x000f, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1e31, 0x0000, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1e32, 0xffff, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1e3b, 0xffff, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1e3c, 0xffff, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1e3d, 0x003f, ADDRESS_MODE_16BIT);
    for (i = 0; i < 20; i++)
        mstar_set_reg_16bit_by_addr(0x2110 + i, 0x0000, ADDRESS_MODE_16BIT);
    for (i = 0; i < 16; i++)
        mstar_set_reg_16bit_by_addr(0x2160 + i, 0x0000, ADDRESS_MODE_16BIT);
    //post idle for
    mstar_set_reg_16bit_by_addr(0x101a, 0x0028, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101b, 0x0028, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101c, 0x0028, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101d, 0x0028, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x101e, 0x0028, ADDRESS_MODE_16BIT);
}

s32 mstar_mp_ito_short(void)
{
    int *deltaC = NULL, ret = 0;
    //int deltaC[MAX_MUTUAL_NUM] = {0};
    s16 i;
    u16 drv_mode = 2;

    deltaC = (int *)kzalloc(sizeof(int) * MAX_MUTUAL_NUM, GFP_KERNEL);

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_prepare_ana_for_short();
    msg30xx_set_sensor_pad_state(POS_PULSE);
    //DAC0 dati overwrite
    mstar_set_reg_16bit_by_addr(0x1506, 0x0066, ADDRESS_MODE_16BIT);    //bit0-7 : dac0,  bit8-15 : dac1 //AFE:1.3v for test
    mstar_set_reg_16bit_by_addr(0x1507, BIT0, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_by_addr(0x1542, 0xCAFE, ADDRESS_MODE_16BIT);
    mstar_mp_ana_load_setting();
    mstar_reg_mask_16bit(0x136A, (uint) BIT15, (uint) BIT15, ADDRESS_MODE_16BIT);
    mstar_mp_ana_sw_reset();
    mstar_mp_fw_ana_setting_for_short();
    memset(g_deltac_buf, 0, sizeof(int) * MAX_MUTUAL_NUM);
    if (mstar_mp_get_valueR(g_deltac_buf, drv_mode) < 0) {
        TS_LOG_INFO("*** Msg30xx Ito Short Test# GetValueR failed! ***\n");
        ret = -1;
        goto END;
    }

    TS_LOG_DEBUG("*** Msg30xx Ito Short Test# GetValueR 1.3v! ***\n");
    mstar_mp_debug_show_array2(g_deltac_buf, 70, -32, 10, 14);

    ///DAC overwrite
    mstar_set_reg_16bit_by_addr(0x1506, 0x00FF, ADDRESS_MODE_16BIT);    //bit0-7 : dac0,  bit8-15 : dac1 //AFE:3.09v for test
    if (mstar_mp_get_valueR(deltaC, drv_mode) < 0) {
        TS_LOG_INFO("*** Msg30xx Short Test# GetValueR failed! ***\n");
        ret = -1;
        goto END;
    }

    TS_LOG_DEBUG("*** Msg30xx Ito Short Test# GetValueR 3.09v! ***\n");
    mstar_mp_debug_show_array2(deltaC, 70, -32, 10, 14);
    for (i = 0; i < 70; i++)    // 14 AFE * 5 subframe
    {
        if ((abs(deltaC[i]) < IIR_MAX) && (abs(g_deltac_buf[i]) < IIR_MAX))
            g_deltac_buf[i] = deltaC[i] - g_deltac_buf[i];
        else {
            if (abs(deltaC[i]) > abs(g_deltac_buf[i]))
                g_deltac_buf[i] = deltaC[i];
        }

        if (abs(g_deltac_buf[i]) >= (IIR_MAX))
            g_deltac_buf[i] = 0x7FFF;
        else
            g_deltac_buf[i] = abs(g_deltac_buf[i]);
    }

    TS_LOG_DEBUG("*** Msg30xx Ito Short Test# GetValueR 3.09v - 1.3v ! ***\n");
    mstar_mp_debug_show_array2(g_deltac_buf, 70, -32, 10, 14);
END:
    kfree(deltaC);
    return ret;
}

s8 g_normal_test_fail_short[TEST_ITEM_NUM] = { 0 }; //0:golden    1:ratio

u16 g_normal_check_test_fail_short[TEST_ITEM_NUM][MAX_MUTUAL_NUM];  //6:max subframe    14:max afe

s32 mstar_mp_ito_shortTest(void)
{
    int i = 0, j = 0;
    int *senseR = kcalloc(g_sense_num, sizeof(int), GFP_KERNEL);
    int *driveR = kcalloc(g_drive_num, sizeof(int), GFP_KERNEL);
    int *GRR = kcalloc(g_gr_num, sizeof(int), GFP_KERNEL);
    int thrs = 0, dump_time;
    int count_test_pin = 0;
    u16 Temp_20_3E_Settings[16] = { 0 };
    u16 nTestItemLoop = 6 - 1;
    u16 nTestItem = 0;
    u16 deep_standby = mutual_mp_test_data->deep_standby;
    u16 fmode;
    u32 nRetVal = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_finger_touch_report_disable();
    mstar_dev_hw_reset();

    mstar_mp_dbbus_enter();
    mdelay(100);

    mstar_mp_start_mcu();
    fmode = MUTUAL_SINGLE_DRIVE;
    if (mstar_mp_switch_fw_mode(&fmode, &deep_standby) < 0) {
        TS_LOG_ERR("*** Msg30xx Ito Short Test# SwitchFwMode failed! ***\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }

    mstar_mp_stop_mcu();

    dump_time = (int)((mutual_mp_test_data->SHORT_Dump1 + 1) * 4 * 2) / 52;

    thrs = mstar_mp_convert_R_value(dump_time, mutual_mp_test_data->sensorInfo.thrsShort);

    for (i = 0; i < g_sense_num; i++)
        senseR[i] = thrs;
    for (i = 0; i < g_drive_num; i++)
        driveR[i] = thrs;
    for (i = 0; i < g_gr_num; i++)
        GRR[i] = thrs;

    memset(g_normal_test_fail_short, 0, TEST_ITEM_NUM * sizeof(s8));

    for (i = 0; i < 6; i++) {   //max 6 subframe
        for (j = 0; j < 14; j++) {  // max 14 AFE
            if ((i * 14 + j) < MAX_CHANNEL_NUM_30XX)
                mutual_mp_test_result->pShortFailChannel[i * 14 + j] = (u32) PIN_UN_USE;
            g_normal_check_test_fail_short[i][j] = PIN_UN_USE;
        }
    }

    //N1_ShortTest
    if (mstar_mp_ito_short() < 0) {
        TS_LOG_INFO("*** Msg30xx Ito Short Test# Get DeltaC failed! ***\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }

    for (nTestItem = 0; nTestItem < nTestItemLoop; nTestItem++) {
        if (!mstar_mp_short_test_judge(nTestItem, g_normal_test_fail_short, g_normal_check_test_fail_short)) {
            TS_LOG_INFO("*** Msg30xx Ito Short Test# ShortTestJudge failed! ***\n");
            nRetVal = -1;
            //goto ITO_TEST_END;
        }

        count_test_pin = 0;

        for (i = 0; i < 14; i++) {
            //TS_LOG_INFO("normalTestFail_check[%d][%d] = %x", nTestItem, i, normalTestFail_check[nTestItem][i]);
            if (g_normal_check_test_fail_short[nTestItem][i] != PIN_UN_USE) {
                count_test_pin++;
            }
        }

        TS_LOG_DEBUG("nTestItem = %d, count_test_pin = %d\n", nTestItem, count_test_pin);

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        // ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str);
        if (nTestItem == 0 || nTestItem == 1 || (nTestItem == 5 && (BUFFER_NULL_LEN))) {
            TS_LOG_DEBUG("SHORT_TEST_N3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < g_sense_num; j++) {
                    if (g_normal_check_test_fail_short[nTestItem][i] ==
                        mutual_mp_test_data->PAD2Sense[j]) {
                        senseR[j] =
                            mstar_mp_convert_R_value(dump_time,
                                         g_deltac_buf[i + nTestItem * 14]);
                        TS_LOG_DEBUG("senseR[%d] = %.2d, g_deltac_buf[%d] = %d\n", j, senseR[j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);
                        mutual_mp_test_result->pShortRData[j] = senseR[j];
                        mutual_mp_test_result->pShortResultData[j] =
                            g_deltac_buf[i + nTestItem * 14];
                        if (senseR[j] >= 10)
                            mutual_mp_test_result->pShortRData[j] = 10;

                        if (0 ==
                            mstar_check_value_in_range(g_deltac_buf[i + nTestItem * 14],
                                           mutual_mp_test_data->sensorInfo.
                                           thrsShort,
                                           -mutual_mp_test_data->sensorInfo.
                                           thrsShort)) {
                            mutual_mp_test_result->pShortFailChannel[j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                            TS_LOG_DEBUG
                                ("Ito Short senseR, count_test_pin = %d, g_normal_check_test_fail_short[%d][%d] = %d, pShortFailChannel[%d] = %d, g_deltac_buf[%d] = %d\n",
                                 count_test_pin, nTestItem, i,
                                 g_normal_check_test_fail_short[nTestItem][i], j,
                                 mutual_mp_test_result->pShortFailChannel[j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);
                        }
                    }   // if (normalTestFail_check[nTestItem, i]....
                }   // for (j = 0; j < sizeof(sens...
            }   // for (i = 0; i < count_test_pin; i++)
        }       // if (nTestItem == 1 || nTe....

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        //ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str);

        if (nTestItem == 2 || nTestItem == 3 || (nTestItem == 4 && (BUFFER_NULL_LEN))) {
            TS_LOG_DEBUG("SHORT_TEST_S3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < g_drive_num; j++) {
                    if (g_normal_check_test_fail_short[nTestItem][i] ==
                        mutual_mp_test_data->PAD2Drive[j]) {
                        driveR[j] =
                            mstar_mp_convert_R_value(dump_time,
                                         g_deltac_buf[i + nTestItem * 14]);
                        TS_LOG_DEBUG("driveR[%d] = %.2d, g_deltac_buf[%d] = %d\n", j, driveR[j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);
                        mutual_mp_test_result->pShortRData[g_sense_num + j] = driveR[j];
                        mutual_mp_test_result->pShortResultData[g_sense_num + j] =
                            g_deltac_buf[i + nTestItem * 14];
                        if (driveR[j] >= 10)
                            mutual_mp_test_result->pShortRData[g_sense_num + j] = 10;
                        if (0 ==
                            mstar_check_value_in_range(g_deltac_buf[i + nTestItem * 14],
                                           mutual_mp_test_data->sensorInfo.
                                           thrsShort,
                                           -mutual_mp_test_data->sensorInfo.
                                           thrsShort)) {
                            mutual_mp_test_result->pShortFailChannel[g_sense_num + j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                            TS_LOG_DEBUG
                                ("Ito Short driveR, count_test_pin = %d, g_normal_check_test_fail_short[%d][%d] = %d, pShortFailChannel[%d] = %d, g_deltac_buf[%d] = %d\n",
                                 count_test_pin, nTestItem, i,
                                 g_normal_check_test_fail_short[nTestItem][i],
                                 g_sense_num + j,
                                 mutual_mp_test_result->pShortFailChannel[g_sense_num + j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);
                        }
                    }   // if (normalTestFail_check[nTestItem, i] ....
                }   // for (j = 0; j < sizeof(driveR....
            }   // for (i = 0; i < count_test_pin; i++)
        }       // if (nTestItem == 3 || nTe....

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        //ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str);

        if (nTestItem == 4 && (BUFFER_NULL_LEN)) {
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < g_gr_num; j++) {
                    TS_LOG_DEBUG
                        ("g_normal_check_test_fail_short[nTestItem = %d][i = %d] = %d, mutual_mp_test_data->PAD2GR[j = %d] = %d\n",
                         nTestItem, i, g_normal_check_test_fail_short[nTestItem][i], j,
                         mutual_mp_test_data->PAD2GR[j]);
                    if (g_normal_check_test_fail_short[nTestItem][i] ==
                        mutual_mp_test_data->PAD2GR[j]) {
                        GRR[j] =
                            mstar_mp_convert_R_value(dump_time,
                                         g_deltac_buf[i + nTestItem * 14]);
                        mutual_mp_test_result->pShortRData[g_sense_num + g_drive_num + j] =
                            GRR[j];
                        mutual_mp_test_result->pShortResultData[g_sense_num + g_drive_num + j] =
                            g_deltac_buf[i + nTestItem * 14];
                        if (GRR[j] >= 10)
                            mutual_mp_test_result->pShortRData[g_sense_num + g_drive_num +
                                               j] = 10.0;
                        if (0 ==
                            mstar_check_value_in_range(g_deltac_buf[i + nTestItem * 14],
                                           mutual_mp_test_data->sensorInfo.
                                           thrsShort,
                                           -mutual_mp_test_data->sensorInfo.
                                           thrsShort)) {
                            mutual_mp_test_result->pShortFailChannel[g_sense_num +
                                                 g_drive_num + j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                            TS_LOG_DEBUG
                                ("Ito Short GRR, count_test_pin = %d, g_normal_check_test_fail_short[%d][%d] = %d, pShortFailChannel[%d] = %d, g_deltac_buf[%d] = %d\n",
                                 count_test_pin, nTestItem, i,
                                 g_normal_check_test_fail_short[nTestItem][i],
                                 g_sense_num + j,
                                 mutual_mp_test_result->pShortFailChannel[g_sense_num + j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);
                        }
                    }   // if (normalTestFail_check[nTestItem, i]
                }   // for (j = 0; j < sizeof(GRR)
            }   // for (i = 0; i < count_test_pin; i++)
        }       // if (nTestItem == 4 &&

        if (g_normal_test_fail_short[nTestItem]) {
            mutual_mp_test_result->pCheck_Fail[3] = g_normal_test_fail_short[nTestItem];    // ito short fail
            nRetVal = -1;
        }
    }

    TS_LOG_INFO("-----------short test----------\n");
    for (i = 1; i < 13 + 25 + 1; i++) {
        if (i % 10 == 0)
            TS_LOG_DEBUG("\n");
        TS_LOG_DEBUG("%d,", mutual_mp_test_result->pShortResultData[i - 1]);
    }
    TS_LOG_DEBUG("\n");

    mstar_mp_exit_dbbus();

ITO_TEST_END:

    mstar_dev_hw_reset();
    mdelay(300);
    mstar_finger_touch_report_enable();
    kfree(senseR);
    kfree(driveR);
    kfree(GRR);
    return nRetVal;
}

s32 mstar_mp_short_test_entry(void)
{
    u32 nRetVal1 = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    mstar_mp_read_settings();
    g_sense_num = mutual_mp_test_data->sensorInfo.numSen;
    g_drive_num = mutual_mp_test_data->sensorInfo.numDrv;
    g_gr_num = mutual_mp_test_data->sensorInfo.numGr;
    TS_LOG_DEBUG("g_sense_num = %d, g_drive_num = %d, g_gr_num = %d\n", g_sense_num, g_drive_num, g_gr_num);

    //must execute once only , or it will return invalid buffer data (IniGetIntArray("SHORT_TEST_GR", "TEST_PIN", testPins))
    //_gMsg30xxGRSize = Msg30xxreadGR_ICpin();

    /* Running New MP Test Flow if mode is 2, otherwise older one */
    if (mp_test_info.mode == 2) {
        nRetVal1 = new_flow_main(1);
    } else {
        nRetVal1 = mstar_mp_ito_shortTest();
    }
    //nRetVal2 = Msg30xxICPinShortTest();

    TS_LOG_DEBUG("nRetVal1 = %d\n", nRetVal1);

    if ((nRetVal1 == -1))
        return -1;
    else
        return 0;
}

int mstar_mp_short_test_start(void)
{
    int nRetVal = 0;
    int nRet = 0;

    TS_LOG_DEBUG("*** %s() ***\n", __func__);

    nRetVal = mstar_mp_short_test_entry();
    if (nRetVal == 0) {
        nRet = ITO_TEST_OK; //PASS
        TS_LOG_INFO("Msg30xx Short Test# MP test success\n");
    } else {
        if (nRetVal == -1) {
            TS_LOG_INFO("Msg30xx Short Test# MP test fail\n");
            nRet = ITO_TEST_FAIL;
        } else if (nRetVal == -2) {
            nRet = ITO_TEST_GET_TP_TYPE_ERROR;
        } else {
            nRet = ITO_TEST_UNDEFINED_ERROR;
        }

        TS_LOG_ERR("Msg30xx Short# MP test failed\n");
    }

    return nRet;
}
