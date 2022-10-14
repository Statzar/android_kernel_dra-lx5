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
extern u16 g_normal_test_result_check[MAX_MUTUAL_NUM];

extern s8 g_normal_test_fail_short[TEST_ITEM_NUM];
extern u16 g_normal_check_test_fail_short[TEST_ITEM_NUM][MAX_MUTUAL_NUM];

struct mp_test_info {
    int mp_run;
    int sense_num;
    int drive_num;
    int gr_num;
    int short_thrs;
};

struct mp_test_info mti;

enum new_flow_test_type {
    TYPE_RESET = 0x0,
    TYPE_OPEN = 0x1,
    TYPE_SHORT = 0x2,
    TYPE_BootPalm = 0x3,
    TYPE_KPhase = 0x4,
    TYPE_Idle = 0x5,
};

#define MP_DEBUG

static void new_send_dbbus_access_command(u8 byte)
{
    u8 cmd[1] = { 0 };

    cmd[0] = byte;
    mstar_iic_write_data(SLAVE_I2C_ID_DBBUS, cmd, 1);
}

static int new_get_twoshort_raw_data(s16 * pRawData)
{
    int i, j, offset, ret = 0;
    u8 *pFirstShotOri = NULL;
    s16 *pFirstShotAll = NULL;
    u8 *pSecondShotOri = NULL;
    s16 *pSecondShotAll = NULL;
    u16 fout_base_addr = 0x0, RegData = 0;
    u16 data_length = 0;
    s32 sum = 0, cunt = 0, avg = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    /* In short, only 56 IC pins will be scaned. */
    data_length = 56 * 2;

    pFirstShotOri = kcalloc(data_length * 2, sizeof(u8), GFP_KERNEL);
    pSecondShotOri = kcalloc(data_length * 2, sizeof(u8), GFP_KERNEL);

    if (ERR_ALLOC_MEM(pFirstShotOri) || ERR_ALLOC_MEM(pSecondShotOri)) {
        TS_LOG_ERR("Failed to allocate pFirstShot/pSecondShot mem\n");
        ret = -1;
        goto out;
    }

    pFirstShotAll = kcalloc(data_length, sizeof(s16), GFP_KERNEL);
    pSecondShotAll = kcalloc(data_length, sizeof(s16), GFP_KERNEL);

    if (ERR_ALLOC_MEM(pFirstShotAll) || ERR_ALLOC_MEM(pSecondShotAll)) {
        TS_LOG_ERR("Failed to allocate pFirstShotAll/pSecondShotAll mem\n");
        ret = -1;
        goto out;
    }

    /* Read DQ base address for first shot */
    RegData = mstar_get_reg_16bit_by_addr(0x1362, ADDRESS_MODE_16BIT);
    fout_base_addr = (int)(RegData << 2);

    TS_LOG_INFO("*** %s: First: fout_base_addr = 0x%x ***\n", __func__, fout_base_addr);

    mstar_mp_stop_mcu();

    new_send_dbbus_access_command(0x80);
    new_send_dbbus_access_command(0x82);
    new_send_dbbus_access_command(0x85);

    /* Read data directly */
    mstar_mp_ddbus_read_dq_mem_start();
    mstar_reg_get_xbit_write_4byte_value(fout_base_addr, pFirstShotOri, data_length, MAX_I2C_TRANSACTION_LENGTH_LIMIT);
    //mstar_mp_dbbus_read_dq_mem_end();
    new_send_dbbus_access_command(0x7F);

#ifdef MP_DEBUG
    TS_LOG_INFO("*** %s: First Shot Original Data ***\n", __func__);
    for (i = 0; i < data_length; i++) {
        TS_LOG_DEBUG(" %02x ", pFirstShotOri[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    /* Cmobine 8 bit to 16 bit */
    for (j = 0; j < data_length / 2; j++) {
        pFirstShotAll[j] = (pFirstShotOri[2 * j] | pFirstShotOri[2 * j + 1] << 8);
    }

#ifdef MP_DEBUG
    TS_LOG_INFO("*** %s: First Shot Data Combined ***\n", __func__);
    for (i = 0; i < data_length / 2; i++) {
        TS_LOG_DEBUG(" %04d ", pFirstShotAll[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    new_send_dbbus_access_command(0x51);
    new_send_dbbus_access_command(0x81);
    new_send_dbbus_access_command(0x83);
    new_send_dbbus_access_command(0x84);
    new_send_dbbus_access_command(0x35);
    new_send_dbbus_access_command(0x7E);
    new_send_dbbus_access_command(0x34);
    mstar_reg_mask_16bit(0x0F28, BIT1, BIT1, ADDRESS_MODE_16BIT);
    /* Read DQ base address for second shot */
    mstar_reg_mask_16bit(0x0FE6, BIT0, 0, ADDRESS_MODE_16BIT);
    mstar_dbbus_wait_mcu();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    mstar_mp_start_mcu();
    RegData = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
    mstar_mp_stop_mcu();
    mstar_reg_mask_16bit(0x0FE6, BIT0, BIT0, ADDRESS_MODE_16BIT);
    mstar_set_reg_16bit_off(0x0F50, BIT2);
    fout_base_addr = (int)(RegData << 2);

    TS_LOG_INFO("*** %s: Second: fout_base_addr = 0x%x ***\n", __func__, fout_base_addr);

    mstar_mp_ddbus_read_dq_mem_start();
    mstar_reg_get_xbit_write_4byte_value(fout_base_addr, pSecondShotOri, data_length * 2, MAX_I2C_TRANSACTION_LENGTH_LIMIT);
    mstar_mp_dbbus_read_dq_mem_end();

#ifdef MP_DEBUG
    TS_LOG_INFO("*** %s: Second Shot Original Data ***\n", __func__);
    for (i = 0; i < data_length; i++) {
        TS_LOG_DEBUG(" %02x ", pSecondShotOri[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    /* Cmobine 8 bit to 16 bit */
    for (j = 0; j < data_length / 2; j++) {
        pSecondShotAll[j] = (pSecondShotOri[2 * j] | pSecondShotOri[2 * j + 1] << 8);
    }

#ifdef MP_DEBUG
    TS_LOG_INFO("*** %s: Second Shot Data Combined ***\n", __func__);
    for (i = 0; i < data_length / 2; i++) {
        TS_LOG_DEBUG(" %04d ", pSecondShotAll[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    /*
     * TODO: Should implement Short Method 2 to calcuate the final result.
     * As the old flow did implement Short Method 1 instead of 2, we need to
     * implment the calculation of Method 2 in this new flow.
     */
    for (i = 0; i < 28; i++)    // 2 AFE * 28 subframe,but AFE0 un-use
    {
        pFirstShotAll[i * 2] = pSecondShotAll[i * 2 + 1];

        if ((pFirstShotAll[i * 2] > -1000) && (pFirstShotAll[i * 2] < 1000)) {
            sum += pFirstShotAll[i * 2];
            cunt++;
        }

        if ((pFirstShotAll[i * 2 + 1] > -1000) && (pFirstShotAll[i * 2 + 1] < 1000)) {
            sum += pFirstShotAll[i * 2 + 1];
            cunt++;
        }
    }

    if (cunt != 0) {
        avg = (int)(sum / cunt);
        TS_LOG_INFO(" ********* SUM = %d, cunt = %d, AVG = %d  ************\n", sum, cunt, avg);
        for (i = 0; i < 56; i++) {
            TS_LOG_DEBUG("pFirstShotAll[%d] = %d\n", i, pFirstShotAll[i]);
            pFirstShotAll[i] = pFirstShotAll[i] - avg;
            pRawData[i] = pFirstShotAll[i];
        }
    }
#ifdef MP_DEBUG
    TS_LOG_INFO("*** %s: Final Short Data (unsorted) ***\n", __func__);
    for (i = 0; i < data_length / 2; i++) {
        TS_LOG_DEBUG(" %04d ", pRawData[i]);
        if (i != 0 && (i % 10 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    new_send_dbbus_access_command(0x84);
    mstar_mp_start_mcu();

out:
    if(!ERR_ALLOC_MEM(pFirstShotOri)) {
        kfree(pFirstShotOri);
        pFirstShotOri = NULL;
    }
    if(!ERR_ALLOC_MEM(pSecondShotOri)) {
        kfree(pSecondShotOri);
        pSecondShotOri = NULL;
    }
    if(!ERR_ALLOC_MEM(pFirstShotAll)) {
        kfree(pFirstShotAll);
        pFirstShotAll = NULL;
    }
    if(!ERR_ALLOC_MEM(pSecondShotAll)) {
        kfree(pSecondShotAll);
        pSecondShotAll = NULL;
    }

    return ret;
}

static int new_get_oneshot_raw_data(s16 *pRawData)
{
    int i, j, shift = 0, offset, ret = 0;
    u8 *pShotOriData = NULL;
    s32 *pShotDataAll = NULL;
    u16 fout_base_addr = 0x0, RegData = 0;
    u16 data_length = mti.sense_num * mti.drive_num;

    TS_LOG_INFO("*** %s() data_length = %d ***\n", __func__, data_length);

    /* one byte original data */
    pShotOriData = kcalloc(data_length * 2, sizeof(u8), GFP_KERNEL);
    if (ERR_ALLOC_MEM(pShotOriData)) {
        TS_LOG_ERR("Failed to allocate pShotOriData mem\n");
        ret = -1;
        goto out;
    }

    /* two bytes combined by above */
    pShotDataAll = kcalloc(data_length, sizeof(s32), GFP_KERNEL);
    if (ERR_ALLOC_MEM(pShotDataAll)) {
        TS_LOG_ERR("Failed to allocate pShotDataAll mem\n");
        ret = -1;
        goto out;
    }

    /* Read DQ base */
    RegData = mstar_get_reg_16bit_by_addr(0x1361, ADDRESS_MODE_16BIT);
    fout_base_addr = (int)(RegData << 2);

    TS_LOG_INFO("*** %s: fout_base_addr = 0x%x ***\n", __func__, fout_base_addr);

    if (fout_base_addr <= 0) {
        TS_LOG_ERR("Failed to get fout_base_addr\n");
        ret = -1;
        goto out;
    }

    mstar_mp_stop_mcu();
    mstar_reg_mask_16bit(0x1066, 0x01, 0x01, ADDRESS_MODE_16BIT);

    /* Read data segmentally */
    for (offset = 0; offset < data_length * 2; offset += MAX_I2C_TRANSACTION_LENGTH_LIMIT) {
        mstar_mp_ddbus_read_dq_mem_start();
        if (offset == 0) {
            mstar_reg_get_xbit_write_4byte_value(fout_base_addr + offset, pShotOriData + offset,
                                 MAX_I2C_TRANSACTION_LENGTH_LIMIT,
                                 MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        }
        //RegGetXBitValue(fout_base_addr + offset, pShotOriData + offset, MAX_I2C_TRANSACTION_LENGTH_LIMIT, MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        else if (offset + MAX_I2C_TRANSACTION_LENGTH_LIMIT < data_length * 2) {
            mstar_reg_get_xbit_write_4byte_value(fout_base_addr + offset, pShotOriData + offset,
                                 MAX_I2C_TRANSACTION_LENGTH_LIMIT,
                                 MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        } else {
            mstar_reg_get_xbit_write_4byte_value(fout_base_addr + offset, pShotOriData + offset,
                                 data_length * 2 - offset,
                                 MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        }
        //RegGetXBitValue(fout_base_addr + offset, pShotOriData + offset, offset, MAX_I2C_TRANSACTION_LENGTH_LIMIT);
        mstar_mp_dbbus_read_dq_mem_end();
    }
#ifdef MP_DEBUG
    for (i = 0; i < data_length * 2; i++) {
        TS_LOG_DEBUG(" %02x ", pShotOriData[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    for (j = 0; j < data_length; j++) {
        pShotDataAll[j] = (pShotOriData[2 * j] | pShotOriData[2 * j + 1] << 8);
    }

#ifdef MP_DEBUG
    for (i = 0; i < data_length; i++) {
        TS_LOG_DEBUG(" %02x ", pShotDataAll[i]);
        if (i != 0 && (i % 16 == 0))
            TS_LOG_DEBUG("\n");
    }
    TS_LOG_DEBUG(" \n\n ");
#endif

    for (i = 0; i < mti.sense_num; i++) {
        for (j = 0; j < mti.drive_num; j++) {
            shift = i * mti.drive_num + j;
            pRawData[shift] = pShotDataAll[shift];
        }
    }

    //mstar_dbbus_enter_serial_debug();
    mstar_dbbus_iic_use_bus();
    mstar_dbbus_iic_reshape();
    mstar_mp_start_mcu();

out:
    if(!ERR_ALLOC_MEM(pShotOriData)) {
        kfree(pShotOriData);
        pShotOriData = NULL;
    }
    if(!ERR_ALLOC_MEM(pShotDataAll)) {
        kfree(pShotDataAll);
        pShotDataAll = NULL;
    }

    return ret;
}

static int new_get_deltac(void)
{
    int i;
    s16 *pRawData = NULL, count = 0;

    pRawData = kcalloc(MAX_CHANNEL_SEN * MAX_CHANNEL_DRV, sizeof(s16), GFP_KERNEL);
    if (ERR_ALLOC_MEM(pRawData)) {
        TS_LOG_ERR("Failed to allocate pRawData mem \n");
        return -1;
    }

    if (mti.mp_run) {
        //count = mti.sense_num + mti.drive_num;
        count = 56;
        if (new_get_twoshort_raw_data(pRawData) < 0) {
            TS_LOG_ERR("*** Get DeltaC failed! ***\n");
            return -1;
        }
    } else {
        count = mti.sense_num * mti.drive_num;
        if (new_get_oneshot_raw_data(pRawData) < 0) {
            TS_LOG_ERR("*** Get DeltaC failed! ***\n");
            return -1;
        }
    }

    for (i = 0; i < count; i++) {
        g_deltac_buf[i] = pRawData[i];
    }
    TS_LOG_INFO("*** new_get_deltac ***\n");
#ifdef MP_DEBUG
    if (mti.mp_run) {
        for (i = 0; i < count; i++) {
            TS_LOG_DEBUG(" %05d ", g_deltac_buf[i]);
            if (i != 0 && (i % 10) == 0)
                TS_LOG_DEBUG(" \n ");
        }
        TS_LOG_DEBUG(" \n ");
    } else {
        for (i = 0; i < count; i++) {
            TS_LOG_DEBUG(" %05d ", g_deltac_buf[i]);
            if (i != 0 && (i % mti.drive_num) == 0)
                TS_LOG_DEBUG(" \n ");
        }
        TS_LOG_DEBUG(" \n ");
    }
#endif
    if(!ERR_ALLOC_MEM(pRawData)) {
        kfree(pRawData);
        pRawData = NULL;
    }

    return 0;
}

static int new_short_judege(void)
{
    int i, j;
    int *senseR = NULL;
    int *driveR = NULL;
    int *GRR = NULL;
    int count_test_pin = 0;
    int thrs = 0, dump_time;
    u16 Temp_20_3E_Settings[16] = { 0 };
    u16 nTestItemLoop = 6 - 1;
    u16 nTestItem = 0;
    u32 nRetVal = 0;

    memset(g_normal_test_fail_short, 0, TEST_ITEM_NUM * sizeof(s8));

    senseR = kcalloc(mti.sense_num, sizeof(int), GFP_KERNEL);
    driveR = kcalloc(mti.drive_num, sizeof(int), GFP_KERNEL);
    GRR = kcalloc(mti.gr_num, sizeof(int), GFP_KERNEL);
    if (ERR_ALLOC_MEM(senseR) || ERR_ALLOC_MEM(driveR) || ERR_ALLOC_MEM(GRR)) {
        TS_LOG_ERR("Failed to allocate senseR/driveR/GRR mem\n");
        nRetVal = -1;
        goto ITO_TEST_END;
    }

    dump_time = (((mutual_mp_test_data->SHORT_Dump1 + 1) * 4 * 2) * 100) / 5166;

    thrs = mstar_mp_convert_R_value(dump_time, mti.short_thrs);

    for (i = 0; i < mti.sense_num; i++)
        senseR[i] = thrs;
    for (i = 0; i < mti.drive_num; i++)
        driveR[i] = thrs;
    for (i = 0; i < mti.gr_num; i++)
        GRR[i] = thrs;

    for (i = 0; i < 6; i++) {   //max 6 subframe
        for (j = 0; j < 14; j++) {  // max 14 AFE
            if ((i * 14 + j) < MAX_CHANNEL_NUM_30XX)
                mutual_mp_test_result->pShortFailChannel[i * 14 + j] = (u32) PIN_UN_USE;
            g_normal_check_test_fail_short[i][j] = PIN_UN_USE;
        }
    }

    /*
     * Here is a new judge flow copied from older one.
     * The given data is sorted by PAD TAble written in INI.
     */
    for (nTestItem = 0; nTestItem < nTestItemLoop; nTestItem++) {
        if (!mstar_mp_short_test_judge(nTestItem, g_normal_test_fail_short, g_normal_check_test_fail_short)) {
            TS_LOG_INFO("*** New ITO Short Judge failed! ***\n");
            nRetVal = -1;
            //goto ITO_TEST_END;
        }

        count_test_pin = 0;

        for (i = 0; i < 14; i++) {
            //TS_LOG_INFO("normalTestFail_check[%d][%d] = %x", nTestItem, i, normalTestFail_check[nTestItem][i]);
            if (g_normal_check_test_fail_short[nTestItem][i] != PIN_UN_USE)
                count_test_pin++;
        }

        TS_LOG_INFO("%s: nTestItem = %d, count_test_pin = %d\n", __func__, nTestItem, count_test_pin);

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        //ms_getInidata("SHORT_TEST_N3", "MUX_MEM_20_3E", str);

        if (nTestItem == 0 || nTestItem == 1 || (nTestItem == 4 && BUFFER_NULL_LEN)) {
            TS_LOG_INFO("NEW SHORT_TEST_N3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < mti.sense_num; j++) {
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
                                           mti.short_thrs, -mti.short_thrs))
                            mutual_mp_test_result->pShortFailChannel[j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        //ms_getInidata("SHORT_TEST_S3", "MUX_MEM_20_3E", str);

        if (nTestItem == 2 || nTestItem == 3 || (nTestItem == 4 && BUFFER_NULL_LEN)) {
            TS_LOG_INFO("NEW SHORT_TEST_S3\n");
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < mti.drive_num; j++) {
                    if (g_normal_check_test_fail_short[nTestItem][i] ==
                        mutual_mp_test_data->PAD2Drive[j]) {
                        driveR[j] =
                            mstar_mp_convert_R_value(dump_time,
                                         g_deltac_buf[i + nTestItem * 14]);

                        TS_LOG_DEBUG("driveR[%d] = %.2d, g_deltac_buf[%d] = %d\n", j, driveR[j],
                                 i + nTestItem * 14, g_deltac_buf[i + nTestItem * 14]);

                        mutual_mp_test_result->pShortRData[mti.sense_num + j] = driveR[j];
                        mutual_mp_test_result->pShortResultData[mti.sense_num + j] =
                            g_deltac_buf[i + nTestItem * 14];

                        if (driveR[j] >= 10)
                            mutual_mp_test_result->pShortRData[mti.sense_num + j] = 10;

                        if (0 ==
                            mstar_check_value_in_range(g_deltac_buf[i + nTestItem * 14],
                                           mti.short_thrs, -mti.short_thrs))
                            mutual_mp_test_result->pShortFailChannel[mti.sense_num + j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                    }
                }
            }
        }

        memset(Temp_20_3E_Settings, 0, sizeof(Temp_20_3E_Settings));
        //ms_getInidata("SHORT_TEST_GR", "MUX_MEM_20_3E", str);

        if (nTestItem == 4 && BUFFER_NULL_LEN) {
            for (i = 0; i < count_test_pin; i++) {
                for (j = 0; j < mti.gr_num; j++) {
                    if (g_normal_check_test_fail_short[nTestItem][i] ==
                        mutual_mp_test_data->PAD2GR[j]) {
                        GRR[j] =
                            mstar_mp_convert_R_value(dump_time,
                                         g_deltac_buf[i + nTestItem * 14]);

                        mutual_mp_test_result->pShortRData[mti.sense_num + mti.drive_num + j] =
                            GRR[j];
                        mutual_mp_test_result->pShortResultData[mti.sense_num + mti.drive_num +
                                            j] =
                            g_deltac_buf[i + nTestItem * 14];

                        if (GRR[j] >= 10)
                            mutual_mp_test_result->pShortRData[mti.sense_num +
                                               mti.drive_num + j] = 10.0;

                        if (0 ==
                            mstar_check_value_in_range(g_deltac_buf[i + nTestItem * 14],
                                           mti.short_thrs, -mti.short_thrs))
                            mutual_mp_test_result->pShortFailChannel[mti.sense_num +
                                                 mti.drive_num + j] =
                                (u32) g_normal_check_test_fail_short[nTestItem][i];
                    }
                }
            }
        }

        if (g_normal_test_fail_short[nTestItem]) {
            mutual_mp_test_result->pCheck_Fail[3] = g_normal_test_fail_short[nTestItem];    // ito short fail
            nRetVal = -1;
        }
    }

ITO_TEST_END:
    kfree(senseR);
    kfree(driveR);
    kfree(GRR);
    return nRetVal;
}

static int new_send_test_cmd(u16 fmode)
{
    int ret = 0;
    u8 cmd[8] = { 0 };
    u8 Freq = mutual_mp_test_data->Open_fixed_carrier;
    u8 Freq1 = mutual_mp_test_data->Open_fixed_carrier1;
    u16 chargeT = 0, dumpT = 0;
    u8 Csub = 0, Cfb = 0;
    u8 chargeP = 0;

    cmd[0] = 0xF1;      /* Header */

    if (mti.mp_run) {
        cmd[1] = TYPE_SHORT;
        cmd[2] = 0;
        cmd[3] = 0;
        cmd[4] = 0;
        cmd[5] = 0;
        cmd[6] = 0;
        cmd[7] = 0x0;

    } else {
        cmd[1] = TYPE_OPEN;
        chargeT = mutual_mp_test_data->OPEN_Charge;
        dumpT = mutual_mp_test_data->OPEN_Dump;
        Csub = mutual_mp_test_data->Open_test_csub;
        Cfb = mutual_mp_test_data->Open_test_cfb;
        chargeP = (mutual_mp_test_data->Open_test_chargepump ? 0x02 : 0x00);

        if (fmode == MUTUAL_SINE) {
            cmd[2] = (0x01 | chargeP);

            /* Open test by each frequency */
            cmd[3] = Freq;
            cmd[4] = Freq1;
        } else {
            cmd[2] = (0x00 | chargeP);

            if (chargeT == 0 || dumpT == 0) {
                chargeT = 0x18;
                dumpT = 0x16;
            }

            /* Switch cap mode */
            cmd[3] = Freq;
            cmd[4] = Freq1;
        }

        cmd[5] = Csub;
        switch (Cfb) {
        case 0:
            cmd[6] = 2; //30P
            break;
        case 1:
            //anaTest.SetCfb(mdkDevice, protoHandle, mDK.ana_mutual.CFB_5846A._25p);
            cmd[6] = 7; //25P
            break;
        case 2:
            //anaTest.SetCfb(mdkDevice, protoHandle, mDK.ana_mutual.CFB_5846A._30p);
            cmd[6] = 2; //30P
            break;
        case 3:
            //anaTest.SetCfb(mdkDevice, protoHandle, mDK.ana_mutual.CFB_5846A._35p);
            cmd[6] = 6; //35P
            break;
        case 4:
            //anaTest.SetCfb(mdkDevice, protoHandle, mDK.ana_mutual.CFB_5846A._45p);
            cmd[6] = 5; //45P
            break;
        case 5:
            //anaTest.SetCfb(mdkDevice, protoHandle, mDK.ana_mutual.CFB_5846A._50p);
            cmd[6] = 0; //50P
            break;
        }
        cmd[7] = 0x0;
    }

#ifdef MP_DEBUG
    {
        int i;
        for (i = 0; i < ARRAY_SIZE(cmd); i++)
            TS_LOG_DEBUG("*** %s: cmd[%d] = %d  ***\n", __func__, i, cmd[i]);
    }
#endif

    /* Writting commands via SMBus */
    mstar_iic_write_data(SLAVE_I2C_ID_DWI2C, &cmd[0], ARRAY_SIZE(cmd));
    return ret;
}

static int new_polling_data_ready(void)
{
    int ret = 0;
    int timer = 500;
    u16 RegData = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    while (RegData != 0x7744) {
        RegData = mstar_get_reg_16bit_by_addr(0x1401, ADDRESS_MODE_16BIT);
        TS_LOG_INFO("TIMER = %d, RegData = 0x%04x\n", timer, RegData);
        mdelay(20);
        timer--;
        if (timer < 0)
            break;
    }

    if (timer <= 0)
        ret = -1;

    return ret;
}

static s32 new_clear_switch_status(void)
{
    int regData = 0;
    int timeout = 280;
    int t = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);

    do
    {
        mstar_set_reg_16bit(0x1402, 0xFFFF);
        regData = mstar_get_reg_16bit_by_addr(0x1401, ADDRESS_MODE_16BIT);
        mdelay(20);
        t++;
        if(t > timeout)
            return -1;

        TS_LOG_DEBUG("t = %d, regData = 0x%04x\n", t, regData);

    } while(regData != 0xFFFF);

    return 0;
}

static s32 new_check_switch_status(void)
{
    u16 nRegData = 0;
    int nTimeOut = 50;
    int nT = 0;

    TS_LOG_INFO("*** %s() ***\n", __func__);
    do {
        nRegData = mstar_get_reg_16bit_by_addr(0x1401, ADDRESS_MODE_16BIT);
        mdelay(20);
        nT++;
        if (nT > nTimeOut)
            return -1;

        TS_LOG_INFO("%s: nT = %d, nRegData = 0x%04x\n", __func__, nT, nRegData);
    } while (nRegData != 0x7447);

    return 0;
}

static int new_flow_start_test(void)
{
    int ret = 0;
    u16 fmode = MUTUAL_MODE;
    s8 nNormalTestResult[2] = { 0 };

    switch (mutual_mp_test_data->Open_mode) {
    case 0:
        fmode = MUTUAL_MODE;
        break;
    case 1:
    case 2:
        fmode = MUTUAL_SINE;
        break;
    }

    mti.sense_num = mutual_mp_test_data->sensorInfo.numSen;
    mti.drive_num = mutual_mp_test_data->sensorInfo.numDrv;
    mti.gr_num = mutual_mp_test_data->sensorInfo.numGr;
    mti.short_thrs = mutual_mp_test_data->sensorInfo.thrsShort;

    TS_LOG_INFO("*** %s(): SenSe = %d, Driver = %d, GR = %d ***\n", __func__, mti.sense_num, mti.drive_num,
             mti.gr_num);

    TS_LOG_INFO("*** %s(): fmode = %d ***\n", __func__, fmode);

    new_send_test_cmd(fmode);

    if (new_polling_data_ready() < 0) {
        TS_LOG_ERR("New Flow polling data timout !!\n");
        return -1;
    }

    /* Clear MP mode */
    mstar_set_reg_16bit(0x1402, 0xFFFF);

    /* Get DeltaC */
    ret = new_get_deltac();
    if (ret == 0) {
        /* Judge values */
        if (mti.mp_run)
            ret = new_short_judege();
        else
            ret = mstar_mp_open_judge(0, nNormalTestResult, g_normal_test_result_check);

        TS_LOG_INFO("*** New Judge return value = %d ***\n", ret);
    }

    return ret;
}

int new_flow_main(int item)
{
    int ret = 0, retry = 0;
    TS_LOG_INFO("*** %s() ***\n", __func__);

    mti.mp_run = item;

    TS_LOG_INFO("*** %s: Running %s Test ***\n", __func__, (mti.mp_run ? "Short" : "Open"));

    mstar_finger_touch_report_disable();
MP_RETRY:
    mstar_dev_hw_reset();
    mdelay(100);

    mstar_dbbus_enter_serial_debug();
    mstar_dbbus_wait_mcu();
    mstar_dbbus_iic_use_bus();  //0x35
    mstar_dbbus_iic_reshape();  //0x71

    new_send_dbbus_access_command(0x80);
    new_send_dbbus_access_command(0x82);
    new_send_dbbus_access_command(0x84);
    mstar_set_reg_16bit_off(0x1E04, BIT15);
    mstar_dbbus_i2c_response_ack();

    mdelay(200);

    mstar_mp_stop_mcu();
    mstar_set_reg_16bit(0X3C60, 0xAA55);    // disable watch dog

    mstar_set_reg_16bit(0X3D08, 0xFFFF);
    mstar_set_reg_16bit(0X3D18, 0xFFFF);

    mstar_set_reg_16bit(0x1402, 0x7474);    // enter mp mode
    mstar_set_reg_16bit(0x1E06, 0x0000);
    mstar_set_reg_16bit(0x1E06, 0x0001);
    mdelay(200);

    mstar_mp_start_mcu();

    if (new_check_switch_status() < 0) {
        TS_LOG_ERR("*** %s Switch FW Mode Busy, return fail retry = %d***\n", __func__, retry);
        if(retry < 3)
        {
            retry++;
            goto MP_RETRY;
        } else {
            ret = -1;
            goto out;
        }
    }

    if(new_clear_switch_status() < 0) {
        TS_LOG_ERR("*** Clear Switch status fail ***\n");
        ret = -1;
        goto out;
    }

    //mstar_set_reg_16bit(0x1402, 0xFFFF);    // clear MP mode
    mutual_mp_test_data->SHORT_Dump1 = mstar_get_reg_16bit_by_addr(0x1018, ADDRESS_MODE_16BIT);
    ret = new_flow_start_test();
out:
    /* Exit DBbus */
    mstar_set_reg_16bit_on(0x1E04, BIT15);
    new_send_dbbus_access_command(0x84);
    mstar_mp_exit_dbbus();

    mstar_dev_hw_reset();
    mdelay(10);
    mstar_finger_touch_report_enable();
    return ret;
}
