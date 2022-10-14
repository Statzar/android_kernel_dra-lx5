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

extern MpTestInfo_t mp_test_info;
extern u32 g_IsInMpTest;
int MpTestStatus = 0;

static void mstar_mp_convert_csv_file(int result)
{
    int i = 0, j = 0;

    /* Store Golden data */
    for (j = 0; j < mutual_mp_test_data->sensorInfo.numSen; j++) {
        for (i = 0; i < mutual_mp_test_data->sensorInfo.numDrv; i++) {
            if (mutual_mp_test_result->pGolden_CH[j * mutual_mp_test_data->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
            {
                /* TODO */
                //length += sprintf(SetCsvData+length, "%s", ",");
            } else {
                //length += sprintf(SetCsvData+length, "%.2d,", mutual_mp_test_result->pGolden_CH[j * mutual_mp_test_data->sensorInfo.numDrv + i]);
                mp_test_info.GoldenData[j * mutual_mp_test_data->sensorInfo.numDrv + i] =
                    mutual_mp_test_result->pGolden_CH[j * mutual_mp_test_data->sensorInfo.numDrv + i];
            }
        }
        //length += sprintf(SetCsvData+length,"%s", line);
    }

    /* Store Deltac data */
    for (j = 0; j < mutual_mp_test_data->sensorInfo.numSen; j++) {
        //length += sprintf(SetCsvData+length, ",S%d,", j+1);
        for (i = 0; i < mutual_mp_test_data->sensorInfo.numDrv; i++) {
            if (mutual_mp_test_result->pOpenResultData[j * mutual_mp_test_data->sensorInfo.numDrv + i] == NULL_DATA)    //for mutual key
            {
                //length += sprintf(SetCsvData+length, "%s", ",");
            } else {
                //length+=sprintf(SetCsvData+length, "%1d,", mutual_mp_test_result->pOpenResultData[j * mutual_mp_test_data->sensorInfo.numDrv + i]);
                mp_test_info.DeltaCData[i + j * mutual_mp_test_data->sensorInfo.numDrv] =
                    mutual_mp_test_result->pOpenResultData[j * mutual_mp_test_data->sensorInfo.numDrv +
                                       i];
            }
        }
        //length+=sprintf(SetCsvData+length,"%s", line);
    }

    /* Store the result of Deltac */
    if (mutual_mp_test_result->nOpenResult == 1) {
        //length += sprintf(SetCsvData+length, "DeltaC_Result:PASS\n");
        mp_test_info.allnode_test_result = 1;
    } else {
        if (mutual_mp_test_result->pCheck_Fail[0] == 1) {
            //length += sprintf(SetCsvData+length, "DeltaC_Result:FAIL\nFail Channel:");
            for (i = 0; i < MAX_MUTUAL_NUM; i++) {
                if (mutual_mp_test_result->pOpenFailChannel[i] == PIN_NO_ERROR)
                    continue;

                //length += sprintf(SetCsvData+length,"D%1d.S%2d", mutual_mp_test_result->pOpenFailChannel[i] % 100, mutual_mp_test_result->pOpenFailChannel[i] / 100);
            }
            //length += sprintf(SetCsvData+length,"%s", line);
        } else {
            //length+=sprintf(SetCsvData+length, "DeltaC_Result:PASS\n");
            mp_test_info.allnode_test_result = 1;
        }
    }

    /* Store Ratio data */
    for (j = 0; j < mutual_mp_test_data->sensorInfo.numSen; j++) {
        //length += sprintf(SetCsvData+length, ",S%d,", j+1);
        for (i = 0; i < mutual_mp_test_data->sensorInfo.numDrv; i++) {
            if (mutual_mp_test_result->pOpenResultData[j * mutual_mp_test_data->sensorInfo.numDrv + i] == NULL_DATA)    //for mutual key
            {
                //length+=sprintf(SetCsvData+length, "%s", ",");
            } else {
                //length+=sprintf(SetCsvData+length, "%1d,", mutual_mp_test_result->pGolden_CH_Max_Avg[j * mutual_mp_test_data->sensorInfo.numDrv + i]);
                mp_test_info.RationData[j * mutual_mp_test_data->sensorInfo.numDrv + i] =
                    mutual_mp_test_result->pGolden_CH_Max_Avg[j *
                                          mutual_mp_test_data->sensorInfo.numDrv +
                                          i];
            }
        }
        //length+=sprintf(SetCsvData+length,"%s", line);
    }

    /* Store the result of Ratio */
    if (mutual_mp_test_result->nOpenResult == 1) {
        //length += sprintf(SetCsvData+length, "Ratio_Result:PASS\n");
        mp_test_info.open_test_result = 1;
    } else {
        if (mutual_mp_test_result->pCheck_Fail[1] == 1) {
            //length += sprintf(SetCsvData+length, "Ratio_Result:FAIL\nFail Channel:");
            for (i = 0; i < MAX_MUTUAL_NUM; i++) {
                if (mutual_mp_test_result->pOpenFailChannel[i] == PIN_NO_ERROR)
                    continue;

                //length += sprintf(SetCsvData+length,"D%1d.S%2d", mutual_mp_test_result->pOpenRatioFailChannel[i] % 100, mutual_mp_test_result->pOpenRatioFailChannel[i] / 100);
            }
            //length += sprintf(SetCsvData+length,"%s", line);
        } else {
            //length += sprintf(SetCsvData+length, "Ratio_Result:PASS\n");
            mp_test_info.open_test_result = 1;
        }
    }

    /* Store the result of ITO short */
    if (mutual_mp_test_result->nShortResult == ITO_TEST_OK) {
        mp_test_info.short_test_result = 1;
        //length += sprintf(SetCsvData+length,"%s",  head);
    }

    /* Store ITO short data */
    for (i = 0; i < (mutual_mp_test_data->sensorInfo.numSen); i++) {
        mp_test_info.ShortData[i] = mutual_mp_test_result->pShortResultData[i];
    }

    for (i = 0; i < (mutual_mp_test_data->sensorInfo.numDrv); i++) {
        mp_test_info.ShortData[i + mutual_mp_test_data->sensorInfo.numSen] =
            mutual_mp_test_result->pShortResultData[i + mutual_mp_test_data->sensorInfo.numSen];
    }

    for (i = 0; i < (mutual_mp_test_data->sensorInfo.numGr); i++) {
        mp_test_info.ShortData[i + mutual_mp_test_data->sensorInfo.numSen +
                       mutual_mp_test_data->sensorInfo.numDrv] =
            mutual_mp_test_result->pShortResultData[i + mutual_mp_test_data->sensorInfo.numSen +
                                mutual_mp_test_data->sensorInfo.numDrv];
    }
}

void mstar_mp_test_save_data(int nChipType, int result)
{
    if (nChipType == CHIP_TYPE_ILI2117A || nChipType == CHIP_TYPE_ILI2118A || nChipType == CHIP_TYPE_MSG28XXA) {
        mstar_mp_convert_csv_file(result);
    } else {
        TS_LOG_ERR("%s: It doesn't support this chip type: 0x%x \n", __func__, nChipType);
    }

    return;
}
