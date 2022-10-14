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

#ifndef __MP_COMMON_H__
#define __MP_COMMON_H__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/vmalloc.h>

#include "../mstar_common.h"

// Chip Id
#define CHIP_TYPE_MSG21XX   (0x01)  // EX. MSG2133
#define CHIP_TYPE_MSG21XXA  (0x02)  // EX. MSG2133A/MSG2138A(Besides, use version to distinguish MSG2133A/MSG2138A, you may refer to _DrvFwCtrlUpdateFirmwareCash())
#define CHIP_TYPE_MSG26XXM  (0x03)  // EX. MSG2633M
#define CHIP_TYPE_MSG22XX   (0x7A)  // EX. MSG2238/MSG2256
#define CHIP_TYPE_MSG28XX   (0x85)  // EX. MSG2833/MSG2835/MSG2836/MSG2840/MSG2856/MSG5846
#define CHIP_TYPE_MSG28XXA   (0xBF) // EX. MSG2856
#define CHIP_TYPE_MSG58XXA  (0xBF)  // EX. MSG5846A
#define CHIP_TYPE_ILI2117A  (0x2117)    // EX. ILI2117A
#define CHIP_TYPE_ILI2118A  (0x2118)    // EX. ILI2118A
#define CHIP_TYPE_ILI2121   (0x2121)    // EX. ILI2121
#define CHIP_TYPE_ILI2120   (0x2120)    // (0) // EX. ILI2120

#define EMEM_SIZE_MSG28XX (1024*130)
#define EMEM_SIZE_MSG22XX ((1024*48) + 512 )
#define EMEM_TYPE_ALL_BLOCKS    0x00
#define EMEM_TYPE_MAIN_BLOCK    0x01
#define EMEM_TYPE_INFO_BLOCK    0x02

//MPTest Result Items
#define MPTEST_RESULT                   0
#define MPTEST_SCOPE                    1
#define OPEN_TEST_DATA                  2
#define OPEN_TEST_FAIL_CHANNEL          3
#define SHORT_TEST_DATA                 4
#define SHORT_TEST_FAIL_CHANNEL         5
#define WATERPROOF_TEST_DATA            6
#define WATERPROOF_TEST_FAIL_CHANNEL    7

//MPTest Result
#define ITO_NO_TEST                   0
#define ITO_TEST_OK                   1
#define ITO_TEST_FAIL                 2
#define ITO_TEST_GET_TP_TYPE_ERROR    3
#define ITO_TEST_UNDEFINED_ERROR      4
#define ITO_TEST_PROCESSING           5

//int[] location in Report(MPTEST_RESULT)
#define OPEN_TEST_RESULT          0
#define SHORT_TEST_RESULT         1
#define WATERPROOF_TEST_RESULT    2

#define ERR_ALLOC_MEM(X)    ((IS_ERR(X) || X == NULL) ? 1 : 0)

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define NULL_DATA -3240
#define PIN_NO_ERROR 0xFFFF
#define UN_USE_SENSOR 0x5AA5

#define IIR_MAX 32600
#define PIN_UN_USE 0xABCD

#define MAX_CHANNEL_DRV  30
#define MAX_CHANNEL_SEN  20
#define MAX_CHANNEL_NUM_28XX    60
#define MAX_CHANNEL_NUM_30XX    49
#define MAX_AFE_NUM_30XX    14
#define MAX_AFE_NUM_28XX    13
#define TEST_ITEM_NUM 8
#define MAX_MUTUAL_NUM  1904    // Camaro : 14AFE * 17DRI * 8SF, Cayenne: 13AFE * 15DRI * 6SF

#define KEY_SEPERATE    0x5566
#define KEY_COMBINE     0x7788

#define BUFFER_NULL_LEN     0

typedef enum {
    ONE_DAC_ENABLE = 0,
    TWO_DAC_ENABLE = 1,
} ItoTestDACStatus;

typedef enum {
    SINGLE = 0,
    MULTI = 1,
} ItoTestScreenType;

typedef enum {
    eDAC_0,
    eDAC_1,
} ItoTestDAC;

typedef enum {
    _Msg30xx50p,
    _Msg30xx45p,
    _Msg30xx35p,
    _Msg30xx30p,
    _Msg30xx25p
} ItoTestMsg30xxPin;

typedef struct {
    u16 X;
    u16 Y;
} MutualMapping_t;

typedef struct {
    char *sSupportIC;
    int bmDKVerify;
    int bCurrentTest;
    int bChipVerify;
    int bFWUpdate;
    int bFWTest;
    int bOpenTest;
    int bShortTest;
    int bWpTest;
    int bFunctionTest;
    int bAutoStart;
    int bAutoMation;
    int bTriggerMode;
    int bTSMode;
    int bTSEnable;
    int bPhaseKTest;
} MutualUIConfig_t;

typedef struct {
    u16 persentDC_VA_Range;
    u16 persentDC_VA_Ratio;

    u16 persentDC_Border_Ratio;
    u16 persentDC_VA_Range_up;
    u16 persentDC_VA_Ratio_up;

    u16 persentDG_Range;
    u16 persentDG_Ratio;
    u16 persentDG_Range_up;
    u16 persentDG_Ratio_up;
    u16 persentWater_DG_Range;
} MutualToast_t;

typedef struct {
    u16 numKey;
    u16 numKeyLine;
    u16 numDummy;
    u16 numTriangle;
    u16 KeyDrv;
    u16 KEY_CH;
    u16 KeyDrv_o;
    char *key_type;
    int thrsShort;
    int thrsICpin;
    int thrsOpen;
    int thrsWater;

    u16 numSen;
    u16 numDrv;
    u16 numGr;
    MutualMapping_t *mapping;
} MutualSensor_t;

typedef struct {
    MutualUIConfig_t UIConfig;
    int logResult;
    int logFWResult;

    int Enable;

    char *ana_version;
    char *project_name;
    char *binname;
    char *versionFW;
    u16 slaveI2cID;
    char *stationNow;
    char *inipassword;
    u16 Mutual_Key;
    u16 Pattern_type;
    u16 Pattern_model;

    int Crc_check;

    MutualSensor_t sensorInfo;
    MutualToast_t ToastInfo;
    int FPC_threshold;
    int KeyDC_threshold;
    int KEY_Timer;
    int Open_test_csub;
    int Open_test_cfb;
    int Open_mode;
    int Open_fixed_carrier;
    int Open_fixed_carrier1;
    int Open_test_chargepump;
    int inverter_mode;
    int Current_threshold;
    int CurrentThreshold_Powerdown;

    int OPEN_Charge;
    int OPEN_Dump;
    int SHORT_Charge;
    int SHORT_Dump1;
    int SHORT_Dump2;
    int Water_Charge;
    int Water_Dump;

    int *KeySen;
    int *Goldensample_CH_0_Max_Avg; ///[ana26xxM.MAX_MUTUAL_NUM];
    int *Goldensample_CH_0_Max; ///[ana26xxM.MAX_MUTUAL_NUM];
    int *Goldensample_CH_0; ///[ana26xxM.MAX_MUTUAL_NUM];
    int *Goldensample_CH_0_Min; ///[ana26xxM.MAX_MUTUAL_NUM];
    int *Goldensample_CH_0_Min_Avg; ///[ana26xxM.MAX_MUTUAL_NUM];

    int *PhaseGolden_Max;   ///[ana26xxM.MAX_MUTUAL_NUM];
    int *PhaseGolden;   ///[ana26xxM.MAX_MUTUAL_NUM];
    int *PhaseGolden_Min;   ///[ana26xxM.MAX_MUTUAL_NUM];

    int *PhaseWaterGolden_Max;
    int *PhaseWaterGolden;
    int *PhaseWaterGolden_Min;

    u16 *PAD2Sense;
    u16 *PAD2Drive;
    u16 *PAD2GR;

    u16 *phase_freq;
    u16 freq_num;
    u16 phase_time;
    u16 band_num;
    u16 *pgd;
    u16 *water_pgd;
    u16 *water_sense;
    u16 *water_drive;
    u16 charge_pump;
    u16 raw_type;
    u16 noise_thd;
    u16 sub_frame;
    u16 afe_num;
    u16 phase_sen_num;
    u16 *phase_sense;
    u16 water_sen_num;
    u16 water_drv_num;
    int update_bin;
    int force_phaseK;
    int update_info;
    int log_phasek;
    int border_drive_phase;
    int sw_calibration;
    u8 phase_version;
    u8 mapping_version;
    int deep_standby;
    int deep_standby_timeout;
    int Open_KeySettingByFW;
} MutualMpTest_t;

typedef struct {
    int nOpenResult;
    int nShortResult;
    int nRetry;

    int nRatioAvg_max;
    int nRatioAvg_min;
    int nBorder_RatioAvg_max;
    int nBorder_RatioAvg_min;

    int *pOpenResultData;
    int *pOpenFailChannel;
    int *pOpenRatioFailChannel;
    int *pShortResultData;
    int *pShortRData;
    int *pICPinShortResultData;
    int *pICPinShortRData;
    int *pICPinChannel;
    int *pShortFailChannel;
    int *pICPinShortFailChannel;
    //int * pWaterProofResultData;
    //int * pWaterProofFailChannel;

    int *pCheck_Fail;
    int *pResult_DeltaC;
    int *pGolden_CH_Max_Avg;
    int *pGolden_CH_Min_Avg;
    int *pGolden_CH_Max;
    int *pGolden_CH_Min;
    int *pGolden_CH;
    char *mapTbl_sec;
} MutualMpTestResult_t;

struct mstar_tp_info {
    char DriverVersion[32];
    char FwVersion[32];
    char PlatformVersion[32];
    char MainBlockFWVersion[32];
    char InfoBlockFWVersion[32];
    char ChipType;
};

extern struct mstar_tp_info tpinfo;
extern MutualMpTest_t *mutual_mp_test_data;
extern MutualMpTestResult_t *mutual_mp_test_result;

extern s32 N1_TEST_PIN[13];
extern u16 N1_MUX_MEM_20_3E[16];
extern s32 N2_TEST_PIN[12];
extern u16 N2_MUX_MEM_20_3E[16];
extern s32 S1_TEST_PIN[7];
extern u16 S1_MUX_MEM_20_3E[16];
extern s32 S2_TEST_PIN[6];
extern u16 S2_MUX_MEM_20_3E[16];

extern u16 g_fout_base_addr;
extern u16 g_scan_mode;
extern u16 g_code_type;
extern u16 g_two_dac_enable;
extern u16 g_sqrt_en;

extern u32 SLAVE_I2C_ID_DBBUS;
extern u32 SLAVE_I2C_ID_DWI2C;

// declaired at MsgControl.c
extern void mstar_mp_dbbus_enter(void);
extern void mstar_mp_exit_dbbus(void);
extern void mstar_mp_start_mcu(void);
extern void mstar_mp_stop_mcu(void);
extern void mstar_dbbus_stop_mcu(void);
extern s32 mstar_enter_mp_mode(void);
extern int mstar_mp_read_flash(u8 nChipType, u32 nAddr, int nBlockType, int nLength, u8 * pFlashData);
extern void mstar_reg_get_16bit_byte_value_buf(u16 nAddr, u8 * pBuf, u16 nLen);
extern void mstar_mp_debug_show_array2(void *pBuf, u16 nLen, int nDataType, int nCarry, int nChangeLine);

// declaired at MsgLib.c
extern int mstar_mp_get_fw_ver_on_flash(void);
extern int mstar_mp_test_entry(int nChipType, char *FilePath);
extern void mstar_mp_test_entry_end(int nChipType);

extern void mstar_mp_test_save_data(int nChipType, int result);

extern u8 mstar_check_double_value_in_range(s32 nValue, s32 nMax, s32 nMin);
extern u8 mstar_check_value_in_range(s32 nValue, s32 nMax, s32 nMin);
extern int str_to_hex(char *hex_str);

extern int mstar_mp_load_ini(char *pFilePath);
extern int mstar_mp_test_start(void);
extern int mstar_mp_test_result(int nResultType, int *pResult);
extern void mstar_mp_test_end(void);

extern s32 mstar_mp_open_judge(u16 nItemID, s8 * pNormalTestResult, u16 * pNormalTestResultCheck);
extern int mstar_mp_open_test(u16 fw_ver);
extern void Msg30xxSetCfb(u8 Cfb);

extern s32 mstar_mp_short_test_judge(u8 nItemID, s8 * TestFail, u16 TestFail_check[][MAX_MUTUAL_NUM]);
extern int mstar_mp_convert_R_value(int dump_time, s32 deltaR);
extern int mstar_mp_short_test_start(void);

extern void mstar_mp_dbbus_re_enter(void);
extern void mstar_mp_ddbus_read_dq_mem_start(void);
extern void mstar_mp_dbbus_read_dq_mem_end(void);
extern void mstar_mp_enable_adc_one_shot(void);
extern void mstar_mp_ana_change_cd_time(u16 T1, u16 T2);
//extern s32 Msg30xxTriggerWaterProofOneShot(s16 * pResultData, int *nSize);
extern s32 mstar_mp_trigger_mutual_one_shot(s16 * pResultData, u16 * pSenNum, u16 * pDrvNum, u16 drv_mode);
extern s32 mstar_mp_get_mutual_one_shot_raw_iir(s16 * nResultData, u16 * pSenNum, u16 * pDrvNum, u16 drv_mode);
//extern s32 Msg30xxtGetWaterProofOneShotRawIIR(s16 * pRawDataWP, int *nSize);
extern s32 mstar_mp_check_switch_status(void);
extern s32 mstar_mp_switch_fw_mode(u16 * nFMode, u16 * deep_standby);
extern void mstar_mp_ana_sw_reset(void);
extern void mstar_mp_ana_fix_prs(u16 option);
extern s32 mstar_enter_mp_mode_30xx(void);
extern u16 msg30xx_get_drv_opening(void);
extern void msg30xx_scan_dac_setting(u16 nTwoDAC);
extern void msg30xx_Tgen_Ovwr_dac_enable(u16 nEnable, u16 eDACSelect);
extern void msg30xx_Tgen_Ovwr_reg_en(u16 nEnable);
extern void msg30xx_Tgen_Ovwr_invert_mode(u16 nEnable, u16 nTwoDAC);
extern void msg30xx_Tgen_Ovwr_DrvL_Buf_cfg_setting(u16 nEnable, u16 nTwoDAC);
extern void msg30xx_Tgen_Ovwr_DrvL_Buf_gain_setting(u16 nEnable, u16 nTwoDAC);
extern void msg30xx_Tgen_Ovwr_DrvH_comp_setting(u16 nEnable);
extern void msg30xx_Tgen_Ovwr_Ross_select(u16 nEnable, u16 nTwoDAC);
extern void Msg30xxTgenOvwrDRVHvSetting(u16 nEnable);
extern void msg30xx_Tgen_Ovwr_charge_pump_setting(u16 nEnable_charge_pump);
extern void msg30xx_ana_enable_charge_pump(u16 nEnable_charge_pump);
extern void msg30xx_gain_setting(void);
extern void msg30xx_set_sensor_pad_state(u16 state);
extern void msg30xx_scan_set_sensor_num(u16 nSensorNum);
extern void msg30xx_scan_set_subframe_num(u16 nSubframeNum);
extern void msg30xx_scan_set_sample_num(u16 nSampleNum);
extern void msg30xx_set_chip_swap_enable(u16 nSwapEnable);
extern void msg30xx_adc_desp_invert_enable(u16 nEnable);
extern void msg30xx_set_mutual_csub_via_dbbus(s16 nCSub);

extern int new_flow_main(int item);
#endif
