/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include "kd_imgsensor.h"
#include "imgsensor_sensor_list.h"

/* Add Sensor Init function here
* Note:
* 1. Add by the resolution from ""large to small"", due to large sensor
*    will be possible to be main sensor.
*    This can avoid I2C error during searching sensor.
* 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
*/
struct IMGSENSOR_INIT_FUNC_LIST kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR] = {
#if defined(HI1333_MIPI_RAW)
	{HI1333_SENSOR_ID, SENSOR_DRVNAME_HI1333_MIPI_RAW, HI1333_MIPI_RAW_SensorInit},
#endif
#if defined(S5K3L6_MIPI_RAW)
	{S5K3L6_SENSOR_ID, SENSOR_DRVNAME_S5K3L6_MIPI_RAW, S5K3L6_MIPI_RAW_SensorInit},
#endif
#if defined(OV13855_MIPI_RAW)
	{OV13855_SENSOR_ID, SENSOR_DRVNAME_OV13855_MIPI_RAW, OV13855_MIPI_RAW_SensorInit},
#endif
#if defined(HI846_MIPI_RAW)
	{HI846_SENSOR_ID, SENSOR_DRVNAME_HI846_MIPI_RAW, HI846_MIPI_RAW_SensorInit},
#endif
#if defined(OV8856_MIPI_RAW)
	{OV8856_SENSOR_ID, SENSOR_DRVNAME_OV8856_MIPI_RAW, OV8856_MIPI_RAW_SensorInit},
#endif
#if defined(GC8034_MIPI_RAW)
	{GC8034_SENSOR_ID, SENSOR_DRVNAME_GC8034_MIPI_RAW, GC8034MIPI_RAW_SensorInit},
#endif
#if defined(GC8034A_MIPI_RAW)
    {GC8034A_SENSOR_ID, SENSOR_DRVNAME_GC8034A_MIPI_RAW, GC8034AMIPI_RAW_SensorInit},
#endif
#if defined(HI556_MIPI_RAW)
	{HI556_SENSOR_ID, SENSOR_DRVNAME_HI556_MIPI_RAW, HI556_MIPI_RAW_SensorInit},
#endif
#if defined(GC5025A_MIPI_RAW)
	{GC5025A_SENSOR_ID, SENSOR_DRVNAME_GC5025A_MIPI_RAW, GC5025AMIPI_RAW_SensorInit},
#endif
#if defined(GC5025H_MIPI_RAW)
    {GC5025H_SENSOR_ID, SENSOR_DRVNAME_GC5025H_MIPI_RAW, GC5025HMIPI_RAW_SensorInit},
#endif
#if defined(OV5675_MIPI_RAW)
	{OV5675_SENSOR_ID, SENSOR_DRVNAME_OV5675_MIPI_RAW, OV5675_MIPI_RAW_SensorInit},
#endif
#if defined(IMX338_MIPI_RAW)
	{IMX338_SENSOR_ID, SENSOR_DRVNAME_IMX338_MIPI_RAW, IMX338_MIPI_RAW_SensorInit},
#endif
	/*  ADD sensor driver before this line */
	{0, {0}, NULL}, /* end of list */
};
/* e_add new sensor driver here */

