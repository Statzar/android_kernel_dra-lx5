# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/include

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_ACCELHUB) += accelhub/
obj-$(CONFIG_MTK_MC3433)   += mc3433/
obj-$(CONFIG_MTK_LSM6DS3HG)+= lsm6ds3hg/
obj-$(CONFIG_MTK_LIS3DH)   += lis3dh/
obj-$(CONFIG_MTK_BMI160_I2C)   += bmi160-i2c/
obj-$(CONFIG_MTK_MC3410_I2C) += mc3410-i2c/
obj-$(CONFIG_MTK_MPU6050G)   += mpu6050g/
# +add by ontim
obj-$(CONFIG_MTK_MC3410_ONTIM)   +=  mc3410-ontim/
obj-$(CONFIG_MTK_KXTJ2_1009_ONTIM)   +=  KXTJ2_1009-ontim/
obj-$(CONFIG_MTK_KX126_1063)   +=  KX126_1063/
obj-$(CONFIG_MTK_BMA455)   +=  bma455/
# -add by ontim
obj-$(CONFIG_MTK_LIS3DH_NEW) += lis3dh/
