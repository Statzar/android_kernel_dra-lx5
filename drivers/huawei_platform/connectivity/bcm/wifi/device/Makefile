EXTRA_CFLAGS += -Idrivers/huawei_platform/connectivity/bcm/wifi/include

ifeq ($(CONFIG_DHD_STATIC_KO),y)
  obj-m += dev_wifi.o
else
  obj-y +=dev_wifi.o
endif
ccflags-y += -DCONFIG_DHD_USE_STATIC_BUF
ccflags-y += -DHW_SOFTAP_THROUGHPUT_OPTIMIZE
ccflags-y += -DHW_WIFI_DRIVER_NORMALIZE
ccflags-y += -DBCM_ALLOC_STATIC_10K
ccflags-y += -DDHD_DEVWAKE_EARLY
