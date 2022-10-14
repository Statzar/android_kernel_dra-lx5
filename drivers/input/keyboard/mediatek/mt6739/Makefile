subdir-ccflags-y += -Werror -I$(srctree)/drivers/input/keyboard/mediatek/

ifeq ($(CONFIG_MT_SND_SOC_NEW_ARCH),y)
	EXTRA_CFLAGS += -I$(srctree)/sound/soc/mediatek/common/
endif

ifeq ($(strip $(D85_RUNIN)),yes)
KBUILD_CFLAGS +=-DD85_RUNIN
endif

ifeq ($(strip $(D85_RUNIN)),yes)
KBUILD_CFLAGS +=-DD85_RUNIN
endif


obj-$(CONFIG_KEYBOARD_MTK) := hal_kpd.o
