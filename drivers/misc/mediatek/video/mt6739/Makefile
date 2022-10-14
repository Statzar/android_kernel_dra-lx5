#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

################################################################################
# Platform Dependent
#
#
################################################################################

obj-$(CONFIG_MTK_FB) += dispsys/
obj-$(CONFIG_MTK_FB) += videox/
ifeq ($(CONFIG_LCDKIT_DRIVER),y)
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/include
#EXTRA_CFLAGS += -Idrivers/devkit/tpkit
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/core/common/mtk
endif
