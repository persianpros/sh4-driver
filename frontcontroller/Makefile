TREE_ROOT?=$(DRIVER_TOPDIR)/
CONFIG_STGFB_PATH?=$(DRIVER_TOPDIR)/stgfb/stmfb
CCFLAGSY += -I$(TREE_ROOT)/player2/linux/include/linux/stm
CCFLAGSY += -I$(TREE_ROOT)/player2/linux/drivers/media/video/stm
CCFLAGSY += -I$(KERNEL_LOCATION)/drivers/media/dvb/dvb-core
CCFLAGSY += -I$(CONFIG_STGFB_PATH)/include
CCFLAGSY += -DDEBUG_DVB
CCFLAGSY += -I$(TREE_ROOT)/include
CCFLAGSY += -I$(TREE_ROOT)/frontcontroller/include

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

ifdef UFS910
obj-y += vfd/
endif

ifdef HOMECAST5101
obj-y += vfd_hs5101/
endif

ifdef UFS912
obj-y += kathrein_micom/
endif

ifdef UFS913
obj-y += kathrein_micom/
endif

ifdef UFS922
obj-y += kathrein_micom/
endif

ifdef UFC960
obj-y += kathrein_micom/
endif

ifdef TF7700
obj-y += tffp/
endif

ifdef HL101
obj-y += aotom_hl101/
endif

ifdef VIP1_V2
obj-y += aotom_vip/
endif

ifdef VIP2_V1
obj-y += aotom_vip/
endif

ifdef ADB_BOX
obj-y += adb_5800_fp/
endif

ifdef ADB_2850
obj-y += adb_2850_fp/
endif

ifdef SPARK
obj-y += aotom_spark/
endif

ifdef SPARK7162
obj-y += aotom_spark/
endif

ifdef OCTAGON1008
obj-y += nuvoton/
endif

ifdef FORTIS_HDBOX
obj-y += nuvoton/
endif

ifdef ATEVIO7500
obj-y += nuvoton/
endif

ifdef HS7110
obj-y += nuvoton/
endif

ifdef HS7119
obj-y += nuvoton/
endif

ifdef HS7420
obj-y += nuvoton/
endif

ifdef HS7429
obj-y += nuvoton/
endif

ifdef HS7810A
obj-y += nuvoton/
endif

ifdef HS7819
obj-y += nuvoton/
endif

ifdef FOREVER_NANOSMART
obj-y += fortis_4g_fp/
endif

ifdef FOREVER_9898HD
obj-y += fortis_4g_fp/
endif

ifdef DP7001
obj-y += fortis_4g_fp/
endif

ifdef FOREVER_3434HD
obj-y += fortis_4g_fp/
endif

ifdef FOREVER_2424HD
obj-y += fortis_4g_fp/
endif

ifdef GPV8000
obj-y += fortis_4g_fp/
endif

ifdef EP8000
obj-y += fortis_4g_fp/
endif

ifdef EPP8000
obj-y += fortis_4g_fp/
endif

ifdef ATEMIO520
obj-y += cn_micom/
endif

ifdef ATEMIO530
obj-y += cn_micom/
endif

ifdef IPBOX9900
obj-y += ipbox99xx/
endif

ifdef IPBOX99
obj-y += ipbox99xx/
endif

ifdef IPBOX55
obj-y += ipbox55/
endif

ifdef CUBEREVO
obj-y += ipbox/
else ifdef CUBEREVO_MINI
obj-y += ipbox/
else ifdef CUBEREVO_MINI2
obj-y += ipbox/
else ifdef CUBEREVO_250HD
obj-y += ipbox/
else ifdef CUBEREVO_9500HD
obj-y += ipbox/
else ifdef CUBEREVO_2000HD
obj-y += ipbox/
else ifdef CUBEREVO_MINI_FTA
obj-y += ipbox/
else ifdef CUBEREVO_3000HD
obj-y += ipbox/
endif

ifdef VITAMIN_HD5000
obj-y += vitamin_hd5000/
endif

ifdef SAGEMCOM88
obj-y += front_led/
obj-y += front_vfd/
endif

ifdef ARIVALINK200
ifneq (,$(wildcard $(DRIVER_TOPDIR)/frontcontroller/front_ArivaLink200 ))
obj-y += front_ArivaLink200/
endif
endif

ifdef PACE7241
obj-y += pace_7241_fp/
endif
