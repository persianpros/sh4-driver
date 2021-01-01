ifeq ($(CONFIG_CPU_SUBTYPE_STXH205),y)

include $(STG_TOPDIR)/linux/kernel/drivers/stm/coredisplay/coredisplay.mak

STIH205SRCS = $(STM_HDF_TVOUT) $(STM_HDF_HDMI) $(STM_HDMI_COMMON) $(STM_HDMI_DMA_IFRAME) $(STM_BDISP) $(STM_DEI_IQI)

STIH205SRCS += $(addprefix $(SRC_TOPDIR)/Gamma/, GDPBDispOutput.cpp)

STIH205SRCS += $(addprefix $(SRC_TOPDIR)/soc/stih205/,                         \
			stih205gdp.cpp                                         \
			stih205bdisp.cpp                                       \
			stih205mixer.cpp                                       \
			stih205dvo.cpp                                         \
			stih205mainoutput.cpp                                  \
			stih205auxoutput.cpp                                   \
			stih205hdmi.cpp                                        \
			stih205device.cpp                                      \
			stih205devicecreate.cpp)

stih205-display-objs := $(patsubst %.cpp,%.o,$(ALLSRCS) $(STIH205SRCS))
stih205-display-objs := $(patsubst %.c,%.o,$(stih205-display-objs))

obj-m += stmcore-display-stih205.o

stmcore-display-stih205-objs := $(stih205-display-objs) stih205.o

endif

