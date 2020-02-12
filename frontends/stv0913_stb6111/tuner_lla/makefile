################
# stfrontend.lib
################

DIR_NAME = tuner_lla
# Add for make system version 2 support
DVD_MAKE_VERSION := 2
ifdef IN_OBJECT_DIR
# Pick up default rvariables
include $(DVD_MAKE)/generic.mak

# Other components this component is dependant upon
HEADER_IMPORTS = stos


# Build targets
ST20_TARGETS = $(DIR_NAME)$(LIB_SUFFIX)
ST40_TARGETS := $(LIB_PREFIX)$(DIR_NAME)$(LIB_SUFFIX)
ST200_TARGETS := $(ST40_TARGETS)


# Options to be set for Satellite in driver and application 
# depending on Device selection like STB6110


  
 ifeq "$(DVD_STFRONTEND_TUNER_STV6110)" "YES"
  OBJS += stv6110_tuner.tco
 endif
 
  ifeq "$(DVD_STFRONTEND_TUNER_STV6111)" "YES"
  OBJS += stv6111_tuner.tco
 endif
 
  ifeq "$(DVD_STFRONTEND_TUNER_STV6140)" "YES"
  OBJS += stv6140_tuner.tco
 endif
 
 ifeq "$(DVD_STFRONTEND_TUNER_STV6120)" "YES"
  OBJS += stv6120_tuner.tco
 endif
 
 ifeq "$(DVD_STFRONTEND_TUNER_STV6130)" "YES"
  OBJS += stv6130_tuner.tco
 endif
 
 ifeq "$(DVD_STFRONTEND_TUNER_STB6100)" "YES"
  OBJS += stb6100_tuner.tco
 endif
 
 ifeq "$(DVD_STFRONTEND_TUNER_STB6000)" "YES"
  OBJS += stb6000_tuner.tco
 endif

 ifeq "$(DVD_STFRONTEND_TUNER_SAT_NULL)" "YES"
   OBJS += sat_null_tuner.tco
 endif
 
 ifeq "$(DVD_STFRONTEND_DRV_COVERITY_TEST)" "YES"
   OBJS += stv6110_tuner.tco
   OBJS += stv6111_tuner.tco
   OBJS += stb6100_tuner.tco
   OBJS += stv6130_tuner.tco
   OBJS += stv6140_tuner.tco
   OBJS += sat_null_tuner.tco
 endif
  #vglna will always be compiled with every tuner
 OBJS += stvvglna.tco 
 OBJS += tunerlla_wrapper.tco
 
ST20_OBJS := $(OBJS)
ST40_OBJS := $(call ST20OBJ_TO_ST40OBJ,$(OBJS))
ST200_OBJS := $(call ST20OBJ_TO_ST200OBJ,$(OBJS))

CFLAGS += $(STFRONTEND_CFLAGS)
# Default rules
include $(DVD_MAKE)/defrules.mak


$(LIB_PREFIX)$(DIR_NAME)$(LIB_SUFFIX): $($(ARCHITECTURE)_OBJS)
	@-$(RM) *$(LIB_SUFFIX)
	@$(ECHO) Building $@
	$(BUILD_LIBRARY)

# -- clean commands ---

else

include $(DVD_MAKE)/builddir.mak

endif


#EOF
