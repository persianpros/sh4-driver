################
# stfrontend.lib
################

DIR_NAME = stv0913_lla
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


 # -- sat stv0913 driver set --
OBJS += stv0913_init.tco stv0913_drv.tco 
     

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
