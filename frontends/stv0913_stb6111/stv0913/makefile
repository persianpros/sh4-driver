################
# stfrontend.lib
################

# Add for make system version 2 support
DVD_MAKE_VERSION := 2
ifdef IN_OBJECT_DIR
# Pick up default rvariables
include $(DVD_MAKE)/generic.mak

# Other components this component is dependant upon
HEADER_IMPORTS = stos

# Build targets
ST20_TARGETS = stv0913$(LIB_SUFFIX)
ST40_TARGETS := $(LIB_PREFIX)stv0913$(LIB_SUFFIX)
ST200_TARGETS := $(ST40_TARGETS)

SUBDIRS += stv0913_lla

 # -- sat STV0913 driver set --
  OBJS += d0913.tco #AG removed drv & util tco    

ifeq "$(DVD_STFRONTEND_DRV_COVERITY_TEST)" "YES"
SUBDIRS += stv0913_lla  
endif     

ST20_OBJS := $(OBJS)
ST40_OBJS := $(call ST20OBJ_TO_ST40OBJ,$(OBJS))
ST200_OBJS := $(call ST20OBJ_TO_ST200OBJ,$(OBJS))

CFLAGS += $(STFRONTEND_CFLAGS)
# Default rules
include $(DVD_MAKE)/defrules.mak


$(LIB_PREFIX)stv0913$(LIB_SUFFIX): $($(ARCHITECTURE)_OBJS) $(SUBDIR_LIBS)
	@-$(RM) *$(LIB_SUFFIX)
	@$(ECHO) Building $@
	$(BUILD_LIBRARY)

# -- clean commands ---
clean: $(SUBDIR_CLEAN)
	@$(ECHO) Cleaning $($(ARCHITECTURE)_TARGETS)
	-$(RM) $($(ARCHITECTURE)_TARGETS)
	-$(RM) $($(ARCHITECTURE)_OBJS)

else

include $(DVD_MAKE)/builddir.mak

endif


#EOF
