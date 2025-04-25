CRAZYFLIE_BASE := crazyflie-firmware

OOT_CONFIG := $(PWD)/app-config

include radio.config

OOT_ARGS := -C $(CRAZYFLIE_BASE) OOT=$(PWD) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" CLOAD_CMDS="-w radio://0/$(RADIO_CHANNEL)/2M"

MAKEFLAGS += -j$(shell nproc)

include $(CRAZYFLIE_BASE)/tools/make/oot.mk