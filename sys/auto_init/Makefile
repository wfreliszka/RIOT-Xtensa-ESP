ifneq (,$(filter gnrc_netif_init,$(USEMODULE)))
  DIRS += netif
endif

ifneq (,$(filter auto_init_l3gd20h,$(USEMODULE)))
  DIRS += l3gd20h
endif

ifneq (,$(filter auto_init_can,$(USEMODULE)))
  DIRS += can
endif

ifneq (,$(filter auto_init_loramac,$(USEMODULE)))
  DIRS += loramac
endif

ifneq (,$(filter auto_init_multimedia,$(USEMODULE)))
  DIRS += multimedia
endif

ifneq (,$(filter auto_init_usbus,$(USEMODULE)))
  DIRS += usb
endif

ifneq (,$(filter auto_init_security,$(USEMODULE)))
  DIRS += security
endif

include $(RIOTBASE)/Makefile.base
