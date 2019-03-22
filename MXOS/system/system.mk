#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME = Lib_MXOS_System


$(NAME)_SOURCES := mxos_system_init.c \
                   mxos_system_monitor.c \
                   mxos_system_notification.c \
                   mxos_system_para_storage.c \
                   mxos_system_time.c \
                   mxos_system_power_daemon.c \
                   mxos_filesystem.c \
                   mxos_station_monitor.c \
                   system_misc.c 

$(NAME)_SOURCES  += command_console/mxos_cli.c
$(NAME)_INCLUDES += command_console

$(NAME)_SOURCES += config_server/config_server_menu.c \
                   config_server/config_server.c
                   
$(NAME)_SOURCES += easylink/system_easylink_delegate.c \
                   easylink/system_easylink_wac.c \
                   easylink/system_easylink_monitor.c \
                   easylink/system_easylink_softap.c \
                   easylink/system_aws.c \
                   easylink/internal/easylink_bonjour.c
                   
$(NAME)_INCLUDES += easylink/internal


$(NAME)_SOURCES += mdns/system_discovery.c
                               
$(NAME)_SOURCES += tftp_ota/tftp_ota.c \
                   tftp_ota/tftpc.c
$(NAME)_INCLUDES += tftp_ota
                   
$(NAME)_COMPONENTS := protocols/mdns \
                      system/qc_test \
                      system/easylink/MFi_WAC \
                      system/kv

ifneq ($(DISABLE_MXOS_AWS),1)
$(NAME)_COMPONENTS += system/easylink/aws
$(NAME)_DEFINES += CONFIG_MXOS_AWS
endif

