#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp32-homekit-weather-station-bme280

PROJECT_VER := $( git describe --always --tags --dirty)
CPPFLAGS := -D PROJECT_VER=\"$(PROJECT_VER)\"

CFLAGS += -DHOMEKIT_SHORT_APPLE_UUIDS

include ./components/component_conf.mk
include $(IDF_PATH)/make/project.mk


