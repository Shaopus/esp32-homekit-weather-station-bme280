#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

IOT_COMPONENT_DIRS += components
IOT_COMPONENT_DIRS += components/i2c_devices
IOT_COMPONENT_DIRS += components/i2c_devices/sensor
IOT_COMPONENT_DIRS += components/i2c_devices/others
IOT_COMPONENT_DIRS += components/general

EXTRA_COMPONENT_DIRS += $(IOT_COMPONENT_DIRS)
