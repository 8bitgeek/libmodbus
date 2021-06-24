# ** LIBMODBUS
# SRC_LIBMODBUS=${ENV_LIBMODBUS_ROOT}
SRC_LIBMODBUS=${SRC_ROOT}/libmodbus/src
INC += 	-I $(SRC_LIBMODBUS)

SRCS_CC += $(SRC_LIBMODBUS)/modbus.c
SRCS_CC += $(SRC_LIBMODBUS)/modbus-data.c
SRCS_CC += $(SRC_LIBMODBUS)/modbus-rtu-caribou.c
# SRCS_CC += $(SRC_LIBMODBUS)/modbus-tcp-caribou.c
