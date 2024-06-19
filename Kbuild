EXTRA_CFLAGS := -I$(src)/core_driver
EXTRA_CFLAGS += -I$(src)
EXTRA_CFLAGS += -Wno-unused-function # -Wextra
obj-$(CONFIG_SENSORS_TMF8806) += tmf8806.o
tmf8806-y = tmf8806_driver.o ./core_driver/tmf8806.o ams_i2c.o tmf8806_shim.o tmf8806_hex_interpreter.o
