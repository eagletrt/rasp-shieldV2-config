#!/bin/sh

# CAN Config
# can_interface spi_num chip_select interrupt_num
# spi=spi{spi_num}.{chip_select} e.g. spi0.0, spi0.1, spi1.0, spi1.1
# interrupt_num is the GPIO pin number used for the interrupt of the can controller, e.g. gpio25 -> 25

# CAN 0 Config
can_interface="can0"
spi_num=0
chip_select=0
gpio_interrupt=25
can0_config="$can_interface $spi_num $chip_select $gpio_interrupt"

# CAN 1 Config
can_interface="can1"
spi_num=0
chip_select=1
gpio_interrupt=24
can1_config="$can_interface $spi_num $chip_select $gpio_interrupt"

# CAN 2 Config
can_interface="can2"
spi_num=1
chip_select=0
gpio_interrupt=23
can2_config="$can_interface $spi_num $chip_select $gpio_interrupt"

# CAN Configs Array
can_configs=("$can0_config" "$can1_config" "$can2_config")

cd mcp2515-overlays
if [ $? -ne 0 ]; then
    echo "Error: Could not enter directory to mcp2515-overlays."
    exit 1
fi
if [ ! -f "overlay_maker.sh" ]; then
    echo "Error: Could not find overlay_maker.sh script."
    exit 1
fi
# Create the overlay files for each CAN interface
for config in "${can_configs[@]}"; do
    ./overlay_maker.sh $config
    if [ $? -ne 0 ]; then
        echo "Error: Could not create overlay file for config: $config"
        exit 1
    fi
done

