#!/bin/sh

# CAN Config
# can_interface spi_num chip_select interrupt_num
# spi=spi{spi_num}.{chip_select} e.g. spi0.0, spi0.1, spi1.0, spi1.1
# interrupt_num is the GPIO pin number used for the interrupt of the can controller, e.g. gpio25 -> 25

# CAN 0 Config
can0_config="can0 0 0 25"

# CAN 1 Config
can1_config="can1 0 1 24"

# CAN 2 Config
can2_config="can2 1 0 23"

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
done

