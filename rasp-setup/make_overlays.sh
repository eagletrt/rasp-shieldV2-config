#!/bin/sh

# Boot Firmware Config File
config_file="config.txt"

# CAN Config
# can_interface spi_num chip_select interrupt_num
# spi=spi{spi_num}.{chip_select} e.g. spi0.0, spi0.1, spi1.0, spi1.1
# interrupt_num is the GPIO pin number used for the interrupt of the can controller, e.g. gpio25 -> 25

if [ ${#@} -ne 0 ] && [ $((${#@} % 4)) -eq 0 ]; then
    i=1
    while [ $i -le ${#@} ]; do
        eval "can_interface=\${$i}"
        eval "spi_num=\${$((i + 1))}"
        eval "chip_select=\${$((i + 2))}"
        eval "gpio_interrupt=\${$((i + 3))}"
        can_configs+=("$can_interface $spi_num $chip_select $gpio_interrupt")
        i=$((i + 4))
    done
else
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
fi

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
cd ..
if [ $? -ne 0 ]; then
    echo "Error: Could not return to parent directory after creating overlays."
    exit 1
fi
# Create config file for CAN interfaces
echo "# For more options and information see
# http://rptl.io/configtxt
# Some settings may impact device functionality. See link above for details

# Uncomment some or all of these to enable the optional hardware interfaces
#dtparam=i2c_arm=on
#dtparam=i2s=on
#dtparam=spi=on

# Enable audio (loads snd_bcm2835)
dtparam=audio=on

# Additional overlays and parameters are documented
# /boot/firmware/overlays/README

# Automatically load overlays for detected cameras
camera_auto_detect=1

# Automatically load overlays for detected DSI displays
display_auto_detect=1

# Automatically load initramfs files, if found
auto_initramfs=1

# Enable DRM VC4 V3D driver
dtoverlay=vc4-kms-v3d
max_framebuffers=2

# Don't have the firmware create an initial video= setting in cmdline.txt.
# Use the kernel's default instead.
disable_fw_kms_setup=1

# Run in 64-bit mode
arm_64bit=1

# Disable compensation for displays with overscan
disable_overscan=1

# Run as fast as firmware / board allows
arm_boost=1

[cm4]
# Enable host mode on the 2711 built-in XHCI USB controller.
# This line should be removed if the legacy DWC2 controller is required
# (e.g. for USB device mode) or if USB support is not required.
otg_mode=1

[cm5]
dtoverlay=dwc2,dr_mode=host

[all]
dtparam=spi=on" > $config_file
if [ $? -ne 0 ]; then
    echo "Error: Could not write base config to $config_file."
    exit 1
fi
# Append the overlay configs for each CAN interface to the config file
for conf in "${can_configs[@]}"; do
    overlay="dtoverlay=mcp2515-${conf%% *},oscillator=16000000,interrupt=${conf##* }"
    echo "$overlay" >> $config_file
    if [ $? -ne 0 ]; then
        echo "Error: Could not write overlay config to $config_file for config: $conf"
        exit 1
    fi
done

