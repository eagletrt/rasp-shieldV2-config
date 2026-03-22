#!/bin/sh
# This script is used to create the overlay files for the MCP2515 CAN controller.
# It takes as input the spi and the interrupt pins and generates the overlay files accordingly.

if [ "$#" -ne 4 ]; then
    echo "Error: Invalid number of arguments."
    echo "Usage: $0 <can_interface> <spi_num> <chip_select> <interrupt_num>"
    echo "Example: $0 can0 0 0 25"
    exit 1
fi

can_interface=$1
spi_num=$2
chip_select=$3
interrupt_num=$4

spi="spi${spi_num}.${chip_select}"
filename="mcp2515-${can_interface}-overlay.dts"
file_content="/*
 * Device tree overlay for mcp251x/${can_interface} on ${spi}
 */

/dts-v1/;
/plugin/;

/ {
    compatible = \"brcm,bcm2711\";
    /* disable spi-dev for ${spi} */
    fragment@0 {
        target = <&spi${spi_num}>;
        __overlay__ {
            status = \"okay\";
        };
    };

    fragment@1 {
	target = <&spidev${chip_select}>;
	__overlay__ {
	    status = \"disabled\";
	};
    };

    /* the interrupt pin of the can-controller */
    fragment@2 {
        target = <&gpio>;
        __overlay__ {
            ${can_interface}_pins: ${can_interface}_pins {
                brcm,pins = <${interrupt_num}>; /* default pin, it will be overriden */
                brcm,function = <0>; /* input */
            };
        };
    };

    /* the clock/oscillator of the can-controller */
    fragment@3 {
        target-path = \"/\";
        __overlay__ {
            /* external oscillator of mcp2515 on ${spi} */
            ${can_interface}_osc: ${can_interface}_osc {
                compatible = \"fixed-clock\";
                #clock-cells = <0>;
                clock-frequency  = <16000000>; /* default ext clock speed, it will be overridden */
            };
        };
    };

    /* the spi config of the can-controller itself binding everything together */
    fragment@4 {
        target = <&spi${spi_num}>;
        __overlay__ {
            /* needed to avoid dtc warning */
            #address-cells = <1>;
            #size-cells = <0>;
            ${can_interface}: mcp2515@${chip_select} {
                reg = <${chip_select}>;
                compatible = \"microchip,mcp2515\";
                pinctrl-names = \"default\";
                pinctrl-0 = <&${can_interface}_pins>;
                spi-max-frequency = <10000000>;
                interrupt-parent = <&gpio>;
                interrupts = <${interrupt_num} 8>; /* IRQ_TYPE_LEVEL_LOW */
                clocks = <&${can_interface}_osc>;
            };
        };
    };
    __overrides__ {
        oscillator = <&${can_interface}_osc>,\"clock-frequency:0\";
        spimaxfrequency = <&${can_interface}>,\"spi-max-frequency:0\";
        interrupt = <&${can_interface}_pins>,\"brcm,pins:0\",<&${can_interface}>,\"interrupts:0\";
    };
};"

echo "$file_content" > $filename

