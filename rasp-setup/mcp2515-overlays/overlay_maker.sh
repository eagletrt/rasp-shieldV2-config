#!/bin/sh
# This script is used to create the overlay files for the MCP2515 CAN controller.
# It takes as input the spi and the interrupt pins and generates the overlay files accordingly.

can_interface=can0
spi_num=0
chip_select=0
spi="spi${spi_num}.${chip_select}"
interrupt=gpio25
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
	target = <&spidev${spi_num}>;
	__overlay__ {
	    status = \"disabled\";
	};
    };

    /* the interrupt pin of the can-controller */
    fragment@2 {
        target = <&gpio>;
        __overlay__ {
            ${can_interface}_pins: ${can_interface}_pins {
                brcm,pins = <25>; /* default pin, it will be overriden */
                brcm,function = <0>; /* input */
            };
        };
    };

    /* the clock/oscillator of the can-controller */
    fragment@3 {
        target-path = \"/\";
        __overlay__ {
            /* external oscillator of mcp2515 on SPI${spi_num}.${chip_select} */
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
            ${can_interface}: mcp2515@0 {
                reg = <0>;
                compatible = \"microchip,mcp2515\";
                pinctrl-names = \"default\";
                pinctrl-0 = <&${can_interface}_pins>;
                spi-max-frequency = <10000000>;
                interrupt-parent = <&gpio>;
                interrupts = <25 8>; /* IRQ_TYPE_LEVEL_LOW */
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

