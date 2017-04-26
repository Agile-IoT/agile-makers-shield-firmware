
# AGILE Makers Shield Firmware

This repository contains sources and binaries of
the AGILE Makers Shield firmware.

It also contains scripts to load the firmware to the shield
and the ID EEPROM files to identify the Raspberry HAT.


## Firmware

#### Installation

To load the firmware into the AGILE follow these steps:

1. Install _avrdude_ in your Raspberry Pi:
```bash
# If using a Debian based system
sudo apt-get install avrdude
```
* Use the default configuration for _avrdude_. The shield is
designed to use the default GPIO25 to send the reset signal.
* Copy the `load_agile_firmware.sh` script, along with the
firmware (`agile_firmware_vXXX.hex`), both found in the `bin`
folder, to your Raspberry Pi.
* Put the programming switch in the ON position on your shield.
* Execute the script using the parameter `-f` to specify the
firmware path:
```bash
./load_agile_firmware -p agile_firmware_vXXX.hex
```
* If everything is OK, _avrdude_ will load the firmware and
won't show any errors. The last output lines should be similar
to these:
```
Reading | ################################################## | 100% 2.39s
avrdude: verifying ...
avrdude: 9640 bytes of flash verified
avrdude: safemode: lfuse reads as FF
avrdude: safemode: hfuse reads as D8
avrdude: safemode: efuse reads as FD
avrdude: safemode: Fuses OK (E:FD, H:D8, L:FF)
avrdude done.  Thank you.
```
* Put the programming switch in the OFF position on your shield.
If everything went OK, the shield LEDs should turn on in a
Red-Green-Blue sequence.


#### Build from sources

The source file of the firmware is included in case anyone wants
to build the binary from the source. In order to build the binary,
we will use the [Arduino IDE](https://www.arduino.cc/en/Main/Software),
although other methods can be used.

1. Download and install the
[Arduino IDE](https://www.arduino.cc/en/Guide/HomePage)
* Copy the contents of the `src/sketchbook` folder into your
Arduino `sketchbook` folder.
* Copy the contents of the `src/hardware` folder into your
Arduino installation `hardware` folder.
* Launch the Arduino IDE. Open the `agile_firmware.ino` skecth,
select the `AGILE Maker's Shield` board in the `Tools` menu and
compile the program.
* Click on `Sketch -> Export compiled Binary`. This will generate
the `agile_firmware.hex` file in the sketch folder. Now it can be
installed following the installation method.


## ID EEPROM

The files included in the `id_eeprom` folder are the configuration
file and the compiled configuration file needed to write the ID in
the shield's EEPROM as specified in the [Raspberry Pi HAT ID EEPROM Format Specification](https://github.com/raspberrypi/hats/blob/master/eeprom-format.md).
