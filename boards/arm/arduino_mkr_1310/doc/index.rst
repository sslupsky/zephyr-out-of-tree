.. _arduino_mkr_1310:

Arduino/Genuino MKR WAN 1310
############################

Overview
********

The Arduino MKR WAN 1310 is a maker-friendly development board with
Atmel’s SWD, which provides a full debug interface.

.. image:: img/arduino_mkr_1310.png
     :width: 500px
     :align: center
     :alt: Arduino MKR WAN 1310

Hardware
********

- ATSAMD21G18A ARM Cortex-M0+ processor at 48 MHz
- 32.768 kHz crystal oscillator
- 256 KiB flash memory and 32 KiB of RAM
- 1 user LEDs
- One reset button
- Native USB port
- LoRA radio
- 16MBit Flash
- ECC508 crypto
- BQ24195L PMIC

Supported Features
==================

The arduino_mkr_1310 board configuration supports the following hardware
features:

+-----------+------------+--------------------------------------+
| Interface | Controller | Driver/Component                     |
+===========+============+======================================+
| NVIC      | on-chip    | nested vector interrupt controller   |
+-----------+------------+--------------------------------------+
| Flash     | on-chip    | Can be used with NFFS to store files |
+-----------+------------+--------------------------------------+
| SYSTICK   | on-chip    | systick                              |
+-----------+------------+--------------------------------------+
| WDT       | on-chip    | Watchdog                             |
+-----------+------------+--------------------------------------+
| GPIO      | on-chip    | I/O ports                            |
+-----------+------------+--------------------------------------+
| USART     | on-chip    | Serial ports                         |
+-----------+------------+--------------------------------------+
| SPI       | on-chip    | Serial Peripheral Interface ports    |
+-----------+------------+--------------------------------------+
| I2C       | on-chip    | Serial I2C ports                     |
+-----------+------------+--------------------------------------+
| USB       | on-chip    | USB device                           |
+-----------+------------+--------------------------------------+

Other hardware features are not currently supported by Zephyr.

The default configuration can be found in the Kconfig
:zephyr_file:`boards/arm/arduino_mkr_1310/arduino_mkr_1310_defconfig`.

Connections and IOs
===================

The `Arduino store`_ has detailed information about board
connections. Download the `Arduino MKR WAN 1310 Schematic`_ for more detail.

System Clock
============

The SAMD21 MCU is configured to use the 32.768 kHz external oscillator
with the on-chip PLL generating the 48 MHz system clock.  The internal
APB and GCLK unit are set up similar to the upstream Arduino
libraries.

Serial Port
===========

The SAMD21 MCU has 6 SERCOM based USARTs. SERCOM5 is available on the D13/D14
pins.

SPI Port
========

The SAMD21 MCU has 6 SERCOM based SPIs.

I2C Port
========

The SAMD21 MCU has 6 SERCOM based I2C ports.  On the Arduino MKR WAN 1310, 
SERCOM0 is available on the 6 pin connector at the edge of the board.

USB Device Port
===============

The SAMD21 MCU has a USB device port that can be used to communicate
with a host PC.  See the :ref:`usb-samples` sample applications for
more, such as the :ref:`usb_cdc-acm` sample which sets up a virtual
serial port that echos characters back to the host PC.

Programming and Debugging
*************************

The Arduino MKR WAN 1310 comes with Atmel SWD.  This provides a debug interface
to the SAMD21 chip and is supported by OpenOCD.

Flashing
========

#. Build the Zephyr kernel and the :ref:`hello_world` sample application:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: arduino_mkr_1310
      :goals: build
      :compact:

#. Connect the Arduino MKR WAN 1310 to your host computer using the USB debug
   port.

#. Run your favorite terminal program to listen for output. Under Linux the
   terminal should be :code:`/dev/ttyACM0`. For example:

   .. code-block:: console

      $ minicom -D /dev/ttyACM0 -o

   The -o option tells minicom not to send the modem initialization
   string. Connection should be configured as follows:

   - Speed: 115200
   - Data: 8 bits
   - Parity: None
   - Stop bits: 1

#. To flash an image:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: arduino_mkr_1310
      :goals: flash
      :compact:

   You should see "Hello World! arm" in your terminal.

References
**********

.. target-notes::

.. _Arduino Store:
    https://store.arduino.cc/usa/mkr-wan-1310

.. _Arduino MKR WAN 1310 Schematic:
    https://content.arduino.cc/assets/MKRWAN1310V3.0_sch.pdf
