.. _openAMP_sample:

OpenAMP Sample Application
##########################

Overview
********

This application demonstrates how to use OpenAMP to share an I2C bus between
Zephyr and a remote processor. It is designed to implement a proxy to mux I2C
communication.
It demonstrates the use of the VirIO MMIO and Virtio-I2C to expose to a remote
main processor an I2C bus. This sample isconpatible only with the STM32MP1 SoC.

requested Hardware
*************************

- STM32MP115c-dk2 board
- Nucleo shield ISK01A2
- 2 SSD1306 displays connected on I2C arduino connectors:

  - display @0x3C: managed by Zephyr
  - display @0x3D: managed by Remote processor

Building the application
*************************

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/ipc/openamp_virtio_i2c
   :board: stm32mp157c_dk2
   :goals: debug

Copy the binary file on the SDCard (in /lib/firmware/)

Open a Linux terminal and connect using STLink:

.. code-block:: console

  minicom -D /dev/ttyACM0

Load and start the firmware:

.. code-block:: console

  echo -n zephyr_openamp_virtio_i2c.elf > /sys/class/remoteproc/remoteproc0/firmware
  echo start >/sys/class/remoteproc/remoteproc0/state


This is the Linux console:

.. code-block:: console

  root@stm32mp1:~# i2cdetect -l
  i2c-0   i2c             STM32F7 I2C(0x40012000)                 I2C adapter
  i2c-1   i2c             STM32F7 I2C(0x5c002000)                 I2C adapter
  i2c-2   i2c             i2c-0-mux (chan_id 0)                   I2C adapter
  i2c-3   i2c             i2c_virtio at virtio bus 0              I2C adapter
  root@stm32mp1:~# i2cdetect -y 3
      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
  00:                         -- -- -- -- -- -- -- --
  10: -- -- -- -- -- -- -- -- -- 19 -- -- -- -- 1e --
  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  30: -- -- -- -- -- -- -- -- -- -- -- -- 3c 3d -- --
  40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  50: -- -- -- -- -- -- -- -- -- -- -- -- -- 5d -- 5f
  60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
  70: -- -- -- -- -- -- -- --
