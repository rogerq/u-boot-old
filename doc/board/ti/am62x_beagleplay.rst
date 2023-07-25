.. SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
.. sectionauthor:: Nishanth Menon <nm@ti.com>

Beagleboard.org Beagleplay
==========================

Introduction:
-------------
BeagleBoard.org BeaglePlay is an easy to use, affordable open source
hardware single board computer based on the Texas Instruments AM625
SoC that allows you to create connected devices that work even at long
distances using IEEE 802.15.4g LR-WPAN and IEEE 802.3cg 10Base-T1L.
Expansion is provided over open standards based mikroBUS, Grove and
QWIIC headers among other interfaces.

Further information can be found at:
https://beagleplay.org/
https://git.beagleboard.org/beagleplay/beagleplay

Boot Flow:
----------
Below is the pictorial representation of boot flow:

.. image:: img/boot_diagram_k3_current.svg

- Here TIFS acts as master and provides all the critical services. R5/A53
  requests TIFS to get these services done as shown in the above diagram.

Sources:
--------
.. include::  k3.rst
    :start-after: .. k3_rst_include_start_boot_sources
    :end-before: .. k3_rst_include_end_boot_sources

Build procedure:
----------------
0. Setup the environment variables:

.. include::  k3.rst
    :start-after: .. k3_rst_include_start_common_env_vars_desc
    :end-before: .. k3_rst_include_end_common_env_vars_desc

.. include::  k3.rst
    :start-after: .. k3_rst_include_start_board_env_vars_desc
    :end-before: .. k3_rst_include_end_board_env_vars_desc

Set the variables corresponding to this platform:

.. include::  k3.rst
    :start-after: .. k3_rst_include_start_common_env_vars_defn
    :end-before: .. k3_rst_include_end_common_env_vars_defn
.. code-block:: bash

 $ export UBOOT_CFG_CORTEXR=am62x_beagleplay_r5_defconfig
 $ export UBOOT_CFG_CORTEXA=am62x_beagleplay_a53_defconfig
 $ export TFA_BOARD=lite
 $ # we dont use any extra TFA parameters
 $ unset TFA_EXTRA_ARGS
 $ export OPTEE_PLATFORM=k3-am62x
 $ export OPTEE_EXTRA_ARGS="CFG_WITH_SOFTWARE_PRNG=y"

.. include::  am62x_sk.rst
    :start-after: .. am62x_evm_rst_include_start_build_steps
    :end-before: .. am62x_evm_rst_include_end_build_steps

Target Images
--------------
Copy the below images to an SD card and boot:

* tiboot3-am62x-gp-evm.bin from step 3.1 as tiboot3.bin
* tispl.bin_unsigned from step 3.2 as tispl.bin
* u-boot.img_unsigned from step 3.2 as uboot.img

Image formats:
--------------

- tiboot3.bin

.. image:: img/multi_cert_tiboot3.bin.svg

- tispl.bin

.. image:: img/dm_tispl.bin.svg

A53 SPL DDR Memory Layout
-------------------------

.. include::  am62x_sk.rst
    :start-after: .. am62x_evm_rst_include_start_ddr_mem_layout
    :end-before: .. am62x_evm_rst_include_end_ddr_mem_layout

Switch Setting for Boot Mode
----------------------------

The Boot time switch option is configured via "USR" button on the board.
See https://git.beagleboard.org/beagleplay/beagleplay/-/blob/main/BeaglePlay_sch.pdf
for details.

*Boot Modes*

=========== ============ ==============
Switch Posn Primary Boot Secondary Boot
=========== ============ ==============
Not Pressed  eMMC          UART
Pressed      SD FS mode    USB DFU
=========== ============ ==============

The procedure for using the USR button for switching to SD card boot mode
is to keep the USR button pressed while providing power over the Type-C power
supply and releasing the USR button once the power LED glows.
