# Copyright 2012 Jared Boone <jared@sharebrained.com>
#
# This file is part of HackRF.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_TOOLCHAIN_FILE ../toolchain-arm-cortex-m.cmake)

project(hackrf_usb)

include(../hackrf-common.cmake)

set(SRC_M4
	hackrf_usb.c
	"${PATH_HACKRF_FIRMWARE_COMMON}/tuning.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/streaming.c"
	sgpio_isr.c
	usb_bulk_buffer.c
	"${PATH_HACKRF_FIRMWARE_COMMON}/usb.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/usb_request.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/usb_standard_request.c"
	usb_descriptor.c
	usb_device.c
	usb_endpoint.c
	usb_api_board_info.c
	usb_api_cpld.c
	usb_api_register.c
	usb_api_spiflash.c
	usb_api_transceiver.c
  light_ws2812_cortex.c
	"${PATH_HACKRF_FIRMWARE_COMMON}/usb_queue.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/fault_handler.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/w25q80bv.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/cpld_jtag.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/xapp058/lenval.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/xapp058/micro.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/xapp058/ports.c"
	"${PATH_HACKRF_FIRMWARE_COMMON}/rom_iap.c"
  ../../../r0ketlib/display.o
  ../../../r0ketlib/render.o
  ../../../r0ketlib/fonts/smallfonts.o
  ../../../r0ketlib/fonts/orbitron14.o
  ../../../r0ketlib/fonts/ubuntu18.o
  ../../../r0ketlib/decoder.o
  ../../../r0ketlib/print.o
  ../../../r0ketlib/itoa.o
  ../../../r0ketlib/keyin.o
  ../../../r0ketlib/menu.o
  ../../../rad1olib/spi-flash.o
  ../../../rad1olib/assert.o
  ../../../rad1olib/systick.o
  ../../../fatfs/diskio.o
  ../../../fatfs/ff.o
  ../../../r0ketlib/select.o
  ../../../r0ketlib/idle.o
  ../../../r0ketlib/fs_util.o
  ../../../r0ketlib/execute.o
  ../../../r0ketlib/config.o
  ../../../r0ketlib/stringin.o
  ../../../r0ketlib/colorin.o
  ../../../r0ketlib/random.o
  ../../../r0ketlib/image.o
  ../../../r0ketlib/night.o
  ../../../rad1olib/battery.o
)

include_directories(../../../)

DeclareTargets()
