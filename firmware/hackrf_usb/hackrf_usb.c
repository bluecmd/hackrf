/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <libopencm3/cm3/vector.h>

#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/m4/nvic.h>

#include <r0ketlib/fonts.h>
#include <r0ketlib/keyin.h>
#include <r0ketlib/display.h>
#include <r0ketlib/fs_util.h>
#include <r0ketlib/idle.h>
#include <r0ketlib/render.h>
#include <rad1olib/pins.h>
#include <rad1olib/systick.h>

#include <streaming.h>

#include "usb.h"
#include "usb_standard_request.h"

#include <rom_iap.h>
#include "usb_descriptor.h"

#include "usb_device.h"
#include "usb_endpoint.h"
#include "usb_api_board_info.h"
#include "usb_api_cpld.h"
#include "usb_api_register.h"
#include "usb_api_spiflash.h"

#include "usb_api_transceiver.h"
#include "rf_path.h"
#include "sgpio_isr.h"
#include "usb_bulk_buffer.h"
#include "si5351c.h"
#include "light_ws2812_cortex.h"
#include "tuning.h"
#include "max2837.h"
 
#define min(a,b) (((a) < (b)) ? (a) : (b))

#define WAIT_CPU_CLOCK_INIT_DELAY   (10000)

uint64_t _freq = 0;

uint8_t pattern[] = {
  0,   0,   0,
  0,   0,   0,
  0,   0,   0,
  0,   0,   0,
  0,   0,   0,
  0,   0,   0,
  0,   0,   0,
  0,   0,   0
};

static volatile transceiver_mode_t _transceiver_mode = TRANSCEIVER_MODE_OFF;

bool blink = 0;
void led_redraw() {
  if (_transceiver_mode == TRANSCEIVER_MODE_TX) {
    for (int i = 2*3; i < 7*3; i+=3) {
      pattern[i + 0] = 0;
      pattern[i + 1] = 64;
      pattern[i + 2] = 0;
    }
  } else if (_transceiver_mode == TRANSCEIVER_MODE_RX) {
    for (int i = 2*3; i < 7*3; i+=3) {
      pattern[i + 0] = 64;
      pattern[i + 1] = 64;
      pattern[i + 2] = 64;
    }
  }
  if (_transceiver_mode != TRANSCEIVER_MODE_OFF) {
    blink = !blink;
    if (blink) {
      pattern[7*3 + 0] = 64;
      pattern[7*3 + 1] = 64;
      pattern[7*3 + 2] = 64;
    } else {
      pattern[7*3 + 0] = 0;
      pattern[7*3 + 1] = 64;
      pattern[7*3 + 2] = 0;
    }
  } else {
    memset(pattern, 0, sizeof(pattern));
  }
  ws2812_sendarray(pattern, sizeof(pattern));
}

static void set_transceiver_mode(const transceiver_mode_t new_transceiver_mode,
                                 int usb) {
  baseband_streaming_disable();
  
  if (usb) {
    usb_endpoint_disable(&usb_endpoint_bulk_in);
    usb_endpoint_disable(&usb_endpoint_bulk_out);
  }
  
  _transceiver_mode = new_transceiver_mode;
  
  if( _transceiver_mode == TRANSCEIVER_MODE_RX ) {
    gpio_clear(PORT_LED1_3, PIN_LED3);
    gpio_set(PORT_LED1_3, PIN_LED2);
    usb_endpoint_init(&usb_endpoint_bulk_in);
    rf_path_set_direction(RF_PATH_DIRECTION_RX);
    vector_table.irq[NVIC_SGPIO_IRQ] = sgpio_isr_rx;
  } else if (_transceiver_mode == TRANSCEIVER_MODE_TX) {
    gpio_clear(PORT_LED1_3, PIN_LED2);
    gpio_set(PORT_LED1_3, PIN_LED3);
    usb_endpoint_init(&usb_endpoint_bulk_out);
    rf_path_set_direction(RF_PATH_DIRECTION_TX);
    vector_table.irq[NVIC_SGPIO_IRQ] = sgpio_isr_tx;
  } else {
    gpio_clear(PORT_LED1_3, PIN_LED2);
    gpio_clear(PORT_LED1_3, PIN_LED3);
    rf_path_set_direction(RF_PATH_DIRECTION_OFF);
    vector_table.irq[NVIC_SGPIO_IRQ] = sgpio_isr_rx;
  }

  if( _transceiver_mode != TRANSCEIVER_MODE_OFF ) {
    si5351c_activate_best_clock_source();
    baseband_streaming_enable();
  }
}

transceiver_mode_t transceiver_mode(void) {
  return _transceiver_mode;
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage
) {
  if( stage == USB_TRANSFER_STAGE_SETUP ) {
    switch( endpoint->setup.value ) {
    case TRANSCEIVER_MODE_OFF:
    case TRANSCEIVER_MODE_RX:
    case TRANSCEIVER_MODE_TX:
      set_transceiver_mode(endpoint->setup.value, 1);
      usb_transfer_schedule_ack(endpoint->in);
      return USB_REQUEST_STATUS_OK;
    default:
      return USB_REQUEST_STATUS_STALL;
    }
  } else {
    return USB_REQUEST_STATUS_OK;
  }
}

static const usb_request_handler_fn vendor_request_handler[] = {
  NULL,
  usb_vendor_request_set_transceiver_mode,
  usb_vendor_request_write_max2837,
  usb_vendor_request_read_max2837,
  usb_vendor_request_write_si5351c,
  usb_vendor_request_read_si5351c,
  usb_vendor_request_set_sample_rate_frac,
  usb_vendor_request_set_baseband_filter_bandwidth,
#ifdef RAD1O
    NULL,
    NULL,
#else
  usb_vendor_request_write_rffc5071,
  usb_vendor_request_read_rffc5071,
#endif
  usb_vendor_request_erase_spiflash,
  usb_vendor_request_write_spiflash,
  usb_vendor_request_read_spiflash,
  NULL, // used to be write_cpld
  usb_vendor_request_read_board_id,
  usb_vendor_request_read_version_string,
  usb_vendor_request_set_freq,
  usb_vendor_request_set_amp_enable,
  usb_vendor_request_read_partid_serialno,
  usb_vendor_request_set_lna_gain,
  usb_vendor_request_set_vga_gain,
  usb_vendor_request_set_txvga_gain,
  NULL, // was set_if_freq
#ifdef HACKRF_ONE
  usb_vendor_request_set_antenna_enable,
#else
  NULL,
#endif
  usb_vendor_request_set_freq_explicit,
};

static const uint32_t vendor_request_handler_count =
  sizeof(vendor_request_handler) / sizeof(vendor_request_handler[0]);

usb_request_status_t usb_vendor_request(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage
) {
  usb_request_status_t status = USB_REQUEST_STATUS_STALL;
  
  if( endpoint->setup.request < vendor_request_handler_count ) {
    usb_request_handler_fn handler = vendor_request_handler[endpoint->setup.request];
    if( handler ) {
      status = handler(endpoint, stage);
    }
  }
  
  return status;
}

const usb_request_handlers_t usb_request_handlers = {
  .standard = usb_standard_request,
  .class = 0,
  .vendor = usb_vendor_request,
  .reserved = 0,
};

void usb_configuration_changed(
  usb_device_t* const device
) {
  /* Reset transceiver to idle state until other commands are received */
  set_transceiver_mode(TRANSCEIVER_MODE_OFF, 1);
  if( device->configuration->number == 1 ) {
    // transceiver configuration
    cpu_clock_pll1_max_speed();
    gpio_set(PORT_LED1_3, PIN_LED1);
  } else if( device->configuration->number == 2 ) {
    // CPLD update configuration
    cpu_clock_pll1_max_speed();
    usb_endpoint_init(&usb_endpoint_bulk_out);
    start_cpld_update = true;
  } else {
    /* Configuration number equal 0 means usb bus reset. */
    cpu_clock_pll1_low_speed();
    gpio_clear(PORT_LED1_3, PIN_LED1);
  }
}

void usb_set_descriptor_by_serial_number(void)
{
#ifndef RAD1O
  iap_cmd_res_t iap_cmd_res;
  
  /* Read IAP Serial Number Identification */
  iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_SERIAL_NO;
  iap_cmd_call(&iap_cmd_res);
  
  if (iap_cmd_res.status_res.status_ret == CMD_SUCCESS) {
    usb_descriptor_string_serial_number[0] = USB_DESCRIPTOR_STRING_SERIAL_BUF_LEN;
    usb_descriptor_string_serial_number[1] = USB_DESCRIPTOR_TYPE_STRING;
    
    /* 32 characters of serial number, convert to UTF-16LE */
    for (size_t i=0; i<USB_DESCRIPTOR_STRING_SERIAL_LEN; i++) {
      const uint_fast8_t nibble = (iap_cmd_res.status_res.iap_result[i >> 3] >> (28 - (i & 7) * 4)) & 0xf;
      const char c = (nibble > 9) ? ('a' + nibble - 10) : ('0' + nibble);
      usb_descriptor_string_serial_number[2 + i * 2] = c;
      usb_descriptor_string_serial_number[3 + i * 2] = 0x00;
    }
  } else {
#else
    {
#endif
    usb_descriptor_string_serial_number[0] = 2;
    usb_descriptor_string_serial_number[1] = USB_DESCRIPTOR_TYPE_STRING;
  }
}

int offset = 0;
bool direction = 0;
volatile bool refresh_lock = 0;
void redraw(void) {
  if (refresh_lock) {
    return;
  }
  refresh_lock = 1;

  led_redraw();

  if (!direction) {
    offset += 1;
  } else {
    offset -= 1;
  }

  setExtFont("soviet18.f0n");

  char freq[64];
  if (_freq == 0 || _transceiver_mode == TRANSCEIVER_MODE_OFF) {
    strcpy(freq, "OFF");
  } else {
    snprintf(freq, 64, "%llu.%llu MHz", _freq/1000000, (_freq/10000) % 10);
  }

  int freq_width = DoString(0, 0, freq);
  int nick_width = DoString(0, 0, "blueCmd");

  lcdFill(0xff);

  {
    int dx = (RESX-nick_width)/2;
    if(dx < 0)
      dx = 0;
    int dy = (RESY-getFontHeight())/3;

    if (dx + nick_width + offset + 1 > RESX) {
      direction = !direction;
    } else if (dx + offset - 1 < 0) {
      direction = !direction;
    }

    setTextColor(0xff, 0x00);
    DoString(dx + offset, dy, "blueCmd");
  }
  {
    int dx = (RESX-freq_width)/2;
    if(dx < 0)
      dx = 0;
    int dy = (RESY-getFontHeight())/3 * 2;

    setTextColor(0xff, 0x00);
    DoString(dx, dy, freq);
  }
  lcdDisplay();
  refresh_lock = 0;
}

void sys_tick_handler(void){
  incTimer();
  if ((_timectr & 0x3ff) == 0) {
    redraw();
  }
  if ((_timectr & 0x3ff) == 0) {
    ON(LED4);
  }
  if ((_timectr & 0x3ff) == 512) {
    OFF(LED4);
  }
}

void cpu_set_freq(void) {
  /* initialisation similar to UM10503 v1.9 sec. 13.2.1.1 */
  CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_IRC) | CGU_BASE_M4_CLK_AUTOBLOCK(1));

  /* Enable XTAL */
  CGU_XTAL_OSC_CTRL &= ~(CGU_XTAL_OSC_CTRL_HF_MASK|CGU_XTAL_OSC_CTRL_ENABLE_MASK);
  delay(WAIT_CPU_CLOCK_INIT_DELAY); /* should be 250us / 3000 cycles @ 12MhZ*/

  /* Set PLL1 up for 204 MHz */
  CGU_PLL1_CTRL= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
        | CGU_PLL1_CTRL_MSEL(17-1)
        | CGU_PLL1_CTRL_NSEL(0)
        | CGU_PLL1_CTRL_AUTOBLOCK(1)
        | CGU_PLL1_CTRL_PSEL(0)
        | CGU_PLL1_CTRL_DIRECT(1)
        | CGU_PLL1_CTRL_FBSEL(1)
        | CGU_PLL1_CTRL_BYPASS(0)
        | CGU_PLL1_CTRL_PD(0)
        ;
  /* Wait for PLL Lock */
  while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK_MASK));

  /* TODO(bluecmd): see if we actually need to land on 102 MHz
   * before going to 204 MHz as people say. */

  /* set DIV B to 102 MHz */
  CGU_IDIVB_CTRL= CGU_IDIVB_CTRL_CLK_SEL(CGU_SRC_PLL1)
    | CGU_IDIVB_CTRL_AUTOBLOCK(1) 
    | CGU_IDIVB_CTRL_IDIV(1)
    | CGU_IDIVB_CTRL_PD(0)
    ;

  delay(WAIT_CPU_CLOCK_INIT_DELAY); /* should be 50us / 5100 @ 102MhZ */

  /* set DIV B to 204 MHz */
  CGU_IDIVB_CTRL= CGU_IDIVB_CTRL_CLK_SEL(CGU_SRC_PLL1)
    | CGU_IDIVB_CTRL_AUTOBLOCK(1) 
    | CGU_IDIVB_CTRL_IDIV(0)
    | CGU_IDIVB_CTRL_PD(0)
    ;

  /* use DIV B as main clock */
  /* This means, that possible speeds in MHz are:
   * 204 102 68 51 40.8 34 29.14 25.5 22.66 20.4 18.54 17 15.69 14.57 13.6 12.75
   */

  CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_IDIVB) | CGU_BASE_M4_CLK_AUTOBLOCK(1));

  /* TODO(bluecmd): delay *2 since we're faster now? */
  delay(WAIT_CPU_CLOCK_INIT_DELAY);
}

void new_freq(const uint64_t freq) {
  _freq = freq;
}

void prime_tx(void) {

  set_freq(88000000);
  new_freq(88000000);

  rf_path_set_lna(1);
  sample_rate_frac_set(4000000, 1);
  max2837_set_lna_gain(40);
  max2837_set_txvga_gain(47);
  rf_path_set_antenna(1);

#if 0
#include "short.h"
  memcpy(usb_bulk_buffer, short_dat,
      min(short_dat_len, sizeof(usb_bulk_buffer)));
#endif

  set_transceiver_mode(TRANSCEIVER_MODE_TX, 0);
}

int main(void) {
  pin_setup();
  enable_1v8_power();

#if (defined HACKRF_ONE || defined RAD1O)
  enable_rf_power();
  delay(1000000);
#endif
  cpu_clock_init();
  // TODO(bluecmd): figure out why ws2812 fails @ 204 MHz
  cpu_set_freq();
  
  ssp_clock_init();
  systickInit();

  nvic_set_priority(NVIC_SYSTICK_IRQ, 254);

  SETUPgout(LED4);
  SETUPgout(RGB_LED);

  fsInit(); 
  lcdInit();

  lcdSetContrast(58);

  usb_set_descriptor_by_serial_number();

  usb_set_configuration_changed_cb(usb_configuration_changed);
  usb_peripheral_reset();
  
  usb_device_init(0, &usb_device);
  
  usb_queue_init(&usb_endpoint_control_out_queue);
  usb_queue_init(&usb_endpoint_control_in_queue);
  usb_queue_init(&usb_endpoint_bulk_out_queue);
  usb_queue_init(&usb_endpoint_bulk_in_queue);

  usb_endpoint_init(&usb_endpoint_control_out);
  usb_endpoint_init(&usb_endpoint_control_in);
  
  nvic_set_priority(NVIC_USB0_IRQ, 255);

  usb_run(&usb_device);
  
  ssp1_init();

  rf_path_init();

  unsigned int phase = 0;
  while(true) {
    int ret = 0;

    // Check whether we need to initiate a CPLD update
    if (start_cpld_update)
      cpld_update();

    do {
      switch(getInput()) {
        case BTN_UP:
          prime_tx();
          break;
        case BTN_DOWN:
          set_transceiver_mode(TRANSCEIVER_MODE_OFF, 0);
          break;
      }

      // Set up IN transfer of buffer 0.
      if ( usb_bulk_buffer_offset >= 16384
           && phase == 1
           && transceiver_mode() != TRANSCEIVER_MODE_OFF) {
        ret = usb_transfer_schedule(
          (transceiver_mode() == TRANSCEIVER_MODE_RX)
          ? &usb_endpoint_bulk_in : &usb_endpoint_bulk_out,
          &usb_bulk_buffer[0x0000],
          0x4000,
          NULL, NULL
          );
        phase = 0;
      }

      // Set up IN transfer of buffer 0.
      if ( usb_bulk_buffer_offset < 16384
           && phase == 0
           && transceiver_mode() != TRANSCEIVER_MODE_OFF) {
        ret = usb_transfer_schedule(
          (transceiver_mode() == TRANSCEIVER_MODE_RX)
          ? &usb_endpoint_bulk_in : &usb_endpoint_bulk_out,
          &usb_bulk_buffer[0x4000],
          0x4000,
          NULL, NULL
        );
        phase = 1;
      }
    } while(ret == -1);
  }
  return 0;
}
