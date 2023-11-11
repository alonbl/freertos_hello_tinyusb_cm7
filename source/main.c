/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "board.h"

#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_lpuart.h"
#include "pin_mux.h"

#include "tusb.h"

// Increase stack size when debug log is enabled
#define USBD_STACK_SIZE    (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)

#define CDC_STACK_SZIE      configMINIMAL_STACK_SIZE

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

TimerHandle_t blinky_tm;

static void led_blinky_cb(TimerHandle_t xTimer);
static void usb_device_task(void *param);
void cdc_task(void *params);

static void init_usb_phy(USBPHY_Type* usb_phy) {
  // Enable PHY support for Low speed device + LS via FS Hub
  usb_phy->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK | USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;

  // Enable all power for normal operation
  // TODO may not be needed since it is called within CLOCK_EnableUsbhs0PhyPllClock()
  usb_phy->PWD = 0;

  // TX Timing
  uint32_t phytx = usb_phy->TX;
  phytx &= ~(USBPHY_TX_D_CAL_MASK | USBPHY_TX_TXCAL45DM_MASK | USBPHY_TX_TXCAL45DP_MASK);
  phytx |= USBPHY_TX_D_CAL(0x0C) | USBPHY_TX_TXCAL45DP(0x06) | USBPHY_TX_TXCAL45DM(0x06);
  usb_phy->TX = phytx;
}

void board_init(void)
{
  // make sure the dcache is on.
  if (SCB_CCR_DC_Msk != (SCB_CCR_DC_Msk & SCB->CCR)) SCB_EnableDCache();

  BOARD_ConfigMPU();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  // Init clock
  SystemCoreClockUpdate();

  // Enable IOCON clock
  CLOCK_EnableClock(kCLOCK_Iomuxc);

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_OTG1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

  // Clock
  CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
  CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);

  init_usb_phy(USBPHY1);
}

void USB_OTG1_IRQHandler(void)
{
    tud_int_handler(0);
}

int main(void) {
  board_init();

  DbgConsole_Printf("HELLO.\r\n");

  blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
  xTaskCreate(usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(cdc_task, "cdc", CDC_STACK_SZIE, NULL, configMAX_PRIORITIES - 2, NULL);

  xTimerStart(blinky_tm, 0);

  vTaskStartScheduler();

  return 0;
}

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
static void usb_device_task(void *param) {
  (void) param;

  DbgConsole_Printf("usb_device_task:A001\r\n");

  // init device stack on configured roothub port
  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tud_init(BOARD_TUD_RHPORT);

  DbgConsole_Printf("usb_device_task:A002\r\n");

  // RTOS forever loop
  while (1) {
    // put this thread to waiting state until there is new events
    tud_task();

    // following code only run if tud_task() process at least 1 event
    tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  DbgConsole_Printf("tud_mount_cb:A001\r\n");
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  DbgConsole_Printf("tud_umount_cb:A001\r\n");
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  DbgConsole_Printf("tud_suspend_cb:A001\r\n");
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  DbgConsole_Printf("tud_resume_cb:A001\r\n");
  if (tud_mounted()) {
    xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
  } else {
    xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
  }
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void *params) {
  (void) params;

  DbgConsole_Printf("cdc_task:A001\r\n");

  // RTOS forever loop
  while (1) {
    // connected() check for DTR bit
    // Most but not all terminal client set this when making connection
    // if ( tud_cdc_connected() )
    {
      // There are data available
      while (tud_cdc_available()) {
        uint8_t buf[64];

        // read and echo back
        uint32_t count = tud_cdc_read(buf, sizeof(buf));
        (void) count;

        // Echo back
        // Note: Skip echo by commenting out write() and write_flush()
        // for throughput test e.g
        //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
        tud_cdc_write(buf, count);
      }

      tud_cdc_write_flush();
    }

    // For ESP32-Sx this delay is essential to allow idle how to run and reset watchdog
    vTaskDelay(1);
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void) itf;
  (void) rts;

  DbgConsole_Printf("tud_cdc_line_state_cb\r\n");

  // TODO set some indicator
  if (dtr) {
    // Terminal connected
  } else {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void) itf;
  DbgConsole_Printf("tud_cdc_rx_cb\r\n");
}

static volatile long led_state;

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
static void led_blinky_cb(TimerHandle_t xTimer) {
  (void) xTimer;
  GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_GPIO_PIN, led_state);
  led_state = led_state ? 0 : 1;
  DbgConsole_Printf("(%ld) ", led_state);
}
