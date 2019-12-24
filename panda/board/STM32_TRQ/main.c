// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"

#include "safety_trq.h"

#define CAN CAN1

//#define PEDAL_USB

#ifdef PEDAL_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;


// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

// ********************* serial debugging *********************

#ifdef PEDAL_USB

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      if (ur == &esp_ring) {
        uart_dma_drain();
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** pedal can checksum *****************************

uint8_t pedal_checksum(uint8_t *dat, int len) {
  uint8_t crc = 0xFF;
  uint8_t poly = 0xD5; // standard crc8
  int i, j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_GAS_INPUT  0x250
#define CAN_GAS_OUTPUT 0x251U
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// two independent values, signed
int16_t torque_cmd_1 = 0;
int16_t torque_cmd_2 = 0;

#define MAX_TIMEOUT 10U
uint32_t timeout = 0;
uint32_t current_index = 0;

#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
uint8_t state = FAULT_STARTUP;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_GAS_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint32_t ts = TIM2->CNT;
      bool violation = 0;

      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      // two independent sensor values, signed
      int16_t value_0 = (dat[0] << 8) | dat[1];
      int16_t value_1 = (dat[2] << 8) | dat[3];
      bool enable = ((dat[4] >> 7) & 1U) != 0U;
      uint8_t index = dat[4] & COUNTER_CYCLE;
      if (pedal_checksum(dat, CAN_GAS_SIZE - 1) == dat[5]) {
        if (((current_index + 1U) & COUNTER_CYCLE) == index) {
          #ifdef DEBUG
            puts("setting gas ");
            puth(value_0);
            puts("\n");
          #endif
          if (enable) {
            //ripped from Toyota Safety

            // *** global torque limit check ***
            violation |= max_limit_check(value_0, TRQ_MAX, -TRQ_MAX);
            violation |= max_limit_check(value_1, TRQ_MAX, -TRQ_MAX);

            // // *** torque rate limit check ***
            // violation |= dist_to_meas_check(value_0, desired_torque_last_1,
            //   &torque_meas_1, MAX_RATE, MAX_RATE, MAX_TORQUE_ERROR);

            // violation |= dist_to_meas_check(value_1, desired_torque_last_2,
            //   &torque_meas_2, MAX_RATE, MAX_RATE, MAX_TORQUE_ERROR);

            // used next time
            // desired_torque_last_1 = value_0;
            // desired_torque_last_2 = value_1;

            // *** torque real time rate limit check ***
            violation |= rt_rate_limit_check(value_0, rt_torque_last_1, MAX_RT_DELTA);
            violation |= rt_rate_limit_check(value_1, rt_torque_last_2, MAX_RT_DELTA);

            // every RT_INTERVAL set the new limits
            uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
            if (ts_elapsed > RT_INTERVAL) {
              rt_torque_last_1 = value_0;
              rt_torque_last_2 = value_1;
              ts_last = ts;
            }

            // no torque if controls is not allowed
            if (!enable && ((value_0 != 0) || (value_1 != 0))) {
              violation = 1;
            }

            // reset to 0 if either controls is not allowed or there's a violation
            if (violation || !enable) {
              // desired_torque_last_1 = 0;
              // desired_torque_last_2 = 0;
              rt_torque_last_1 = 0;
              rt_torque_last_2 = 0;
              ts_last = ts;
            }

            if (violation) {
              state = FAULT_INVALID;
            }
            else {
              torque_cmd_1 = value_0;
              torque_cmd_2 = value_1;
            }
          } else {
            // clear the fault state if values are 0
            if ((value_0 == 0U) && (value_1 == 0U)) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
            torque_cmd_1 = 0;
            torque_cmd_2 = 0;
          }
          // clear the timeout
          timeout = 0;
        }
        current_index = index;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

uint32_t trq0 = 0;
uint32_t trq1 = 0;
unsigned int pkt_idx = 0;

int led_value = 0;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void TIM3_IRQ_Handler(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" ");
    puth(trq0);
    puts(" ");
    puth(trq1);
    puts("\n");
  #endif

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[0] = (trq0 >> 8) & 0xFFU;
    dat[1] = (trq0 >> 0) & 0xFFU;
    dat[2] = (trq1 >> 8) & 0xFFU;
    dat[3] = (trq1 >> 0) & 0xFFU;
    dat[4] = ((state & 0xFU) << 4) | pkt_idx;
    dat[5] = pedal_checksum(dat, CAN_GAS_SIZE - 1);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN->sTxMailBox[0].TDTR = 6;  // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_GAS_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
  } else {
    timeout += 1U;
  }
}

bool cycle = false;
// ***************************** main code *****************************

void pedal(void) {
  // read/write
  trq0 = adc_get(ADCCHAN_ACCEL0);
  trq1 = adc_get(ADCCHAN_ACCEL1);
  //bool override = ((trq0 >= 3000) | (trq1 >= 3000)); // TODO: find real oveeride values from an EPS

  // write the pedal to the DAC
  //0x0C14 is the absolute maximum of the DAC

  // int torque_meas_new_1 = trq0;
  // int torque_meas_new_2 = trq1;


  // update array of sample
  // update_sample(&torque_meas_1, torque_meas_new_1);
  // update_sample(&torque_meas_2, torque_meas_new_2);

  // increase torque_meas by 1 to be conservative on rounding
  // torque_meas_1.min--;
  // torque_meas_1.max++;
  // torque_meas_2.min--;
  // torque_meas_2.max++;

  if (state == NO_FAULT) { 
    bool enabled = (torque_cmd_1 != 0) && (torque_cmd_2 != 0);
    bool enabled_1 = (torque_cmd_1 != 0);
    bool enabled_2 = (torque_cmd_2 != 0);
    // interpolate between fake and real values
    // TODO: implement a rate limiter
    if (cycle){
      if (enabled){  
        // check the enable state for each DAC. need a bit flip on each bool (3) for this to fail badly.
        if (enabled_1){ 
          dac_set(0, (0x60A + (torque_cmd_1 / 2)));
        }
        if (enabled_2){ 
          dac_set(1, (0x60A + (torque_cmd_2 / 2)));
        }
      }
    }
    else {
      dac_set(0, trq0);
      dac_set(1, trq1); 
    }
    cycle = !cycle;
  } else {
    // deny CAN input and pass thru sensor values
      dac_set(0, trq0);
      dac_set(1, trq1); 
  }

  watchdog_feed();
}

int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();
  
  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();

#ifdef PEDAL_USB
  // enable USB
  usb_init();
#endif

  // pedal stuff
  dac_init();
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  llcan_init(CAN1);

  TIM2->PSC = 48-1;
  TIM2->CR1 = TIM_CR1_CEN;
  TIM2->EGR = TIM_EGR_UG;

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    pedal();
  }

  return 0;
}
