// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"

#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
//#include "drivers/pwm.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "drivers/eeprom.h"

#define CAN_BL_INPUT 0x1
#define CAN_BL_OUTPUT 0x2

#define CAN CAN1

#define PEDAL_UART

#ifdef PEDAL_UART
  #include "drivers/uart.h"
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
  void putch(unsigned int i) {
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

#ifdef PEDAL_UART

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

#endif

uint32_t eepromdata = 0;


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
#define CAN_GAS_INPUT  0x200
#define CAN_GAS_OUTPUT 0x201U
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_TX_IRQHandler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// two independent values
uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;

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

void bl_can_send(uint8_t *odat) {
  // wait for send
  while (!(CAN->TSR & CAN_TSR_TME0));

  // send continue
  CAN->sTxMailBox[0].TDLR = ((uint32_t*)odat)[0];
  CAN->sTxMailBox[0].TDHR = ((uint32_t*)odat)[1];
  CAN->sTxMailBox[0].TDTR = 8;
  CAN->sTxMailBox[0].TIR = (CAN_BL_OUTPUT << 21) | 1;
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_RX0_IRQHandler(void) {
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
        }else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }
    
      // normal packet
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      uint16_t value_0 = (dat[0] << 8) | dat[1];
      uint16_t value_1 = (dat[2] << 8) | dat[3];
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
            gas_set_0 = value_0;
            gas_set_1 = value_1;
          } else {
            // clear the fault state if values are 0
            if ((value_0 == 0U) && (value_1 == 0U)) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID;
            }
            gas_set_0 = 0;
            gas_set_1 = 0;
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
    if (address == CAN_BL_INPUT) {
      uint8_t dat[8];
      for (int i = 0; i < 8; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
        putch(dat[i]);
      }
      //uint8_t odat[8];
      bl_can_send(dat);
     }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}
// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_SCE_IRQHandler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

uint32_t pdl0 = 0;
uint32_t pdl1 = 0;
unsigned int pkt_idx = 0;

int led_value = 0;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void TIM3_IRQHandler(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" ");
    puth(pdl0);
    puts(" ");
    puth(pdl1);
    puts("\n");
  #endif

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[8];
    dat[0] = (pdl0 >> 8) & 0xFFU;
    dat[1] = (pdl0 >> 0) & 0xFFU;
    dat[2] = (pdl1 >> 8) & 0xFFU;
    dat[3] = (pdl1 >> 0) & 0xFFU;
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

// ***************************** main code *****************************

void pedal(void) {
  // read/write
  pdl0 = adc_get(ADCCHAN_ACCEL0);
  pdl1 = adc_get(ADCCHAN_ACCEL1);

  // write the pedal to the DAC
  // for now let's do nothing instead.
  // TODO: motor controls based on torque
  if (state == NO_FAULT) {
    // dac_set(0, MAX(gas_set_0, pdl0));
    // dac_set(1, MAX(gas_set_1, pdl1));
  } else {
    // dac_set(0, pdl0);
    // dac_set(1, pdl1);
  }

  watchdog_feed();
}

int main(void) {
  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init pwm (currently using PA6 and PA7)
  // pwm_init();
  // timer_init(TIM2, 240);
  // TIM2->CCR3 = 0;
  // TIM2->CCR4 = 0;
  // NVIC_EnableIRQ(TIM2_IRQn);
    // A4,A5,A6,A7: setup SPI
  //set_gpio_alternate(GPIOA, 4, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 5, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 6, GPIO_AF5_SPI1);
  set_gpio_alternate(GPIOA, 7, GPIO_AF5_SPI1);

  // A2, A3: USART 2 for debugging
  set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);

  // init board
  current_board->init();

#ifdef PEDAL_UART
  // enable uart
  uart_init(USART2, 115200);
#endif

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  llcan_init(CAN1);

  // 48mhz / 65536 ~= 732
  //timer_init(TIM3, 15);
  //NVIC_EnableIRQ(TIM3_IRQn);

  // spi eeprom
  eeprom_init();
  //eeprom_write(0x8, 0xCF); 


  //watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();
  // main pedal loop
  uint8_t bytebuf[4];
  //let's write the EEPROM!
  // for(int ii=0;ii<=0x20;ii++){
  //   eeprom_write(bytebuf, ii, 0xDC);
  // }
  uint16_t addr = 0;
  

  while (1) {
    puts("THE MAIN LOOP\n");
     // Wait RXNE (Receive not empty (we have data))
    current_board->set_led(LED_GREEN, led_value);
    led_value = !led_value;
    if (addr >= 0x20){
      addr = 0;
    }
    uint8_t eeprom_out = eeprom_read(bytebuf, addr);
    puth(eeprom_out);
    //uint8_t output = SPI1->DR;
    delay(1000U);
    watchdog_feed();
    addr++;
    //delay(1000U);
    //puts("\n");
    //pedal();
  }

  return 0;
}
