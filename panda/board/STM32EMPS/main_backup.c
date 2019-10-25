// ********************* Includes *********************
#include "../config.h"
#include "libc.h"

#include "main_declarations.h"

#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/dac.h"
#include "drivers/pwm.h"
#include "drivers/timer.h"

#include "gpio.h"

#define CAN CAN1

//#define PEDAL_USB
//#include "drivers/uart.h"

// #ifdef PEDAL_USB
// #include "drivers/uart.h"
// #include "drivers/usb.h"

//#else
// no serial either
void puts(const char *a)
{
  UNUSED(a);
}
void puth(unsigned int i)
{
  UNUSED(i);
}
void puth2(unsigned int i)
{
  UNUSED(i);
}
//#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void)
{
  early();
}

// ********************* serial debugging *********************


// void debug_ring_callback(uart_ring *ring)
// {
//   char rcv;
//   while (getc(ring, &rcv) != 0)
//   {
//     (void)putc(ring, rcv);
//   }
// }


#ifdef PEDAL_USB

int usb_cb_ep1_in(uint8_t *usbdata, int len, bool hardwired)
{
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(uint8_t *usbdata, int len, bool hardwired)
{
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(uint8_t *usbdata, int len, bool hardwired)
{
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired)
{
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest)
  {
  // **** 0xe0: uart read
  case 0xe0:
    ur = get_ring_by_number(setup->b.wValue.w);
    if (!ur)
    {
      break;
    }
    if (ur == &esp_ring)
    {
      uart_dma_drain();
    }
    // read
    while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
           getc(ur, (char *)&resp[resp_len]))
    {
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

uint8_t pedal_checksum(uint8_t *dat, int len)
{
  uint8_t crc = 0xFF;
  uint8_t poly = 0xD5; // standard crc8
  int i, j;
  for (i = len - 1; i >= 0; i--)
  {
    crc ^= dat[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ***************************** can port *****************************

// addresses to be used on CAN
#define CAN_GAS_INPUT 0x200
#define CAN_GAS_OUTPUT 0x201U
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_TX_IRQHandler(void)
{
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

uint32_t lka_torque_req = 0;
uint16_t driver_torque = 0;

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
void CAN1_RX0_IRQHandler(void)
{
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0)
  {
#ifdef DEBUG
    puts("CAN RX\n");
#endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_GAS_INPUT)
    {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface)
      {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e)
        {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e)
        {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0xca55e77e)
        {
          //do the eeprom flashing stuff here
          //enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          //puts("You got to params setting correctly");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        }
        else
        {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }
    }
      // normal packet
      uint8_t dat[8];
      for (int i = 0; i < 8; i++)
      {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      uint16_t value_0 = (dat[0] << 8) | dat[1];
      uint16_t value_1 = (dat[2] << 8) | dat[3];
      bool enable = ((dat[4] >> 7) & 1U) != 0U;
      uint8_t index = dat[4] & COUNTER_CYCLE;
      if (pedal_checksum(dat, CAN_GAS_SIZE - 1) == dat[5])
      {
        if (((current_index + 1U) & COUNTER_CYCLE) == index)
        {
#ifdef DEBUG
          puts("setting gas ");
          puth(value_0);
          puts("\n");
#endif
          if (enable)
          {
            lka_torque_req = value_0;
          }
          else
          {
            // clear the fault state if values are 0
            if ((value_0 == 0U) && (value_1 == 0U))
            {
              state = NO_FAULT;
            }
            else
            {
              state = FAULT_INVALID;
            }
            lka_torque_req = 0;
          }
          // clear the timeout
          timeout = 0;
        }
        current_index = index;
      }
      else
      {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void CAN1_SCE_IRQHandler(void)
{
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

uint16_t trq0 = 0;
uint16_t trq1 = 0;

unsigned int pkt_idx = 0;

int led_value = 0;

int32_t pwmch0_output = 0;
uint32_t pwmch1_output = 0;
uint32_t pwmch2_output = 0;
bool dir = false;

// void TIM2_IRQHandler(void)
// {
//   //sets the PWM channel values on a clock interrupt
//   if (TIM2->SR & TIM_SR_UIF) // if UIF flag is set
//   {
//     TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
//     // TODO: for some reason, i can't set PWM in a seperate function
//     TIM2->CCR2 = pwmch0_output;
//     TIM2->CCR3 = pwmch1_output;
//     TIM2->CCR4 = pwmch2_output;
//   }
// }

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void TIM3_IRQHandler(void)
{
#ifdef DEBUG
  puth(TIM3->CNT);
  puts(" ");
  puth(trq0);
  puts(" ");
  puth(trq1);
  puts("\n");
#endif
  puts("TIM3_IRQHandler");

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
  {
    uint8_t dat[8];
    //TODO: have the output reflect the measured driver input torque rather than sensor output
    dat[0] = (trq0 >> 8) & 0xFFU;
    dat[1] = (trq0 >> 0) & 0xFFU;
    dat[2] = (trq1 >> 8) & 0xFFU;
    dat[3] = (trq1 >> 0) & 0xFFU;
    dat[4] = ((state & 0xFU) << 4) | pkt_idx;
    dat[5] = pedal_checksum(dat, CAN_GAS_SIZE - 1);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN->sTxMailBox[0].TDTR = 6; // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_GAS_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  }
  else
  {
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
  if (timeout == MAX_TIMEOUT)
  {
    state = FAULT_TIMEOUT;
  }
  else
  {
    timeout += 1U;
  }
}



// ***************************** main code *****************************

void motor_controller(void)
{
  // read/write
  // each is a 12 bit message. using PC0 and PC1 (ADC 10 and 11)
  trq0 = adc_get(10);
  trq1 = adc_get(11);

  //TODO: eeprom reading at init + torque scaling based on zero point and deadzone

  //write the pedal to the DAC
  // if (state == NO_FAULT)
  // {
  //   //dummy output for the time being. the torque req needs to be split depending on direction
  //   TIM2->CCR2 = MAX(lka_torque_req, trq0);
  //   TIM2->CCR3 = MAX(lka_torque_req, trq1);
  // }
  // else
  // {
  //   //also dummy output
  //   TIM2->CCR2 = trq0; //lka_torque_req;
  //   TIM2->CCR3 = trq1; //lka_torque_req;
  // }

  //set PA6
  //let's do something with this. maybe a relay enable? also, we have PA7 and PA3(PWM) still.
  //set_gpio_output(GPIOA, 6, 1);

  watchdog_feed();
}

int main(void)
{
  __disable_irq();

  // init devices
  clock_init();
  peripherals_init();

  detect_configuration();
  detect_board_type();

  // pwm_init();

  // timer_init(TIM2, 240);

  // TIM2->CCR3 = 0;
  // TIM2->CCR4 = 0;

  // NVIC_EnableIRQ(TIM2_IRQn);
  

#ifdef PEDAL_USB
  // enable USB
  usb_init();
#endif

  // pedal stuff
  dac_init();
  adc_init();

  // can is messing up pwm for some reason?
  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set)
  {
    puts("Failed to set llcan speed");
  }

  llcan_init(CAN1);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);

  NVIC_EnableIRQ(TIM3_IRQn);
  
  // A2, A3: USART 2 for debugging
  // set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  // set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);

  // uart_init(USART2, 115200);

  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  __enable_irq();

  // main pedal loop
  while (1)
  {
    //puts("TESTING");
    //puts("\n");
    //delay(100000);
    motor_controller();
  }

  return 0;
}
