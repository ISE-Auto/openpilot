// IRQs: DMA2_Stream2, DMA2_Stream3, EXTI4

void eeprom_init(void);
#define READ 0x03;
#define WRITE 0x02;

void eeprom_init(void)
{                                     
  //RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //Enable SPI clock

  //Set baud prescaler
  SPI1->CR1 = SPI_CR1_BR_2; //Slowest SPI I can have

  //CPHA CPOL
  SPI1->CR1 &= ~(SPI_CR1_CPOL); //| SPI_CR1_CPHA); //
  SPI1->CR1 |= SPI_CR1_CPHA; 

  //8 bit data format
  SPI1->CR1 &= ~(SPI_CR1_DFF);

  //Full duplex mode
  SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);
  SPI1->CR1 &= ~(SPI_CR1_RXONLY);

  //MSB first
  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);

  //Master mode
  SPI1->CR1 |= SPI_CR1_MSTR;

  //Software NSS mode off
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

  //Enable
  SPI1->CR1 |= SPI_CR1_SPE;

  //Enable the SPI GPIO pins
}

uint8_t spi_write(uint8_t *byte, int len)
{
  SPI1->DR = 0x0 | 0x40;
  set_gpio_output(GPIOA, 4, 0);
  for (int ii = 0; ii <= len; ii++)
  {
    SPI1->DR = byte[ii]; // & 0xFF; //send 1 byte at a time. TODO: fix
    while (!(SPI1->SR & SPI_SR_TXE))
    ;
    while (!(SPI1->SR & SPI_SR_RXNE))
    ; // Wait RXNE (Receive not empty (we have data))
    while (SPI1->SR & (SPI_SR_BSY))
    ; // wait for not busy
  }
  while (!(SPI1->SR & SPI_SR_TXE))
  ;
  uint8_t ret = SPI1->DR;
  set_gpio_output(GPIOA, 4, 1);
  return ret;
}

uint8_t eeprom_read(uint8_t* buf, uint16_t addr)
{
  memset(buf, 0x0, 4);
  buf[0] = READ;
  buf[1] = (uint8_t)(addr >> 8)&0xFF;
  buf[2] = (uint8_t)(addr >> 0)&0xFF;
  buf[3] = 0x0;
  uint8_t byte = spi_write(buf, 4);
  return byte;
}

void eeprom_write(uint8_t* buf, uint16_t addr, uint8_t data)
{
  memset(buf, 0x0, 4);
  buf[0] = WRITE;
  buf[1] = (uint8_t)(addr >> 8)&0xFF;
  buf[2] = (uint8_t)(addr >> 8)&0xFF;
  buf[3] = data;
  spi_write(buf, 4);
}
