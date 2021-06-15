# stm32template
Linux/stm32/libopencm3/stm32flash template project

### Preparation
```bash
sudo apt install git gcc-arm-none-eabi stm32flash

cd ~/dev
git clone https://github.com/grytole/stm32template.git
cd ./stm32template
git submodule add https://github.com/libopencm3/libopencm3.git
git commit -m "import libopencm3"
make -C ./libopencm3
make
make flash
```

### Snippets
#### System Clock
```c
#include <libopencm3/stm32/rcc.h>

void clock_setup(void)
{
  /* Source: internal oscillator */
  /* RCC_CLOCK_HSI_{24, 48, 64}MHZ */
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);

  /* Source: external oscillator (8, 12, 16 or 25 MHz) */
  /* RCC_CLOCK_HSE{8_24, 8_72, 12_72, 16_72, 25_72}MHZ */
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  /* Peripheral clock frequencies globals (Hz) */
  uint32_t ahb = rcc_ahb_frequency;
  uint32_t apb1 = rcc_apb1_frequency;
  uint32_t apb2 = rcc_apb2_frequency;
}
```

#### GPIO
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  
  /* RCC_GPIO{A - G} */
  rcc_periph_clock_enable(RCC_GPIOB);
}

void gpio_setup(void)
{
  /* GPIO{A - G} */
  /* GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_{2_MHZ, 10_MHZ, 50_MHZ} */
  /* GPIO_CNF_INPUT_{ANALOG, FLOAT, PULL_UPDOWN},
     GPIO_CNF_OUTPUT_{PUSHPULL, OPENDRAIN, ALTFN_PUSHPULL, ALTFN_OPENDRAIN} */
  /* GPIO{0 - 15} */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
}

void do_something(void)
{
  uint16_t val = 0x0000;
  
  /* GPIO{A - G} */
  /* GPIO{0 - 15} */
  gpio_clear(GPIOB, GPIO11);
  gpio_set(GPIOB, GPIO12);
  gpio_toggle(GPIOB, GPIO12);
  val = gpio_get(GPIOB, GPIO11|GPIO12);
  val = gpio_port_read(GPIOB);
  gpio_port_write(GPIOB, val);
}
```

#### UART
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  /* RCC_GPIO{A - G} */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* RCC_USART{1 - 3}, RCC_UART{4 - 5} */
  rcc_periph_clock_enable(RCC_USART1);
}

void usart_setup(void)
{
  /* GPIO{A - G} */
  /* GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_{2_MHZ, 10_MHZ, 50_MHZ} */
  /* GPIO_CNF_INPUT_{ANALOG, FLOAT, PULL_UPDOWN},
     GPIO_CNF_OUTPUT_{PUSHPULL, OPENDRAIN, ALTFN_PUSHPULL, ALTFN_OPENDRAIN} */
  /* GPIO_USART1_{TX, RX, RE_TX, RE_RX} */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* USART{1 - 3}, UART{4 - 5} */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);

  /* USART_STOPBITS_{0_5, 1, 1_5, 2} */
  usart_set_stopbits(USART1, USART_STOPBITS_1);

  /* USART_MODE_{TX, RX, TX_RX} */
  usart_set_mode(USART1, USART_MODE_TX);

  /* USART_PARITY_{NONE, EVEN, ODD} */
  usart_set_parity(USART1, USART_PARITY_NONE);

  /* USART_FLOWCONTROL_{NONE, RTS, CTS, RTS_CTS} */
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  usart_enable(USART1);
}

void do_something(void)
{
  usart_send_blocking(USART1, '\r');
}
```

#### System Tick
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

volatile uint64_t ticks_ms = 0;

/* SysTick timer callback */
void sys_tick_handler(void)
{
  ticks_ms++;
}

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void systick_setup(void)
{
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  /* 1000Hz --> 1ms */
  systick_set_frequency(1000, rcc_ahb_frequency);
  systick_clear();
  systick_interrupt_enable();
  systick_counter_enable();
}

uint64_t ms(void)
{
  return ticks_ms;
}
```

#### External Interrupts
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  rcc_periph_clock_enable(RCC_SYSCFG);
}

void exti_setup(void)
{
  /* EXTI{0 - 4, 9_5, 15_10} */
  exti_enable_request(EXTI0);

  /* EXTI_TRIGGER_{RISING, FALLING, BOTH} */
  exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);

  /* GPIO{A - G} */
  exti_select_source(EXTI0, GPIOA);

  /* NVIC_EXTI{0 - 4, 9_5, 15_10}_IRQ */
  nvic_enable_irq(NVIC_EXTI0_IRQ);
}

/* exti{0 - 4, 9_5, 15_10}_isr */
void exti0_isr(void)
{
  /* EXTI{0 - 4, 9_5, 15_10} */
  exti_reset_request(EXTI0);

  ...
}
```

#### SPI Master
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  /* RCC_GPIO{A - G} */
  rcc_periph_clock_enable(RCC_GPIOB);

  /* RCC_SPI{1 - 3} */
  rcc_periph_clock_enable(RCC_SPI2);
}

void spi_setup(void)
{
  /* configure SCK=(PB13), MOSI=(PB15) pins as AF pushpull */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13 | GPIO15);

  /* configure MISO=(PB14) pin as floating input or input with pullup */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);
  /* enable internal pullup if required */
  gpio_set(GPIOB, GPIO14);
  
  /* configure NSS as pushpull (or opendrain if there is an external pullup) if required */
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  /* NSS should be high when no SPI exchange */
  gpio_set(GPIOB, GPIO12);

  /* SPI{1-3} */
  spi_reset(SPI2);

  /* SPI_CR1_BAUDRATE_FPCLK_DIV_{2, 4, 8, 16, 32, 64, 128, 256} */
  /* SPI_CR1_CPOL_CLK_TO_{0, 1}_WHEN_IDLE */
  /* SPI_CR1_CPHA_CLK_TRANSITION_{1, 2} */
  /* SPI_CR1_DFF_{8, 16}BIT */
  /* SPI_CR1_{MSB, LSB}FIRST */
  spi_init_master(SPI2,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                  SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2,
                  SPI_CR1_DFF_8BIT,
                  SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  spi_enable(SPI2);
}

void do_something(void)
{
  uint16_t tx_value = 0x0042;
  uint16_t rx_value = 0x0000;

  /* pull NSS low to select a slave device */
  gpio_clear(GPIOB, GPIO12);
  /* send to tx register - blocks until tx completed */
  spi_send(SPI2, tx_value);
  /* read from rx register - blocks until rx completed */
  rx_value = spi_read(SPI2);
  /* pull NSS high to unselect a slave device */
  gpio_set(GPIOB, GPIO12);

  /* pull NSS low to select a slave device */
  gpio_clear(GPIOB, GPIO12);
  /* data exchange - blocks until rx completed */
  rx_value = spi_xfer(SPI2, tx_value);
  /* pull NSS high to unselect a slave device */
  gpio_set(GPIOB, GPIO12);
}
```

#### CAN
```c
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>

void clock_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  /* RCC_GPIO{A - G} */
  rcc_periph_clock_enable(RCC_GPIOB);

  rcc_periph_clock_enable(RCC_CAN);
}

void can_setup(void)
{
  int32_t result = -1;

  bool ttcm = false; /* Time triggered communication mode */
  bool abom = true;  /* Automatic bus-off management */
  bool awum = false; /* Automatic wakeup mode */
  bool nart = false; /* No automatic retransmission */
  bool rflm = false; /* Receive FIFO locked mode */
  bool txfp = false; /* Transmit FIFO priority */
  /* This must be calculated */
  uint32_t sjw = CAN_BTR_SJW_1TQ; /* Resynchronization time quanta jump width */
  uint32_t ts1 = CAN_BTR_TS1_3TQ; /* Time segment 1 time quanta width */
  uint32_t ts2 = CAN_BTR_TS2_4TQ; /* Time segment 2 time quanta width */
  uint32_t brp = 12;     /* Baud rate prescaler */
  bool loopback = false; /* Loopback mode */
  bool silent = false;   /* Silent mode */

  uint32_t filter_id = 0;    /* ID number of the filter - 0-13 for non-connectivity devices */
  uint32_t msg_id = 0;       /* Message ID to filter */
  uint32_t msg_id_mask = 0;  /* Message ID bit mask - 0 bit: don't care; 1 bit: must match with ID */
  uint32_t fifo_id = 0;      /* FIFO ID - 0:FIFO0, 1:FIFO1 */
  bool filter_enable = true; /* Enable filter */

  /* Remap CAN to PB8/PB9 if needed */
  /* gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB); */

  /* CAN TX - PA12 (or PB9) */
  /* GPIO_BANK_{CAN1, CAN1_PB}_TX, GPIO_{CAN1, CAN1_PB}_TX */
  gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);
  
  /* CAN RX - PA11 (or PB8) */
  /* GPIO_BANK_{CAN1, CAN1_PB}_RX, GPIO_{CAN1, CAN1_PB}_RX */
  gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
  gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

  /* Enable interrupt */
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
  nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

  can_reset(CAN1);

  result = can_init(CAN1, ttcm, abom, awum, nart, rflm, txfp, sjw, ts1, ts2, brp, loopback, silent);
  if (0 != result)
  {
    /* CAN init failed! */
    return;
  }

  /* Route all incoming messages to FIFO0 */
  can_filter_id_mask_32bit_init(filter_id, msg_id, msg_id_mask, fifo_id, filter_enable);

  /* FMPIE0: FIFO message pending interrupt enable */
  can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void do_something(void)
{
  int32_t result = -1;
  bool has_available_mailbox = false;

  uint32_t id = 0x0000; /* Message ID */
  bool ext = false;     /* Extended ID */
  bool rtr = false;     /* Request transmit */
  uint8_t length = 8;   /* Message payload length */
  uint8_t data[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 }; /* Message payload data */

  has_available_mailbox = can_available_mailbox(CAN1);
  if (has_available_mailbox)
  {
    /* There is at least one empty mailbox ready for TX */
  }

  result = can_transmit(CAN1, id, ext, rtr, length, data);
  if (-1 == result)
  {
    /* No mailbox was available and no transmission got queued! */
    return;
  }
  else
  {
    /* result has a number of mailbox used: 0, 1, 2 */
  }
}

/* FIFO message pending interrupt */
void usb_lp_can_rx0_isr(void)
{
  uint8_t fifo = 0;     /* FIFO ID */
  bool release = true;  /* Release the FIFO automatically after coping data out */

  uint32_t id = 0x0000; /* Message ID */
  bool ext = false;     /* Extended ID */
  bool rtr = false;     /* Request transmit */
  uint8_t fmi = 0;      /* ID of the matched filter */
  uint8_t length = 8;   /* Message payload length */
  uint8_t data[8] = {}; /* Message payload data */
  
  can_receive(CAN1, fifo, release, &id, &ext, &rtr, &fmi, &length, data, NULL);

  ...
  
  /* call can_fifo_release(CAN1, fifo) here if release flag of can_receive() was false */
}
```

#### I2C
```c
/* TODO */
```

#### Timer
```c
/* TODO */
```
