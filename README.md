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
```

### Snippets
#### System Clock
```c
#include <libopencm3/stm32/rcc.h>

void clock_setup(void)
{
  /* Source: internal oscillator */
  rcc_clock_setup_in_hsi_out_64mhz(void);
  rcc_clock_setup_in_hsi_out_48mhz(void);
  rcc_clock_setup_in_hsi_out_24mhz(void);

  /* Source: external oscillator (8, 12, 16 or 25 MHz) */
  rcc_clock_setup_in_hse_8mhz_out_24mhz(void);
  rcc_clock_setup_in_hse_8mhz_out_72mhz(void);
  rcc_clock_setup_in_hse_12mhz_out_72mhz(void);
  rcc_clock_setup_in_hse_16mhz_out_72mhz(void);
  rcc_clock_setup_in_hse_25mhz_out_72mhz(void);

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
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  
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
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* RCC_GPIO{A - G} */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* RCC_USART{1 - 3}, RCC_UART{4 - 5} */
  rcc_periph_clock_enable(RCC_USART2);
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
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
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

#### I2C
```c
/* TODO */
```

#### Timer
```c
/* TODO */
```
