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

static void clock_setup(void)
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

#### I2C
```c
/* TODO */
```

#### Timer
```c
/* TODO */
```
