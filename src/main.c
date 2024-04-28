#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>



#define MY_GPIO0 DT_NODELABEL(gpio0)
#define LED_PIN 6

const struct device *gpio0_dev = DEVICE_DT_GET(MY_GPIO0);

int main(void)
{

  gpio_pin_configure(gpio0_dev, LED_PIN, GPIO_OUTPUT);

  while (1)
  {
    gpio_pin_toggle(gpio0_dev, LED_PIN);
    k_msleep(1000);
  }
}
