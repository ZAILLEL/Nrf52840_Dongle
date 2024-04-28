#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

#define MY_GPIO0 DT_NODELABEL(gpio0)
#define LED_PIN 6

const struct device *gpio0_dev = DEVICE_DT_GET(MY_GPIO0);

int main(void)
{

  if (!adc_is_ready_dt(&adc_channel))
  {
    printk("ADC controller devivce %s not ready", adc_channel.dev->name);
    return 0;
  }

  if (adc_channel_setup_dt(&adc_channel) < 0)
  {
    printk("Could not setup channel #%d (%d)", 0, adc_channel_setup_dt(&adc_channel));
    return 0;
  }

  int16_t buf;

  struct adc_sequence sequence = {
      .buffer = &buf,
      .buffer_size = sizeof(buf),
      // Optional
      //.calibrate = true,
  };

  if (adc_sequence_init_dt(&adc_channel, &sequence) < 0)
  {
    printk("Could not initalize sequnce");
    return 0;
  }

  uint32_t val_mv;
  printk(" adc res  = %d \r\n", adc_channel.resolution);

  gpio_pin_configure(gpio0_dev, LED_PIN, GPIO_OUTPUT);

  while (1)
  {

    adc_read(adc_channel.dev, &sequence);
    adc_raw_to_millivolts_dt(&adc_channel, &val_mv);

    printk("raw = %d \r\n", val_mv);

    gpio_pin_toggle(gpio0_dev, LED_PIN);
    k_msleep(1000);
  }
}
