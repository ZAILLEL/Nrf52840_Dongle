#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>



#define MY_GPIO0 DT_NODELABEL(gpio0)
#define LED_PIN 6

const struct device *gpio0_dev = DEVICE_DT_GET(MY_GPIO0);

int main(void)
{

  gpio_pin_configure(gpio0_dev, LED_PIN, GPIO_OUTPUT);
  
  max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);
  max30102_clear_fifo(&max30102);
  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);
  
  // Sensor settings
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
  max30102_set_sampling_rate(&max30102, max30102_sr_800);
  max30102_set_led_current_1(&max30102, 6.2);
  max30102_set_led_current_2(&max30102, 6.2);

  // Enter SpO2 mode
  max30102_set_mode(&max30102, max30102_spo2);
  max30102_set_a_full(&max30102, 1);
  
  // Initiate 1 temperature measurement
  max30102_set_die_temp_en(&max30102, 1);
  max30102_set_die_temp_rdy(&max30102, 1);
  
  uint8_t en_reg[2] = {0};
  max30102_read(&max30102, 0x00, en_reg, 1);
  
  

  while (1)
  {
    gpio_pin_toggle(gpio0_dev, LED_PIN);

	if (max30102_has_interrupt(&max30102))
    {
      max30102_interrupt_handler(&max30102);
    }
	
    k_msleep(1000);
  }
}
