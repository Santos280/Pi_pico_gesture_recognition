#include<stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "mpu6050.h"
#include "ei_run_classifier.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"

#define SCL 1
#define SDA 0
#define CONVERT_G_TO_MS2    9.80665f

static bool debug_nn = false;
volatile bool timer0_occur=false,timer1_occur=false;
const uint LED_PIN = 17;

bool repeating_timer_callback(struct repeating_timer *t)
{
    static bool led_stat=0;
    led_stat=1-led_stat;
    gpio_put(LED_PIN,led_stat);
    return true;
}

int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    //pwm_clear_irq(slice_num);
    //pwm_set_irq_enabled(slice_num, true);
    

    i2c_init(I2C_PORT,100000);
    gpio_set_function(SCL,GPIO_FUNC_I2C);
    gpio_set_function(SDA,GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
     mpu6050_reset();
    

    ei_impulse_result_t result = {0};


   int16_t accelerometer[3],gyro[3],temp;


    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return 0;
    }
    struct repeating_timer timer;
   // multicore_launch_core1(led_action);
    while (true) 
    
    {
         ei_printf("\nStarting inferencing in 2 seconds...\n");

    sleep_ms(2000);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        //uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
        mpu6050_read(accelerometer,gyro,&temp);
        buffer[ix + 0]= accelerometer[0];
        buffer[ix + 1]= accelerometer[1];
        buffer[ix + 2]= accelerometer[2];
        //IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;
        sleep_ms(1);
        //delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return 0;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return 0;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
    
    if(result.classification[1].value>=0.8)
    {/*
    irq_set_exclusive_handler(PWM_IRQ_WRAP, led_fade);
    pwm_config config = pwm_get_default_config();

    irq_set_enabled(PWM_IRQ_WRAP, true);

  
    pwm_config_set_clkdiv(&config, 4.f);
    
    pwm_init(slice_num, &config, true);
        */
        if(timer1_occur==true)
        {
            cancel_repeating_timer(&timer);
            timer0_occur=false;
        }
        if(timer0_occur==false)
        {
        add_repeating_timer_ms(200, repeating_timer_callback, NULL, &timer);
       // pwm_set_irq_enabled(slice_num, true);
        
        timer0_occur=true;
        }
        ei_printf("left-right triggered\n");
     //   multicore_fifo_push_blocking(1);
     
    }
    else if(result.classification[2].value>=0.8)
    {/*
    irq_set_exclusive_handler(PWM_IRQ_WRAP, led_blink);
    pwm_config config = pwm_get_default_config();

    irq_set_enabled(PWM_IRQ_WRAP, true);

  
    pwm_config_set_clkdiv(&config, 4.f);
    
    pwm_init(slice_num, &config, true);
        */
        if(timer0_occur==true)
        {
        cancel_repeating_timer(&timer);
      // pwm_set_irq_enabled(slice_num, false);
       
        timer1_occur=false;
        }
        if(timer1_occur==false)
        {
        add_repeating_timer_ms(800,repeating_timer_callback, NULL, &timer);
        timer1_occur=true;
        }
        ei_printf("up-down triggered\n");
      // add_repeating_timer_ms(500, repeating_timer_callback, NULL, &timer);
    
    }

    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    }
return 0;
}










