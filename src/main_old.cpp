#include "main.h"

using namespace pico_ssd1306;

static bool debug_nn = false;
volatile bool timer0_occur=false,timer1_occur=false;
const uint LED_PIN = 25;
/*
bool repeating_timer_callback(struct repeating_timer *t)
{
    static bool led_stat=0;
    led_stat=1-led_stat;
    gpio_put(LED_PIN,led_stat);
    return true;
}*/

void gesture_recognize()
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    uint slice_num = pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN);
// Initialize I2C port 0 and configuring Pins 0 and 1 for MPU6050
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
    }
   // struct repeating_timer timer;
    while (true) 
    
    {
        gpio_put(LED_PIN,1);
         ei_printf("\nStarting inferencing in 2 seconds...\n");
    sleep_ms(2000);
    gpio_put(LED_PIN,0);
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
   /* if(accelerometer[2]>=8000)
    multicore_fifo_push_blocking(4);
    else if(accelerometer[2]<=-7000)
    multicore_fifo_push_blocking(5);*/
    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        break;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        break;
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
        if(timer1_occur==true)
        {
            cancel_repeating_timer(&timer);
            timer0_occur=false;
        }
        if(timer0_occur==false)
        {
        add_repeating_timer_ms(200, repeating_timer_callback, NULL, &timer);
        timer0_occur=true;
        }
        */
        ei_printf("left-right triggered\n");
        multicore_fifo_push_blocking(1);
     
    }
    else if(result.classification[2].value>=0.8)
    {/*
        if(timer0_occur==true)
        {
        cancel_repeating_timer(&timer);
        timer1_occur=false;
        }
        if(timer1_occur==false)
        {
        add_repeating_timer_ms(800,repeating_timer_callback, NULL, &timer);
        timer1_occur=true;
        }
        */
        ei_printf("up-down triggered\n");
        multicore_fifo_push_blocking(2);
    }
    else 
    {
    
    multicore_fifo_push_blocking(3);
    }
//display.sendBuffer();
    
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    }
}

int main()
{
    stdio_init_all();
    multicore_launch_core1(gesture_recognize);
     //Initialize I2C port 1 and Configuring Pins 12 and 13 for SSD1306 OLED display
    i2c_init(SSD1306_I2C, 400000);
    gpio_set_function(SSD1306_SCL, GPIO_FUNC_I2C);
    gpio_set_function(SSD1306_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SSD1306_SCL);
    gpio_pull_up(SSD1306_SDA);
    
    sleep_ms(250);
    SSD1306 display = SSD1306(SSD1306_I2C, 0x3C, Size::W128xH64);
    //display.setOrientation(0);
    int16_t accel[3],gyro[3],time;
    uint32_t gesture;
    while(1)
    {
        mpu6050_read(accel,gyro,&time);
        if(accel[3]>=10000)
        {
            display.setOrientation(0);
        }
        else if(accel[3]<=-8000)
        {
            display.setOrientation(1);
        }
        if(multicore_fifo_rvalid())
        {
        gesture=multicore_fifo_pop_blocking();
        display.clear();
        if(gesture==1)
        {
        display.addBitmapImage(0, 10, 128, 64, image_left_right);
        drawText(&display, font_8x8, "LEFT-RIGHT", 20 ,0);
        }
        else if(gesture==2)
        {
        display.addBitmapImage(0,0,128,64,image_up_down);
        drawText(&display, font_8x8, "UP-DOWN", 30 ,0);
        }
        else if(gesture==3)
        {
        drawText(&display, font_16x32, "IDLE", 30 ,20);
        }
        gesture=-1;
        }
        display.sendBuffer();
        sleep_ms(200);
    }
    return 0;
}










