/*

Copyright (c) 2018, Society of Robotics and Automation, VJTI

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "SRA18.h"

//Functions for custom adjustments
float map(float x, float min_in, float max_in, float min_out, float max_out)
{
    return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

float constrain(float x, float lower_limit, float higher_limit)
{
    if(x < lower_limit)
        x = lower_limit;
    
    else if(x > higher_limit)
        x = higher_limit;

    return x;
}

float absolute(float number)
{
    if(number < 0)
    {
        return (-1)*number;
    }

    return number;
}

//Assign GPIO 2 and 15 to buttons
void enable_buttons()
{
    gpio_set_direction(BUTTON_1,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_1,GPIO_PULLUP_ONLY);
    gpio_set_direction(BUTTON_2,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_2,GPIO_PULLUP_ONLY);
}

//Check if the specified button is pressed, returns 1 if pressed
int pressed_switch(int button_num)
{
    return !gpio_get_level(button_num);    
}

//Change ADC parameters if required
 void adc1_init(adc1_channel_t channel[4])
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    for(int i = 0;i < 4;i++)
    {
      adc1_config_channel_atten(channel[i], ADC_ATTEN_11db);
    }
}


//Initialise GPIOs for MCPWM
 void mcpwm_gpio_initialize()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

//Intialise MCPWM 
 void mcpwm_initialize()
{
    mcpwm_gpio_initialize();
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    printf("Configuring pwm_config...\n");
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    printf("Initialize pwm_init...\n");
    gpio_set_direction(GPIO_NUM0, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM3, GPIO_MODE_OUTPUT);
    printf("Set direction to GPIO pins...\n");
}

//Functions to control bot motion
 void bot_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); 
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); 
}

 void bot_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,0);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); 
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); 
}

 void bot_spot_left(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,0);
    gpio_set_level(GPIO_NUM2,0);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

 void bot_spot_right(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,0);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

 void bot_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}


void save_to_flash(char* message)
{
    // function to final solution to the flash memory

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
     
    else 
    {
        printf("Done\n");

        // Write
        printf("Updating message in NVS ... ");

        err = nvs_set_str(my_handle, "message", message);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
}

char* get_from_flash()
{
    char* value = " ";
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else 
    {
        printf("Done\n");

        // Read
        printf("reading string value... ");
        
        size_t string_size;
        esp_err_t err = nvs_get_str(my_handle, "message", NULL, &string_size);
        value = malloc(string_size);
        err = nvs_get_str(my_handle, "message", value, &string_size);

        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("message%s\n", value);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        nvs_close(my_handle);
    }

    return value;    
}


void app_main()
{
    vTaskDelay(2000/portTICK_PERIOD_MS);
    xTaskCreate(&run_task,"run", 10000, NULL, 1, NULL);
}
