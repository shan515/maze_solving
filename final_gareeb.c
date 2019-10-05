  //#include <bits/stdc++.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "SRA18.h"
#include "TUNING.h"
//using namespace std;

/*
LUL - S
RUR - S
LUR - U
RUL - U
LUS - R
RUS - L
SUL - R
SUR - L
SUS - U
LRRRL - R
SRRRS - L
RLLLR - L
SLLLS - R
*/


adc1_channel_t channel[4] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3};

int weights[4] = {3,1,-1,-3};

/*
 * Line Following PID Constants
 */
#define kP 1
#define kI 0
#define kD 1.5

/*
 * Motor value constraints
 */
float opt = 75;
float lower_pwm_constrain = 65;
float higher_pwm_constrain = 85;
float left_pwm = 0, right_pwm = 0;
int x,y,fx,node;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[4];
float sensor_value[4];


static void read_sensors()
{
  for(int i = 0; i < 4; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
    }
}

static void extra_sensors()
{

     gpio_set_direction(GPIO_NUM4,GPIO_MODE_INPUT);
     gpio_set_direction(GPIO_NUM5,GPIO_MODE_INPUT);
     gpio_set_direction(GPIO_NUM6,GPIO_MODE_INPUT);

     fx=gpio_get_level(GPIO_NUM4);  //left32
     y=gpio_get_level(GPIO_NUM5);  //right19
     x=gpio_get_level(GPIO_NUM6);  //front18

}

static void calc_sensor_values()
{
    for(int i = 0; i < 4; i++)
    {
        sensor_value[i] = map(adc_reading[i], 1700, 4000, 0, 1000);
        sensor_value[i] = constrain(sensor_value[i],0,1000);
    }

}

static void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0, pos = 0;

    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
        {
            all_black_flag = 0;
        }

        weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        sum += sensor_value[i];

    }

    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 2.5;
        else
            pos = -2.5;
    }

    error = pos;

}

static void calculate_correction()
{
    error *= 10;
    difference = (error - prev_error);
    cumulative_error += error;

    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }

    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }

    correction = kP*error + kI*cumulative_error + kD*difference;
    prev_error = error;
}

char string[30] = { };
int v=0,len1=0;
char* nodecheck()
{
    extra_sensors();

    if (fx==0)  //left turn
    {

        extra_sensors();
        read_sensors();
        while(1)
        {
        bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0,65,65);
        extra_sensors();
        read_sensors();
        if(x==0)
            break;
        }
        if(fx==0)
        {
          string[v]='E';
          v++;
          bot_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
          vTaskDelay( 10000 / portTICK_PERIOD_MS );
        }
        read_sensors();
        calc_sensor_values();
        if(sensor_value[1]<150 && sensor_value[2]<150)
        {
          string[v]='L';
          v++;
          while(1)
            {
            bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
            read_sensors();
            calc_sensor_values();
            if(sensor_value[1]>150&&sensor_value[2]>150)
              break;
            }

        }
        else if(sensor_value[1]>150 && sensor_value[2]>150)
        {
          string[v]='L';
          v++;
          while(1)
                {
                    bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                    read_sensors();
                    calc_sensor_values();
                    if(sensor_value[1]<150 )
                        break;
                }
            while(1)
                {
                    bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 80, 80);
                    read_sensors();
                    calc_sensor_values();
                    if(sensor_value[1]>150 && sensor_value[2]>150)
                        break;
                }

        }
    }

        else if(y==0) // right and straight
        {
            read_sensors();
            calc_sensor_values();

            if(sensor_value[1]>150 || sensor_value[2]>150)
            {
                string[v] = 'S';
                v++;
                while(1)
                {
                    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);
                    extra_sensors();
                    if(x==1 && y==1)
                        break;
                }
            }
            else if(sensor_value[1]<150 && sensor_value[2]<150)
            {
                string[v] = 'R';
                v++;
                while(1)
                {
                    bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 80, 80);
                    read_sensors();
                    calc_sensor_values();
                    if(sensor_value[1]>150 && sensor_value[2]>150)
                        break;
                }
            }

        }
        else if(x==1&&y==1&&fx==1&&sensor_value[1]<150&&sensor_value[2]<150)
        {
          string[v] = 'U';
          v++;
          while(1)
          {
            bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 80, 80);
            read_sensors();
            calc_sensor_values();
            if(sensor_value[1]>150 && sensor_value[2]>150)
                break;
          }
        }
        len1 = sizeof(string);
        //char b[len1];
        int flag =0;
        for (int i=0 ; i<len1; i++)
        {
        	while(string[i] != 'E')
        	{
        		flag+=1;
        	}
        }
        char b[flag];
        return b;


}


void string_reduction (char a[])
{
	int count=0;
  //char a[] = {'L','R','R','L','L','U','L','R','U','R','L','L','U','S','L','L','L','R'};
    int len = sizeof(a);
    //char a[len];
    for(int i=0; i<len; i++)
    {
      if(a[i+1]=='U')
      {
        count += 2;
        if(a[i]=='L')
        {
          if(a[i+2]=='L')
          {
            a[i]= 'S';
          }
          else if(a[i+2]=='R')
          {
            a[i]= 'U';
          }
          else if(a[i+2]=='S')
          {
            a[i]= 'R';
          }
        }
        else if(a[i]=='R')
        {
          if(a[i+2]=='R')
          {
            a[i] ='S';
          }
          else if(a[i+2]=='L')
          {
            a[i] = 'U';
          }
          else if(a[i+2]=='S')
          {
            a[i] = 'L';
          }
        }
        else if(a[i]=='S')
        {
          if(a[i+2]=='L')
          {
            a[i] = 'R';
          }
          else if(a[i+2]=='R')
          {
            a[i] = 'L';
          }
          else if(a[i+2] == 'S')
          {
            a[i] = 'U';
          }
        }

        for(int k=i; k<len-3;k++)
        {
          a[k+1] = a[k+3];
        }
        for(int l=len-1; l>((len-1)-count); l--)
        {
          a[l] = 'X';
        }
        printf("%d\n", count );
        printf("%d\ni is : ",i);
        printf("\n");
      }
      else if((a[i+1]==a[i+2])&&(a[i+2]==a[i+3])&&(a[i]==a[i+4])&&(a[i]!='X'))
      {
        printf("%d\n", 111);
        count += 4;
        if((a[i]=='L')&&(a[i+1]=='R'))
        {
          a[i] = 'R';
        }
        else if((a[i]=='S')&&(a[i+1]=='R'))
        {
          a[i] = 'L';
        }
        else if((a[i]=='R')&&(a[i+1]=='L'))
        {
          a[i] = 'L';
          printf("%d\n",90  );
        }
        else if((a[i]=='S')&&(a[i+1]=='L'))
        {
          a[i] = 'R';
        }
        for(int k=i; k<len-5;k++)
        {
          a[k+1] = a[k+5];
        }
        for(int l=len-1; l<(len-1)-count; l--)
        {
          a[l] = 'X';
        }
        printf("%d\n", count);
      }
      printf("%s\n", a);

    }
    printf("%d\n", count );
    int j = len - count;
    char final[j];
    int x=0;
    for(int i=0;i<j;i++)
    {
      final[i] = a[i];
      x++;
    }
    printf("%d\n", x );
    printf("%s\n",final);
    save_to_flash(final);
    char* ms = get_from_flash();
    printf("message: %s\n", ms);


}

void follow_node ()
{
	char *path =get_from_flash();
	int count =0;
	int len = sizeof(path);
	for (int i=0; i<len ;i++)
	{
		if (sensor_value[1]>150 && sensor_value[2]>150 && (sensor_value[0] || sensor_value[3]))
		{
			count+=1;
			if (path[i] == 'L')
			{
				bot_spot_left(MCPWM_UNIT_0, MCPWM_TIMER_0, 70, 70);

			}

			else if (path[i] == 'R')
			{
				bot_spot_right(MCPWM_UNIT_0, MCPWM_TIMER_0, 80, 80);
			}

			else
			{
				bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
			}
		}
	}

}




void line_follow_task(void *arg)
{
  enable_buttons();
  mcpwm_initialize();

  while(1)
  {
    read_sensors();

    calc_sensor_values();
    calculate_error();
    calculate_correction();
    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
    char* b = nodecheck();
    //nodecheck();
    string_reduction(b);

    if(pressed_switch(BUTTON_1))
    	break;

}
while(1)
{
    read_sensors();

    calc_sensor_values();
    calculate_error();
    calculate_correction();
    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
    //nodecheck();
    follow_node ();
    //string_reduction(b);

}

}

void app_main()
{
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
}
