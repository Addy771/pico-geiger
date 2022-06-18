/**
 *
 */
#define PICO_DEFAULT_UART_BAUD_RATE 921600

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico-oled/pico-oled.hpp"

#include "bitmap/radiation.h"
#include "bitmap/picotext.h"

#include "pico-oled/gfx_font.h"
#include "pico-oled/font/press_start_2p.h"
#include "pico-oled/font/too_simple.h"


#define DISPLAY_I2C_ADDR _u(0x3C)
#define DISPLAY_WIDTH _u(128)
#define DISPLAY_HEIGHT _u(64)

#define PWM_GPIO 20
#define PSU_MODE_GPIO 23
#define VBUS_SENSE_GPIO 24
#define HVPS_ADC_GPIO 26
#define VSYS_ADC_GPIO 29
#define CHG_DONE_GPIO 22
#define PB1_GPIO 10
#define PB2_GPIO 11
#define PB3_GPIO 12
#define DEBUG_GPIO 2

#define DEFAULT_HVPS_SP 390.0   // Default setpoint for the HVPS in volts
#define HVPS_RAMP_RATE 0.1      // Magnitude to ramp in volts. HVPS setpoint changes by this magnitude each control loop iteration

#define PWM_FREQ 20000          // PWM switching frequency in Hz
#define CTRL_LOOP_FREQ 1000     // Control loop update frequency in Hz
#define PWM_DUTY_MAX 80         // Maximum PWM duty cycle in percent
#define PWM_MIN_PULSE 5         // Minimum on-time, in microseconds

float HVPS_p_gain = 7;          // Proportional gain
float HVPS_i_gain = 0;          // Integral gain

uint16_t pwm_top = 0;
uint16_t pwm_min_duty = 0;
uint16_t pwm_max_duty = 0;
uint16_t ctrl_skip_cnt;
float HVPS_setpoint;
float vsys = 0.0;
float v_sys = 0.0;
float v_hv = 0.0;
int16_t pwm_duty = 0;

// PWM states
enum 
{
    STANDBY = 0,
    RAMP_TO_TARGET,
    PWM_REGULATING
};

// Fault bits
enum
{
    HVPS_REGULATION_FAIL
};

uint8_t pwm_state = STANDBY;
uint faults = 0;    // 

void core1_entry();


// Core 0
int main()
{
    // Start core 1
    multicore_launch_core1(core1_entry);

    stdio_init_all();

    // Init i2c and configure it's IO pins
    //uint32_t i2c_clk = i2c_init(i2c_default, 400 * 1000);
    uint32_t i2c_clk = i2c_init(i2c_default, 1000 * 1000);  // 1120 kHz max
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    //sleep_ms(1000);
    
    // Instantiate display and initialize it
    pico_oled display(DISPLAY_I2C_ADDR, DISPLAY_WIDTH, DISPLAY_HEIGHT);   
    display.oled_init();

    //display.set_font(too_simple);
    display.set_font(press_start_2p);

    // Print out some system information
    uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint32_t start_time, end_time;

    uint8_t box_w = radiation.width+4;
    uint8_t box_h = radiation.height+4;
    uint8_t box_x = 0;
    uint8_t box_y = 0;
    uint8_t box_h_dir = 1;
    uint8_t box_v_dir = 1;

    // // Show a page of debug values
    // display.set_cursor(0,0);
    // display.print("CLK_SYS: ");
    // display.print_num("%d kHz\n", f_clk_sys);
    // display.print("PWM_TOP: ");
    // display.print_num("%d\n", pwm_top);
    // display.print("PWM_MIN_DUTY: ");
    // display.print_num("%d\n", pwm_min_duty);    
    // display.print("PWM_MAX_DUTY: ");
    // display.print_num("%d\n", pwm_max_duty);    
    // display.print("CTRL_SKIP_CNT: ");
    // display.print_num("%d", ctrl_skip_cnt);        

    
    // display.render();
    // sleep_ms(1000);    

    while (true)
    {
        display.fill(0);    // Clear display
        display.set_cursor(0,0);
        display.print("V_SYS: ");
        display.print_num("%.3fV\n", v_sys);        
        display.print("PWM_STATE: ");
        display.print_num("%d \n", pwm_state);        
        display.print("HVPS_SET: ");
        display.print_num("%.1fV\n", HVPS_setpoint);
        display.print("V_HV: ");
        display.print_num("%.1fV\n", v_hv);        
        display.print("pwm_duty: ");
        display.print_num("%d\n", pwm_duty);

        display.render();
        sleep_ms(10);
    }

    //// 1!!1!
    //reset_usb_boot(0, 2);   // Reset, picoboot disabled

    end_time = time_us_32() + 1000; // Set a reasonable default value
    while (true)
    {
        start_time = time_us_32();
        //sleep_ms(500);
        display.fill(0); // Clear display buffer

        // Draw coordinates at top left
        display.set_cursor(0, 0);
        display.print("V_SYS: ");
        display.print_num("%.3f V", vsys);


        // Draw box window at it's current location
        //display.blit_screen(landscape.bitmap, landscape.width, box_x, box_y, box_w, box_h, box_x, box_y);
        //display.blit_screen(suzie.bitmap, suzie.width, 135, 51, box_w, box_h, box_x, box_y);

        display.blit_screen(radiation.bitmap, radiation.width, 0, 0, radiation.width, radiation.height, box_x+2, box_y+2);
      
        
        display.draw_line(0,0,box_x, box_y);                            // U/L corner
        display.draw_line(127,0,box_x + box_w-1, box_y);                // U/R corner
        display.draw_line(127,63,box_x + box_w-1, box_y + box_h-1);     // B/R corner
        display.draw_line(0,63,box_x, box_y + box_h-1);                 // B/R corner

        start_time = time_us_32();  
        display.draw_line(box_x, box_y, box_x + box_w-1, box_y);        // Top 
        display.draw_line(box_x, box_y, box_x, box_y + box_h-1);        // Left
        display.draw_line(box_x + box_w-1, box_y + box_h-1, box_x + box_w-1, box_y);    // Right
        display.draw_line(box_x, box_y + box_h-1, box_x + box_w-1, box_y + box_h-1);    // Bottom
        end_time = time_us_32() - start_time;   
   
        display.render();       

        // Update box horizontal position
        if (box_h_dir)
        {
            box_x++;

            // Check if box is hitting the edge
            if (box_x + box_w == DISPLAY_WIDTH)
                box_h_dir = 0;
        }
        else
        {
            box_x--;
            
            if (box_x == 0)
                box_h_dir = 1;
        }

        // Update box vertical position
        if (box_v_dir)
        {
            box_y++;

            if (box_y + box_h == DISPLAY_HEIGHT)
                box_v_dir = 0;
        }
        else
        {
            box_y--;

            if (box_y == 0)
                box_v_dir = 1;
        }
    }


    while(true);
}


// Runs control loop update
// For 20 kHz PWM, pwm period is 50 us -> 5400 sys_clock cycles 
// For 50 kHz PWM, pwm period is 20 us -> 2160 sys_clock cycles
void on_pwm_wrap()
{
    // Max voltage of vsys divider is VREF * (200+100k) / 100k: 9.9V
    const float vsys_conversion_factor = (3.3f * (300)/100) / (1 << 12);

    // Max voltage of hvfb divider is VREF * (4.7M + 4.7M + 47k) / 47k: 663.3V
    //const float hvfb_conversion_factor = (3.3f * (9400 + 47)/47) / (1 << 12);
    const float hvfb_conversion_factor = (3.3f * (9400 + 45.3)/45.3) / (1 << 12);       //// fudged to compensate for ADC parallel resistance, Rb ~= 45.3k


    static uint v_sys_raw;
    static uint v_hv_raw;
    static float target_setpoint = 0.0;
    static uint16_t int_count = 0;
    static float ramp_rate;

    // Clear interrupt flag
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_GPIO));


    // Skip loops until count indicates that control loop should run
    if (++int_count < ctrl_skip_cnt)
        return;
    else
        int_count = 0;

    // Delay so that ADC samples aren't taken during the PWM switching edge
    //// vary delay time according to PWM duty
    uint start_time = time_us_32();  
    while (time_us_32() - start_time < 5);

    // Set debug IO to mark start of control loop
    gpio_put(DEBUG_GPIO, 1);

    adc_select_input(HVPS_ADC_GPIO - 26);    // Sample HVFB (ADC0)
    v_hv_raw = adc_read();

    adc_select_input(VSYS_ADC_GPIO - 26);    // Sample V_SYS (ADC3)
    v_sys_raw = adc_read();

    // Apply conversion factors 
    v_sys = v_sys_raw * vsys_conversion_factor;   
    v_hv = v_hv_raw * hvfb_conversion_factor;


    // Filter battery voltage - weighted average


    // Check charge status


    static uint8_t enter_state = 1;

    // PWM state machine
    switch(pwm_state)
    {
        case STANDBY:
        // STANDBY - default state, PWM disabled
        // Only start HVPS if battery SOC is high enough and fault flag is not set

        if (enter_state)
        {
            gpio_put(PSU_MODE_GPIO, 0); // Set LOW to use power-saving PFM mode
            enter_state = 0;
        }

        // Start HVPS when ready
        if (true && faults == 0)
        {
            pwm_state = RAMP_TO_TARGET;
            enter_state = 1;

            // Start output from 0 and ramp to target
            HVPS_setpoint = 0.0;
            target_setpoint = DEFAULT_HVPS_SP;  //// update with stored value from flash

            gpio_put(PSU_MODE_GPIO, 1); // Set HIGH to use low-noise PWM mode
        }
        break;


        case RAMP_TO_TARGET:
        // RAMP_TO_TARGET
        // PWM enabled
        // Increment or decrement current setpoint so it approaches target setpoint

        if (enter_state)
        {
            enter_state = 0;

            // Figure out whether to increment or decrement to reach target
            if (HVPS_setpoint < target_setpoint)
                ramp_rate = HVPS_RAMP_RATE;     // Positive ramp
            else
                ramp_rate = -1* HVPS_RAMP_RATE; // Negative ramp
        }        

        HVPS_setpoint += ramp_rate;

        // Change states once we're about to reach the target setpoint
        if (abs(HVPS_setpoint - target_setpoint) <= HVPS_RAMP_RATE)
        {
            pwm_state = PWM_REGULATING;
            enter_state = 1;

            HVPS_setpoint = target_setpoint;
        }

        // Return to standby if any faults occur
        if (faults)
        {
            pwm_state = STANDBY;
            enter_state = 1;
        }

        break;



        case PWM_REGULATING:
        // PWM_REGULATING
        // PWM enabled
        
        if (enter_state)
        {
                
            enter_state = 0;
        }        

        // Return to standby if any faults occur
        if (faults)
        {
            pwm_state = STANDBY;
            enter_state = 1;
        }

        break;        

    }


    // Feedback calculation
    pwm_duty = HVPS_p_gain * (HVPS_setpoint - v_hv);






    // Write the updated duty value to the PWM channel when it's enabled
    if (pwm_state != STANDBY)
    {
        // If the on-time would be too short, keep the PWM off instead
        if (pwm_duty < pwm_min_duty)
        {
            pwm_set_gpio_level(PWM_GPIO, 0);   
        }
        // If the duty cycle would be too high, limit it to the maximum allowed value
        else if (pwm_duty > pwm_max_duty)
        {
            pwm_set_gpio_level(PWM_GPIO, pwm_max_duty);   
        }
        else
        {
            pwm_set_gpio_level(PWM_GPIO, pwm_duty);   
        }

    }
    else
    {
        pwm_set_gpio_level(PWM_GPIO, 0);    // Disables PWM output
    }



    // Clear debug IO to mark end of control loop
    gpio_put(DEBUG_GPIO, 0);
}


void core1_entry()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    faults = 0; // Clear all fault bits by default

    // Set the LED pin as an output
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Set GPIO2 pin as an output for debugging
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);

    // GPIO24 is connected to VBUS - will be high when USB is connected

    adc_init();
    // Set up ADC inputs
    // V_SYS divider, RT = 200k, RB = 100k
    adc_gpio_init(VSYS_ADC_GPIO);  
    adc_select_input(VSYS_ADC_GPIO - 26);

    // HVFB divider, RT = 4.7M*2, RB = 47k 
    adc_gpio_init(HVPS_ADC_GPIO);


    // Configure PWM GPIO (without enabling it yet) and find out what slice is connected
    gpio_set_drive_strength(PWM_GPIO, GPIO_DRIVE_STRENGTH_12MA);
    uint slice_num = pwm_gpio_to_slice_num(PWM_GPIO); 

    // Configure PWM interrupt to be set for our slice
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);

    // Register and enable PWM interrupt handler
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    
    //Configure PWM period and limits
    uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    pwm_top = (f_clk_sys * 1000) / PWM_FREQ;
    pwm_min_duty = (PWM_MIN_PULSE * 1000.0) / ((1.0 / f_clk_sys) * 1000000);
    pwm_max_duty = (pwm_top * PWM_DUTY_MAX) / 100;
    ctrl_skip_cnt = PWM_FREQ / CTRL_LOOP_FREQ;

    pwm_set_wrap(slice_num, pwm_top);
    pwm_set_clkdiv_int_frac(slice_num, 1, 0);   // 1.0, no divider

    HVPS_setpoint = 0;    //// update with proper initial value later, read from flash

    // Set PWM output off initially and enable PWM
    pwm_set_output_polarity(slice_num, true, false);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm_min_duty);   //// set back to 0
    pwm_set_enabled(slice_num, true);
    gpio_set_function(PWM_GPIO, GPIO_FUNC_PWM);     // Now that PWM is configured, the output can be enabled safely


    while (true)
    {
        // Max voltage of input to voltage divider is VREF * (200+100k) / 100k
        const float conversion_factor = (3.3f * (300)/100) / (1 << 12);

        vsys = adc_read() * conversion_factor;

        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);      
    }
}