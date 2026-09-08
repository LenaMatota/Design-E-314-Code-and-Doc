/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ssd1306.h>
#include <ssd1306_fonts.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_TIME_MS 200
#define TEMP_BUFFER_SIZE 5
#define TRIG_PIN GPIO_PIN_3
#define TRIG_PORT GPIOC

//LED definitions
#define D2_LED_PIN GPIO_PIN_5
#define D2_LED_PORT GPIOB
#define D3_LED_PIN GPIO_PIN_4
#define D3_LED_PORT GPIOB
#define D4_LED_PIN GPIO_PIN_10
#define D4_LED_PORT GPIOA
#define D5_LED_PIN GPIO_PIN_4
#define D5_LED_PORT GPIOC

//Button definitions
#define S1_Button_pin GPIO_PIN_0
#define S1_BUTTON_PORT GPIOA
#define S2_Button_pin GPIO_PIN_8
#define S2_BUTTON_PORT GPIOB
#define S3_Button_pin GPIO_PIN_7
#define S3_BUTTON_PORT GPIOA
#define S4_Button_pin GPIO_PIN_6
#define S4_BUTTON_PORT GPIOB
#define S5_Button_pin GPIO_PIN_9
#define S5_BUTTON_PORT GPIOB

// Temperature sensor definitions
#define TEMP_WINDOW_MS              200U
#define TEMP_FILTER_SIZE            5U
#define TEMP_HIGH_ALARM_C           30.0f

#define TEMP_STARTUP_DISCARD_COUNT  1U
#define TEMP_MAX_STEP_C             3.0f
#define TEMP_MIN_VALID_C           -20.0f
#define TEMP_MAX_VALID_C            80.0f
#define TEMP_MIN_VALID_PULSES      400U
#define TEMP_MAX_VALID_PULSES     2500U
#define TEMP_STALE_TIMEOUT_MS     80U

// Proximity sensor definitions
#define ULTRA_TRIGGER_PERIOD_MS     100U
#define ULTRA_ALARM_CM              10.0f
#define ULTRA_CLEAR_CM              30.0f

//Keypad definitions
#define COLUMN_1_PIN GPIO_PIN_11
#define COLUMN_1_PORT GPIOA
#define COLUMN_2_PIN GPIO_PIN_2
#define COLUMN_2_PORT GPIOB
#define COLUMN_3_PIN GPIO_PIN_5
#define COLUMN_3_PORT GPIOC
#define ROW_1_PIN GPIO_PIN_12
#define ROW_1_PORT GPIOB
#define ROW_2_PIN GPIO_PIN_13
#define ROW_2_PORT GPIOB
#define ROW_3_PIN GPIO_PIN_6
#define ROW_3_PORT GPIOC
#define ROW_4_PIN GPIO_PIN_12
#define ROW_4_PORT GPIOA

//For the light sensor alarm condition
#define LIGHT_LOW_THRESHOLD   300.0f
#define LIGHT_HIGH_THRESHOLD  350.0f   // hysteresis

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// LED state tracking
uint8_t led_d3_state = 1;  // D3 starts ON (continuously)
uint8_t led_d5_state = 1;  // D5 starts ON (continuously)

// Flash timing
uint32_t lastFlashTime = 0;
uint8_t flashToggle = 0;

// Flag variables for buttons
volatile uint8_t flag_S1 = 0;
volatile uint8_t flag_S2 = 0;
volatile uint8_t flag_S3 = 0;
volatile uint8_t flag_S4 = 0;
volatile uint8_t flag_S5 = 0;

// Debounce trackers for each specific button
uint32_t LastTick_S1  = 0; // S1
uint32_t LastTick_S2  = 0; // S2
uint32_t LastTick_S3  = 0; // S3
uint32_t LastTick_S4  = 0; // S4
uint32_t LastTick_S5  = 0; // S5

/* ================= TEMPERATURE SENSOR ================= */
volatile uint32_t temp_pulse_count = 0;
volatile uint32_t temp_last_pulse_tick = 0;
volatile uint8_t  temp_raw_ready = 0;

float temp_raw_c = 0.0f;
float temp_calibrated_c = 0.0f;
float current_temp = 0.0f;

/* Two-point calibration: y = m*x + b
   Fill these from your own calibration measurements */
float temp_cal_m = 1.0000f;
float temp_cal_b = 0.0000f;

/* Robustness filter: keep 5 readings */
float temp_hist[TEMP_FILTER_SIZE] = {0};
uint8_t temp_hist_index = 0;
uint8_t temp_hist_full = 0;

/* ================= PROXIMITY SENSOR ================= */
float distance_cm = 0.0f;

// For the UART @Stat Command
uint8_t rx;
char uart_cmd[32];
uint8_t idx = 0;

// For alarm coding
volatile uint8_t proximity_alarm = 0;
volatile uint8_t temperature_alarm = 0;
volatile uint8_t alarm_check_enabled = 0;
uint32_t light_low_start_time = 0;
uint8_t light_alarm_state = 0;
volatile uint8_t light_warn_enabled = 1;
volatile uint8_t proximity_warn_enabled = 1;
volatile uint8_t temperature_warn_enabled = 1;
volatile uint8_t unsafe_driving = 0;
volatile uint8_t impact_detected = 0;
volatile uint8_t low_light_warning = 0;
volatile uint8_t warn_override[6] = {0};  // index 1–5
uint8_t data_logging_enabled = 0;
volatile uint8_t unsafe_alarm = 0;
volatile uint8_t impact_alarm = 0;
uint8_t temp_startup_discards = 0;
uint8_t temp_has_valid_reading = 0;
float temp_last_good_c = 0.0f;

//Menu navigation pages
typedef enum {
    PAGE_DEFAULT,

    PAGE_DISPLAY_1,
    PAGE_DISPLAY_2,
    PAGE_DISPLAY_3,
    PAGE_DISPLAY_4,
    PAGE_DISPLAY_5,

    PAGE_DATA_ENTRY_1,
    PAGE_DATA_ENTRY_2,
    PAGE_DATA_ENTRY_3,

    PAGE_DIAGNOSTIC_1,
    PAGE_DIAGNOSTIC_2,
    PAGE_DIAGNOSTIC_3,
    PAGE_DIAGNOSTIC_4,

    PAGE_WARNING
} SystemPage;

volatile SystemPage currentPage = PAGE_DEFAULT;

// ===== NEW MENU SYSTEM =====
uint8_t in_menu = 1;
uint8_t menu_state = 0;   // 0=Measurements, 1=Data Entry, 2=Diagnostics
uint8_t page = 0;
uint8_t subpage = 0;   // NEW: for deeper layer

uint8_t max_pages[] = {5, 3, 4};
// Measurements, Data Entry, Diagnostics
uint8_t in_warning = 0;
uint8_t warning_type = 0;

// Data entry
uint8_t editing = 0;
char input_buffer[10] = "";
float fuel_value = 0.0f;
float odometer_value = 0.0f;

// For Keypad debouncing
uint32_t keypad_last_time = 0;
char last_key = 0;

//For the alarm pages
uint8_t warning_acknowledged = 0;

//For the light sensor
volatile uint16_t light_adc = 0;
volatile float light_voltage = 0.0f;
volatile float light_lux = 0.0f;
uint16_t adc_raw = 0;

//For SetWarn implementation
volatile uint8_t forced_unsafe = 0;
volatile uint8_t forced_impact = 0;
volatile uint8_t forced_low_light = 0;
volatile uint8_t forced_proximity = 0;
volatile uint8_t forced_temp = 0;

/* ================= FUEL EFFICIENCY ================= */
float fuel_eff_kmpl = 0.0f;
float fuel_eff_l_per_100km = 0.0f;
volatile uint8_t sfd_received = 0;     // SFD has been received/executed

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void ProcessButtons(void);
float FilterMiddle3Of5(float new_value);
void UpdateTemperatureSensor(void);
void SetTemperatureCalibration(float raw_low, float actual_low, float raw_high, float actual_high);
float Get_Distance(void);
float ApplyLinearCalibration(float raw_value, float m, float b);
void UpdateLightSensor(void);
void UpdateLightAlarm(void);
void SlideFixed1Buffer(char *buf, char digit);
float Fixed1BufferToFloat(const char *buf);
void FormatSlidingValue(char *out, size_t out_size, const char *buf);
void RecalculateFuelEfficiency(void);
void HandleSFDCommand(const char *cmd);
void SendRFEResponse(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SendStudent_Number(void)
{
    HAL_Delay(350);
    char uart_buffer[20];
    const char studentNumber[] = "*24989142#\n";
    sprintf(uart_buffer, "%s", studentNumber);
    HAL_UART_Transmit(&huart2, (uint8_t*)studentNumber, strlen(studentNumber), 500);
}

void UpdateLEDs(void)
{
    uint32_t now = HAL_GetTick();

    if((now - lastFlashTime) >= 500U)
    {
        lastFlashTime = now;
        flashToggle ^= 1U;
    }
    /* Alarm sources for LEDs:
       - live measured alarms
       - OR SetWarn forced alarms
       SetWarn must not disable live alarm behaviour.
    */
    uint8_t d2_alarm = (proximity_alarm || forced_proximity);
    uint8_t d3_alarm = (unsafe_alarm || impact_alarm || forced_unsafe || forced_impact);
    uint8_t d4_alarm = (light_alarm_state || forced_low_light);
    uint8_t d5_alarm = (temperature_alarm || forced_temp);
    /* If you want LEDs OFF when no alarm, use RESET in the false case.
       That matches "alarm indicator" behaviour better than keeping them ON.
    */
    HAL_GPIO_WritePin(D2_LED_PORT, D2_LED_PIN,
        d2_alarm ? (flashToggle ? GPIO_PIN_SET : GPIO_PIN_RESET) : GPIO_PIN_SET);

    HAL_GPIO_WritePin(D3_LED_PORT, D3_LED_PIN,
        d3_alarm ? (flashToggle ? GPIO_PIN_SET : GPIO_PIN_RESET) : GPIO_PIN_SET);

    HAL_GPIO_WritePin(D4_LED_PORT, D4_LED_PIN,
        d4_alarm ? (flashToggle ? GPIO_PIN_SET : GPIO_PIN_RESET) : GPIO_PIN_SET);

    HAL_GPIO_WritePin(D5_LED_PORT, D5_LED_PIN,
        d5_alarm ? (flashToggle ? GPIO_PIN_SET : GPIO_PIN_RESET) : GPIO_PIN_SET);
}

void RecalculateFuelEfficiency(void)
{
    /* fuel_value      = liters refuelled
       odometer_value  = distance travelled since previous refuelling */
	fuel_eff_kmpl = odometer_value / fuel_value;
	fuel_eff_l_per_100km = (fuel_value / odometer_value) * 100.0f;
}

void HandleSFDCommand(const char *cmd)
{
    float liters = 0.0f;
    float km = 0.0f;
    if(sscanf(cmd, "@SFD %f;%f&", &liters, &km) == 2)
    {
        if(liters > 0.0f && km > 0.0f)
        {
            fuel_value = liters;
            odometer_value = km;
            sfd_received = 1;              // <-- missing line
            RecalculateFuelEfficiency();
        }
    }
}

void SendRFEResponse(void)
{
    char msg[128];
    /* Replace the timestamp later with your RTC values */
    snprintf(msg, sizeof(msg),
    		"@2026/02/26 22:12:42\n"
            "Fuel Eff: %04.1f km/L %04.1f L/100km\n"
            "&\n",
            fuel_eff_kmpl, fuel_eff_l_per_100km);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 500);
}

void SlideFixed1Buffer(char *buf, char digit)
{
    size_t len = strlen(buf);
    if(len < 4)
    {
        buf[len] = digit;
        buf[len + 1] = '\0';
    }
    else
    {
        buf[0] = buf[1];
        buf[1] = buf[2];
        buf[2] = buf[3];
        buf[3] = digit;
        buf[4] = '\0';
    }
}

float Fixed1BufferToFloat(const char *buf)
{
    int raw = 0;
    size_t len = strlen(buf);
    for(size_t i = 0; i < len; i++)
    {
        if(buf[i] >= '0' && buf[i] <= '9')
        {
            raw = (raw * 10) + (buf[i] - '0');
        }
    }
    return ((float)raw) / 10.0f;
}

void FormatSlidingValue(char *out, size_t out_size, const char *buf)
{
    int raw = 0;
    size_t len = strlen(buf);
    for(size_t i = 0; i < len; i++)
    {
        if(buf[i] >= '0' && buf[i] <= '9')
        {
            raw = (raw * 10) + (buf[i] - '0');
        }
    }
    snprintf(out, out_size, "%3d.%1d", raw / 10, raw % 10);
}

void ProcessButtons(void)
{
    // S1 = UP
    if(flag_S1)
    {
        flag_S1 = 0;
        if(in_warning)
            return;
        if(in_menu)   // top-level menu select
        {
            menu_state = (menu_state == 0) ? 2 : (menu_state - 1);
        }
        else if(!editing)   // inside selected menu, page navigation
        {
            if(page == 0)
                page = max_pages[menu_state] - 1;
            else
                page--;
        }
    }
    // S5 = DOWN
    if(flag_S5)
    {
        flag_S5 = 0;
        if(in_warning)
            return;
        if(in_menu)
        {
            menu_state = (menu_state + 1) % 3;
        }
        else if(!editing)
        {
            page++;
            if(page >= max_pages[menu_state])
                page = 0;
        }
    }
    // S4 = ENTER selected top-level menu
    if(flag_S4)
    {
        flag_S4 = 0;
        if(in_warning)
            return;
        if(in_menu)
        {
            in_menu = 0;
            page = 0;
        }
    }
    // S2 = EXIT / BACK
    if(flag_S2)
    {
        flag_S2 = 0;
        if(in_warning)
            return;   // S2 does nothing on warning page now
        if(editing)
        {
            editing = 0;
            memset(input_buffer, 0, sizeof(input_buffer));
        }
        else if(!in_menu)
        {
            in_menu = 1;
            page = 0;
        }
    }
    // S3 = DATA ENTRY toggle on pages 0 and 1 only
    if(flag_S3)
    {
        flag_S3 = 0;
        if(in_warning)
        {
            in_warning = 0;
            warning_acknowledged = 1;
            warning_type = 0;
            return;
        }
        if(!in_menu && menu_state == 1 && (page == 0 || page == 1))
        {
            if(!editing)
            {
                editing = 1;
                memset(input_buffer, 0, sizeof(input_buffer));
            }
            else
            {
                if(strlen(input_buffer) > 0)
                {
                    float val = Fixed1BufferToFloat(input_buffer);
                    if(page == 0)
                        fuel_value = val;
                    else
                        odometer_value = val;
                    RecalculateFuelEfficiency();
                }
                editing = 0;
                memset(input_buffer, 0, sizeof(input_buffer));
            }
        }
    }
}

float ApplyLinearCalibration(float raw_value, float m, float b)
{
    return (m * raw_value) + b;
}

void UpdateTemperatureSensor(void)
{
    static uint32_t last_window_tick = 0;
    uint32_t now = HAL_GetTick();

    if((now - last_window_tick) < TEMP_WINDOW_MS)
    {
        return;
    }
    last_window_tick = now;

    uint32_t pulses;
    uint32_t last_pulse_tick;

    __disable_irq();
    pulses = temp_pulse_count;
    temp_pulse_count = 0;
    last_pulse_tick = temp_last_pulse_tick;
    __enable_irq();

    /* Ignore stale or partial captures */
    if((now - last_pulse_tick) > TEMP_STALE_TIMEOUT_MS)
    {
        return;
    }

    /* Reject clearly bad pulse counts */
    if((pulses < TEMP_MIN_VALID_PULSES) || (pulses > TEMP_MAX_VALID_PULSES))
    {
        return;
    }

    temp_raw_c = (((float)pulses / 4096.0f) * 256.0f) - 50.0f;
    temp_calibrated_c = ApplyLinearCalibration(temp_raw_c, temp_cal_m, temp_cal_b);

    /* Reject impossible calibrated temperatures */
    if((temp_calibrated_c < TEMP_MIN_VALID_C) || (temp_calibrated_c > TEMP_MAX_VALID_C))
    {
        return;
    }

    /* Discard first few readings after startup per LMT01 guidance */
    if(temp_startup_discards < TEMP_STARTUP_DISCARD_COUNT)
    {
        temp_startup_discards++;
        return;
    }

    /* Reject single-sample spikes */
    if(temp_has_valid_reading)
    {
        float step = temp_calibrated_c - temp_last_good_c;
        if(step < 0.0f)
            step = -step;

        if(step > TEMP_MAX_STEP_C)
        {
            return;
        }
    }

    current_temp = FilterMiddle3Of5(temp_calibrated_c);
    temp_last_good_c = current_temp;
    temp_has_valid_reading = 1U;
    temp_raw_ready = 1U;

    if(alarm_check_enabled && temperature_warn_enabled)
    {
        temperature_alarm = (current_temp > TEMP_HIGH_ALARM_C) ? 1U : 0U;
    }
    else
    {
        temperature_alarm = 0U;
    }
}

void SetTemperatureCalibration(float raw_low, float actual_low, float raw_high, float actual_high)
{
    float denom = raw_high - raw_low;
    if(denom > -0.0001f && denom < 0.0001f)
    {
        temp_cal_m = 1.0f;
        temp_cal_b = 0.0f;
        return;
    }
    temp_cal_m = (actual_high - actual_low) / denom;
    temp_cal_b = actual_low - (temp_cal_m * raw_low);
}

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while((DWT->CYCCNT - start) < cycles)
    {
        /* wait */
    }
}

float Get_Distance(void)
{
    uint32_t ticks = 0;
    uint32_t timeout;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    timeout = 30000U;
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
    {
        if(--timeout == 0U)
            return -1.0f;
        delay_us(1);
    }
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    timeout = 30000U;
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
    {
        if(--timeout == 0U)
            return -2.0f;
        delay_us(1);
    }
    ticks = __HAL_TIM_GET_COUNTER(&htim2);
    return ((float)ticks * 0.0343f) / 2.0f;
}

void OLED_Update(void)
{
    ssd1306_Fill(Black);
    char buf[25];
    // ================= WARNING MODE =================
    if(in_warning)
    {
        ssd1306_SetCursor(0,1);
        ssd1306_WriteString("==== WARNING ====", Font_7x10, White);
        switch(warning_type)
        {
        	case 1: // Unsafe driving
        		ssd1306_SetCursor(0,11);
        		ssd1306_WriteString("--Unsafe Driving--", Font_7x10, White);
        		snprintf(buf, sizeof(buf), "Accel: |%1.2f| g", 0.00f);
        		ssd1306_SetCursor(0,22);
        		ssd1306_WriteString(buf, Font_7x10, White);
        	break;

        	case 2: // Impact detected
        		ssd1306_SetCursor(0,11);
        		ssd1306_WriteString("-Impact Detected--", Font_7x10, White);
        		snprintf(buf, sizeof(buf), "Accel: |%1.2f| g", 0.00f);
        		ssd1306_SetCursor(0,22);
        		ssd1306_WriteString(buf, Font_7x10, White);
        	break;

        	case 3: // Low lighting
        		ssd1306_SetCursor(0,11);
        		ssd1306_WriteString("----Low Light----", Font_7x10, White);
        		snprintf(buf, sizeof(buf), "Light: %4.0f lux", light_lux);
        		ssd1306_SetCursor(0,22);
        		ssd1306_WriteString(buf, Font_7x10, White);
        	break;

        	case 4: // Proximity
                ssd1306_SetCursor(0,11);
                ssd1306_WriteString("---Proximity---", Font_7x10, White);
                snprintf(buf, sizeof(buf), "Dist: %4.1f cm", distance_cm);
                ssd1306_SetCursor(0,22);
                ssd1306_WriteString(buf, Font_7x10, White);
            break;

            case 5: // Temperature
                ssd1306_SetCursor(0,11);
                ssd1306_WriteString("-High Temperature-", Font_7x10, White);
                snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);
                ssd1306_SetCursor(0,22);
                ssd1306_WriteString(buf, Font_7x10, White);
            break;
        }
        ssd1306_UpdateScreen();
        return;
    }
    // ================= MENU =================
    if(in_menu)
    {
        ssd1306_SetCursor(0,1);
        ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
        ssd1306_SetCursor(0,11);
        switch(menu_state)
        {
            case 0: ssd1306_WriteString("== Measurements ==", Font_7x10, White); break;
            case 1: ssd1306_WriteString("=== Data Entry ===", Font_7x10, White); break;
            case 2: ssd1306_WriteString("== Diagnostics ==", Font_7x10, White); break;
        }
        ssd1306_SetCursor(0,22);
        ssd1306_WriteString("Press -> display", Font_7x10, White);

        ssd1306_UpdateScreen();
        return;
    }
    else
    {
    	// ================= PAGE LEVEL =================
    	switch(menu_state)
    	{
    	case 0: // Measurements
    	    switch(page)
    	    {
    	        case 0:
    	            ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
    	            snprintf(buf, sizeof(buf), "Dist: %4.1f cm", distance_cm);      // xx.x
    	            ssd1306_SetCursor(0,11);
    	            ssd1306_WriteString(buf, Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Temp: %4.1f C", current_temp);      // xx.x
    	            ssd1306_SetCursor(0,22);
    	            ssd1306_WriteString(buf, Font_7x10, White);
    	            break;

    	        case 1:
    	            ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Accel: |%1.2f| g", 0.00f);        // x.xx
    	            ssd1306_SetCursor(0,11);
    	            ssd1306_WriteString(buf, Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Light: %4.0f lux", light_lux);      // xxxx
    	            ssd1306_SetCursor(0,22);
    	            ssd1306_WriteString(buf, Font_7x10, White);
    	            break;

    	        case 2:
    	            ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Lat: %9.6f", 0.000000f);            // xx.xxxxxx
    	            ssd1306_SetCursor(0,11);
    	            ssd1306_WriteString(buf, Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Long: %10.6f", 0.000000f);          // xxx.xxxxxx
    	            ssd1306_SetCursor(0,22);
    	            ssd1306_WriteString(buf, Font_7x10, White);
    	            break;

    	        case 3:
    	            ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Heading: %3.0f deg", 0.0f);        // xxx
    	            ssd1306_SetCursor(0,11);
    	            ssd1306_WriteString(buf, Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "Speed: %5.1f km/h", 0.0f);          // xxx.x
    	            ssd1306_SetCursor(0,22);
    	            ssd1306_WriteString(buf, Font_7x10, White);
    	            break;

    	        case 4:
    	            ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("Fuel Efficiency:", Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "%4.1f km/L", fuel_eff_kmpl);
    	            ssd1306_SetCursor(0,11);
    	            ssd1306_WriteString(buf, Font_7x10, White);

    	            snprintf(buf, sizeof(buf), "%4.1f L/100 km", fuel_eff_l_per_100km);
    	            ssd1306_SetCursor(0,22);
    	            ssd1306_WriteString(buf, Font_7x10, White);
    	            break;
    	    }
    	break;

    	case 1: // Data Entry
    		switch(page)
    		{
    			case 0: // Fuel
    				ssd1306_SetCursor(0,1);
    	            ssd1306_WriteString("Enter fuel liters", Font_7x10, White);
    	            if(editing)
    	            {
    	            	char value_buf[10];
    	            	FormatSlidingValue(value_buf, sizeof(value_buf), input_buffer);
    	            	snprintf(buf, sizeof(buf), "Current: %s L", value_buf);
    	            	ssd1306_SetCursor(0,11);
    	            	ssd1306_WriteString(buf, Font_7x10, White);
    	                ssd1306_SetCursor(0,22);
    	                ssd1306_WriteString("Press S3 to accept", Font_7x10, White);
    	            }
    	            else
    	            {
    	            	snprintf(buf, sizeof(buf), "Current: %.1f L", fuel_value);   // xxx.x
    	            	ssd1306_SetCursor(0,11);
    	            	ssd1306_WriteString(buf, Font_7x10, White);
    	            	ssd1306_SetCursor(0,22);
    	                ssd1306_WriteString("Press S3 to change", Font_7x10, White);
    	            }
    	         break;

    	         case 1: // Odometer
    	        	 ssd1306_SetCursor(0,1);
    	             ssd1306_WriteString("Enter odometer km", Font_7x10, White);
    	             if(editing)
    	             {
    	            	 char value_buf[10];
    	            	 FormatSlidingValue(value_buf, sizeof(value_buf), input_buffer);
    	            	 snprintf(buf, sizeof(buf), "Current: %s km", value_buf);
    	            	 ssd1306_SetCursor(0,11);
    	            	 ssd1306_WriteString(buf, Font_7x10, White);
    	                 ssd1306_SetCursor(0,22);
    	                 ssd1306_WriteString("Press S3 to accept", Font_7x10, White);
    	             }
    	             else
    	             {
    	            	 snprintf(buf, sizeof(buf), "Current: %.1f km", odometer_value); // xxx.x
    	            	 ssd1306_SetCursor(0,11);
    	            	 ssd1306_WriteString(buf, Font_7x10, White);
    	            	 ssd1306_SetCursor(0,22);
    	                 ssd1306_WriteString("Press S3 to change", Font_7x10, White);
    	             }
    	         break;

    	         case 2: // Data Logging (RESTORED)
    	             ssd1306_SetCursor(0,1);
    	             ssd1306_WriteString("Log data", Font_7x10, White);
    	             ssd1306_SetCursor(0,11);
    	             ssd1306_WriteString("'*' = Y / '#' = N", Font_7x10, White);
    	             if(data_logging_enabled)
    	             {
    	            	 ssd1306_SetCursor(0,22);
    	                 ssd1306_WriteString("Log Data: ENABLED", Font_7x10, White);
    	             }
    	             else
    	             {
    	            	 ssd1306_SetCursor(0,22);
    	            	 ssd1306_WriteString("Log Data: DISABLED", Font_7x10, White);
    	             }

    	         break;
    		}
    	break;

        case 2: // Diagnostics
        	switch(page)
        	{
        		case 0: // SD Card
        			ssd1306_SetCursor(0,0);
                    ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
                    ssd1306_SetCursor(0,10);
                    ssd1306_WriteString("-- Diagnostics ---", Font_7x10, White);
                    ssd1306_SetCursor(0,20);
                    ssd1306_WriteString("SD-card:", Font_7x10, White);
                    ssd1306_SetCursor(80,20);
                    ssd1306_WriteString("NOT OK", Font_7x10, White); // hard-coded
                break;

            	case 1: // MPU-6050
            		ssd1306_SetCursor(0,0);
            		ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
            		ssd1306_SetCursor(0,10);
            		ssd1306_WriteString("-- Diagnostics ---", Font_7x10, White);
            		ssd1306_SetCursor(0,20);
                    ssd1306_WriteString("MPU-6050:", Font_7x10, White);
                    ssd1306_SetCursor(80,20);
                    ssd1306_WriteString("NOT OK", Font_7x10, White);
                break;

            	case 2: // GPS
            		ssd1306_SetCursor(0,0);
            		ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
            		ssd1306_SetCursor(0,10);
                    ssd1306_WriteString("-- Diagnostics ---", Font_7x10, White);
                    ssd1306_SetCursor(0,20);
                    ssd1306_WriteString("GPS:", Font_7x10, White);
                    ssd1306_SetCursor(80,20);
                    ssd1306_WriteString("NOT OK", Font_7x10, White);
                 break;

                 case 3: // Data Logging
                	 ssd1306_SetCursor(0,0);
                     ssd1306_WriteString("=2026/02/26 22:12=", Font_7x10, White);
                     ssd1306_SetCursor(0,10);
                     ssd1306_WriteString("-- Diagnostics ---", Font_7x10, White);
                     ssd1306_SetCursor(0,20);
                     ssd1306_WriteString("Log Data:", Font_7x10, White);
                     ssd1306_SetCursor(70,20);
                     if(data_logging_enabled)
                    	 ssd1306_WriteString("ENABLED", Font_7x10, White);
                     else
                         ssd1306_WriteString("DISABLED", Font_7x10, White);
                 break;
        	}
        break;
    	}
    }
    ssd1306_UpdateScreen();
}

char Keypad_Scan(void)
{
    const char keys[4][3] = {
        {'1','2','3'},
        {'4','5','6'},
        {'7','8','9'},
        {'*','0','#'}
    };

    GPIO_TypeDef* rowPorts[4] = {ROW_1_PORT, ROW_2_PORT, ROW_3_PORT, ROW_4_PORT};
    uint16_t rowPins[4] = {ROW_1_PIN, ROW_2_PIN, ROW_3_PIN, ROW_4_PIN};
    GPIO_TypeDef* colPorts[3] = {COLUMN_1_PORT, COLUMN_2_PORT, COLUMN_3_PORT};
    uint16_t colPins[3] = {COLUMN_1_PIN, COLUMN_2_PIN, COLUMN_3_PIN};

    for(int i = 0; i < 4; i++)
    {
        // Set all rows LOW first
        for(int r = 0; r < 4; r++)
            HAL_GPIO_WritePin(rowPorts[r], rowPins[r], GPIO_PIN_RESET);
        // Drive only this row HIGH
        HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
        // Small settle time
        for(volatile int d = 0; d < 200; d++);
        for(int j = 0; j < 3; j++)
        {
            if(HAL_GPIO_ReadPin(colPorts[j], colPins[j]) == GPIO_PIN_SET)
            {
                return keys[i][j];
            }
        }
    }
    return 0;
}

void ProcessKeypad(void)
{
    char key = Keypad_Scan();
    if(key == 0)
    {
        last_key = 0;
        return;
    }
    if(key == last_key)
        return;
    if(HAL_GetTick() - keypad_last_time < 200)
        return;
    keypad_last_time = HAL_GetTick();
    last_key = key;
    // Editing fuel/odometer
    if(editing && menu_state == 1 && (page == 0 || page == 1))
    {
        if(key >= '0' && key <= '9')
        {
            SlideFixed1Buffer(input_buffer, key);
        }
        return;
    }
    // Data logging page: * = enable, # = disable
    if(!editing && !in_menu && menu_state == 1 && page == 2)
    {
        if(key == '*')
        {
            data_logging_enabled = 1;
        }
        else if(key == '#')
        {
            data_logging_enabled = 0;
        }
        return;
    }
}

void UpdateLightSensor(void)
{
    uint16_t adc_raw = 0;

    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adc_raw = HAL_ADC_GetValue(&hadc1);
        // Step 1: Convert ADC to measured voltage (0–3.3V)
        float V_measured = (adc_raw / 4095.0f) * 3.3f;
        // Step 2: Apply calibration to reconstruct sensor voltage
        float V_in = (25.0f / 11.0f) * V_measured + 2.43636364f;
        // Step 3: Convert to lux
        light_lux = ((940.0f * V_in) - 2202.0f)/4.0f;
        // Optional: keep voltage for debugging
        light_voltage = V_in;
    }

    HAL_ADC_Stop(&hadc1);
}

void UpdateLightAlarm(void)
{
    uint32_t now = HAL_GetTick();
    if(!alarm_check_enabled || !light_warn_enabled)
    {
        light_alarm_state = 0;
        light_low_start_time = 0;
        return;
    }
    if(light_lux < LIGHT_LOW_THRESHOLD)
    {
        if(light_low_start_time == 0U)
        {
            light_low_start_time = now;
        }

        if((now - light_low_start_time) > 1000U)
        {
            light_alarm_state = 1U;
        }
    }
    else if(light_lux > LIGHT_HIGH_THRESHOLD)
    {
        light_alarm_state = 0U;
        light_low_start_time = 0U;
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//To do list
	//Install the OLED drivers and get it working
	//Build the state machine and get the keypad working
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SendStudent_Number();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);    // D2 ON
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  GPIO_PIN_SET);   // D3 starts ON
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,  GPIO_PIN_SET);  // D4 ON
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,  GPIO_PIN_SET);   // D5 starts ON

  flashToggle = 1;
  lastFlashTime = HAL_GetTick();

  // Initialising Timers
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim2);
  DWT_Delay_Init();

  // The stat command
  HAL_UART_Receive_IT(&huart2,&rx,1);

  // Initialising alarm conditions
  alarm_check_enabled = 1;
  proximity_alarm = 0;
  temperature_alarm = 0;
  light_alarm_state = 0;

  //Initialising OLED
  ssd1306_Init();

  temp_pulse_count = 0;
  temp_last_pulse_tick = HAL_GetTick();
  temp_startup_discards = 0;
  temp_has_valid_reading = 0;
  temp_hist_index = 0;
  temp_hist_full = 0;
  for(uint8_t i = 0; i < TEMP_FILTER_SIZE; i++)
  {
      temp_hist[i] = 0.0f;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t last_ultra_sample = 0;
	      uint32_t now = HAL_GetTick();

	      UpdateLEDs();
	      ProcessButtons();
	      ProcessKeypad();

	      if((now - last_ultra_sample) >= ULTRA_TRIGGER_PERIOD_MS)
	      {
	          last_ultra_sample = now;
	          float d = Get_Distance();
	          if(d > 0.0f)
	          {
	              distance_cm = d;
	          }
	          else
	          {
	              distance_cm = 0.0f;
	          }

	          if(alarm_check_enabled && proximity_warn_enabled)
	          {
	              if(distance_cm > 0.0f)
	              {
	                  if(distance_cm <= ULTRA_ALARM_CM)
	                      proximity_alarm = 1U;
	                  else if(distance_cm >= ULTRA_CLEAR_CM)
	                      proximity_alarm = 0U;
	              }
	          }
	          else
	          {
	              proximity_alarm = 0U;
	          }
	      }
	      UpdateTemperatureSensor();
	      UpdateLightSensor();
	      UpdateLightAlarm();
	      OLED_Update();
	      HAL_Delay(20);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 50000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|D5_LED_Pin|Row_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|D4_LED_Pin|Row_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Row_1_Pin|Row_2_Pin|D3_LED_Pin|D2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 D5_LED_Pin Row_3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|D5_LED_Pin|Row_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Button_Pin S3_Button_Pin */
  GPIO_InitStruct.Pin = S1_Button_Pin|S3_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin D4_LED_Pin Row_4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|D4_LED_Pin|Row_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Column_3_Pin */
  GPIO_InitStruct.Pin = Column_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Column_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Column_2_Pin */
  GPIO_InitStruct.Pin = Column_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Column_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Row_1_Pin Row_2_Pin D3_LED_Pin D2_LED_Pin */
  GPIO_InitStruct.Pin = Row_1_Pin|Row_2_Pin|D3_LED_Pin|D2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Column_1_Pin */
  GPIO_InitStruct.Pin = Column_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Column_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S4_Button_Pin S2_Button_Pin S5_Button_Pin */
  GPIO_InitStruct.Pin = S4_Button_Pin|S2_Button_Pin|S5_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if(GPIO_Pin == S1_Button_pin)
    {
        if(now - LastTick_S1 > BUTTON_DEBOUNCE_TIME_MS)
        {
            LastTick_S1 = now;
            flag_S1 = 1;
        }
    }
    else if(GPIO_Pin == S2_Button_pin)
    {
        if(now - LastTick_S2 > BUTTON_DEBOUNCE_TIME_MS)
        {
            LastTick_S2 = now;
            flag_S2 = 1;
        }
    }
    else if(GPIO_Pin == S3_Button_pin)
    {
        if(now - LastTick_S3 > BUTTON_DEBOUNCE_TIME_MS)
        {
            LastTick_S3 = now;
            flag_S3 = 1;
        }
    }
    else if(GPIO_Pin == S4_Button_pin)
    {
        if(now - LastTick_S4 > BUTTON_DEBOUNCE_TIME_MS)
        {
            LastTick_S4 = now;
            flag_S4 = 1;
        }
    }
    else if(GPIO_Pin == S5_Button_pin)
    {
        if(now - LastTick_S5 > BUTTON_DEBOUNCE_TIME_MS)
        {
            LastTick_S5 = now;
            flag_S5 = 1;
        }
    }
}

float FilterMiddle3Of5(float new_value)
{
    temp_hist[temp_hist_index] = new_value;
    temp_hist_index = (temp_hist_index + 1U) % TEMP_FILTER_SIZE;

    if(temp_hist_full < TEMP_FILTER_SIZE)
    {
        temp_hist_full++;
    }

    /* Until buffer is full, return simple average of available values */
    if(temp_hist_full < TEMP_FILTER_SIZE)
    {
        float sum = 0.0f;
        for(uint8_t i = 0; i < temp_hist_full; i++)
        {
            sum += temp_hist[i];
        }
        return sum / (float)temp_hist_full;
    }

    /* Copy and sort 5 samples */
    float s[TEMP_FILTER_SIZE];
    for(uint8_t i = 0; i < TEMP_FILTER_SIZE; i++)
    {
        s[i] = temp_hist[i];
    }

    for(uint8_t i = 0; i < TEMP_FILTER_SIZE - 1U; i++)
    {
        for(uint8_t j = i + 1U; j < TEMP_FILTER_SIZE; j++)
        {
            if(s[j] < s[i])
            {
                float t = s[i];
                s[i] = s[j];
                s[j] = t;
            }
        }
    }

    /* Drop lowest and highest, average middle 3 */
    return (s[1] + s[2] + s[3]) / 3.0f;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* ================= TEMPERATURE SENSOR ISR ================= */
	if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
	    uint32_t now_ms = HAL_GetTick();
	    if((now_ms - temp_last_pulse_tick) > 4U)
	    {
	        temp_pulse_count = 0;
	    }
	    temp_pulse_count++;
	    temp_last_pulse_tick = now_ms;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
    	if(idx < sizeof(uart_cmd)-1)
    	{
    	    uart_cmd[idx++] = rx;
    	}
    	if(rx == '\n')
        {
            uart_cmd[idx] = '\0';
            if(strstr(uart_cmd, "@Stat&") != NULL)
            {
                char msg[32];

                snprintf(msg, sizeof(msg), "@2026/02/26 22:12:42\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Distance:    %04.1f cm\n", distance_cm);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Temperature: %+05.1f C\n", current_temp);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Light:      %04d lux\n", (int)light_lux);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "X accel:     %+05.2f g\n", 0.0f);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Y accel:     %+05.2f g\n", 0.0f);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Z accel:     %+05.2f g\n", 0.0f);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Unsafe driving:    %d\n",
                         (forced_unsafe || unsafe_alarm) ? 1 : 0);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Impact detected:   %d\n",
                         (forced_impact || impact_alarm) ? 1 : 0);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Low-Light warning: %d\n",
                         (forced_low_light || light_alarm_state) ? 1 : 0);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "Proximity warning: %d\n",
                         (forced_proximity || proximity_alarm) ? 1 : 0);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "High Temperature:  %d\n",
                         (forced_temp || temperature_alarm) ? 1 : 0);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "GPSLat:   %+010.6f\n", 0.0f);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 21, 100);

                snprintf(msg, sizeof(msg), "GPSLong: %+011.6f\n&\n", 0.0f);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, 23, 100);
            }
            if(strncmp(uart_cmd, "@SFD ", 5) == 0)
            {
                HandleSFDCommand(uart_cmd);
            }
            else if(strcmp(uart_cmd, "@RFE&\n") == 0)
            {
                SendRFEResponse();
            }
            idx = 0;
        }
        HAL_UART_Receive_IT(&huart2,&rx,1);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
