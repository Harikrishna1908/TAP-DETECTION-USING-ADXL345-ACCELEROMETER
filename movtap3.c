#include <stdio.h>   // For printf()
#include <math.h>    // For sqrt(), pow(), acos()
#include <stdint.h>  // For fixed-width integer types (int16_t)
#include "driver/i2c.h"
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#define TAG "ADXL345_APP"
// Define multiplier for converting raw accelerometer data to G-units
#define ADXL345_MG2G_MULTIPLIER  0.004  // 1 LSB = 4 mg

// I2C Configuration
#define I2C_MASTER_SCL_IO    18   
#define I2C_MASTER_SDA_IO    5    
#define I2C_MASTER_FREQ_HZ   400000  
#define I2C_MASTER_NUM       I2C_NUM_0  
#define I2C_TX_BUF_DISABLE   0 
#define I2C_RX_BUF_DISABLE   0 

// ADXL345 I2C Registers
#define ADXL345_ADDR         0x53  
#define ADXL345_INT_SOURCE   0x30  
#define PWR_CTL_REG          0x2D
#define INT_ENABLE           0x2E 
#define INT_MAP              0x2F 
#define THRESH_TAP           0x1D
#define DUR                  0x21
#define LATENT               0x22
#define WINDOW               0x23
#define TAP_AXES             0x2A
#define DATA_FORMAT          0x31
#define ACT_TAP_STATUS       0x2B
#define THRESH_ACT           0x24
#define THRESH_INACT         0x25
#define TIME_INACT           0x26
#define ACT_INACT_CTL        0x27
#define DATAX0               0x32  // X-axis LSB
#define DATAX1               0x33  // X-axis MSB
#define DATAY0               0x34  // Y-axis LSB
#define DATAY1               0x35  // Y-axis MSB
#define DATAZ0               0x36  // Z-axis LSB
#define DATAZ1               0x37  // Z-axis MSB

// Gravity Constant (Used for Reference)
#define GRAVITY_CONSTANT 9.81  

// Degree Conversion Factor (1 radian ˜ 57.2958 degrees)
#define RAD_TO_DEG 57.2958  

// GPIO Configuration for Interrupts
#define INT1_GPIO         14  // Activity, Single Tap, Double Tap
#define INT2_GPIO         26  // Inactivity

// Queue and Timer Handles
QueueHandle_t int1_event_queue, int2_event_queue;

float tilt_angle,filter_angle;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
    ESP_LOGI(TAG, "I2C MASTER INITIALIZED");
}

esp_err_t i2c_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_read_register(uint8_t i2c_reg, uint8_t *data_rd) {
if (data_rd == NULL) {
        return ESP_ERR_INVALID_ARG; // Ensure valid pointer
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM; // Ensure command was created
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, i2c_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data_rd, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
float calculate_tilt_angle() {
    int16_t x, y, z;
    uint8_t lsb, msb;
    
    i2c_read_register(DATAX0, &lsb);
    i2c_read_register(DATAX1, &msb);
    x = ((msb << 8) | lsb);
    
    i2c_read_register(DATAY0, &lsb);
    i2c_read_register(DATAY1, &msb);
    y = ((msb << 8) | lsb);
    
    i2c_read_register(DATAZ0, &lsb);
    i2c_read_register(DATAZ1, &msb);
    z = ((msb << 8) | lsb);
    
    float xf = x * ADXL345_MG2G_MULTIPLIER;
    float yf = y * ADXL345_MG2G_MULTIPLIER;
    float zf = z * ADXL345_MG2G_MULTIPLIER;
    
    float magnitude = sqrt(xf * xf + yf * yf + zf * zf);
     tilt_angle = acos(zf / magnitude) * RAD_TO_DEG;
    
    return tilt_angle;
}
int get_first_digit(int num) {
    while (num >= 10) {
        num /= 10;
    }
    return num;
}

float calculateAverage(float arr[], int size) {
    if (size == 0) {
        return 0; // Return 0 if the array is empty.
    }

    int sum = 0;

    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size; // Integer division truncates the decimal part.
}
float filtered_angle(){
 float angle[15];
 for(int i = 0; i< 10;i++){
 angle[i]= (float)calculate_tilt_angle();
 }

 int size = sizeof(angle) / sizeof(angle[0]);
 int first_digits[10]={0};
 // count occurence of first digits
 for(int i = 0; i< size; i++){
 int first_digit = get_first_digit(angle[i]);
 first_digits[first_digit]++;
 }
 // Find most common first digit
int max_count = 0, max_digit = 0;
for (int i = 0; i < 10; i++) {
    if (first_digits[i] > max_count) {
        max_count = first_digits[i];
        max_digit = i;
    }
   }
float final_ang_val[7];
int j =0;
for ( int i=0; i < size; i++) {
    if (get_first_digit(angle[i]) == max_digit) {
        final_ang_val[j] = angle[i];
        j++;
    }
filter_angle = calculateAverage(final_ang_val, j);
}  
 return filter_angle;
}
void adxl345_init() {
    struct {
        uint8_t reg;
        uint8_t value;
    } adxl_config[] = {
        { PWR_CTL_REG, 0x28 },
        { THRESH_TAP, 0x20 },
        { DUR, 0x10 },
        { LATENT, 0x30 },
        { WINDOW, 0x60 },
        { TAP_AXES, 0x01 },
        { INT_ENABLE, 0x78 },
        { INT_MAP, 0x10 },
        { DATA_FORMAT, 0x0B },
        { THRESH_ACT, 0x0E },
        { THRESH_INACT, 0x02 },
        { TIME_INACT, 0x02 },
        { ACT_INACT_CTL, 0x66 }
    };

    for (int i = 0; i < sizeof(adxl_config) / sizeof(adxl_config[0]); i++) {
        i2c_write_register(adxl_config[i].reg, adxl_config[i].value);
    }
}
void detect_int1_event() {
    uint8_t int_status;
    i2c_read_register(ADXL345_INT_SOURCE, &int_status);
    printf("INT1 Event Source: 0x%02X\n", int_status);
    if ((int_status & (1 << 5))) {
    if(filter_angle > (0) && filter_angle <(20))
        printf("Double Tap Detected!\n");
    } else if((int_status & (1 << 3))) {
       if(filter_angle > (0) && filter_angle <(15)){
       printf("tilt angle:%.2f\n",tilt_angle);
      printf("filter angle:%.2f\n",filter_angle);
        printf("Inactivity Detected!\n");}
    }
}

void detect_int2_event() {
    uint8_t int_status;
    i2c_read_register(ADXL345_INT_SOURCE, &int_status);
    printf("INT2 Event Source: 0x%02X\n", int_status);
    if ((int_status & (1 << 4))&&(int_status & (1<<5))) {
    if((filter_angle > (0) && filter_angle <(30))){
      printf("Tilt angle:%.2f\n",filter_angle);
        printf("Double Tap Detected!\n");}
    } else if(int_status & (1<<4)){
     if((filter_angle > (35) && filter_angle <(160))){
       printf("tilt angle:%.2f\n",tilt_angle);
      printf("filter angle:%.2f\n",filter_angle);
      printf("Activity Detected!\n");
    }
}
}

void IRAM_ATTR int1_isr_handler(void *arg) {
    uint8_t event = 1;
    xQueueSendFromISR(int1_event_queue, &event, NULL);
}

void IRAM_ATTR int2_isr_handler(void *arg) {
    uint8_t event = 1;
    xQueueSendFromISR(int2_event_queue, &event, NULL);
}

void gpio_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << INT1_GPIO) | (1ULL << INT2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT1_GPIO, int1_isr_handler, NULL);
    gpio_isr_handler_add(INT2_GPIO, int2_isr_handler, NULL);
}

void int1_event_task(void *pvParameters) {
    uint8_t event;
    while (1) {
        xQueueReceive(int1_event_queue, &event, portMAX_DELAY);
        calculate_tilt_angle();
        filtered_angle();
        detect_int1_event();
    }
}

void int2_event_task(void *pvParameters) {
    uint8_t event;
    while (1) {
        xQueueReceive(int2_event_queue, &event, portMAX_DELAY);
        calculate_tilt_angle();
        filtered_angle();
        detect_int2_event();
    }
}

void app_main() {
    i2c_master_init();
    gpio_init();
    adxl345_init();
    int1_event_queue = xQueueCreate(10, sizeof(uint8_t));
    int2_event_queue = xQueueCreate(10, sizeof(uint8_t));
    xTaskCreate(int1_event_task, "INT1_Event_Task", 4096, NULL, 5, NULL);
    xTaskCreate(int2_event_task, "INT2_Event_Task", 4096, NULL, 5, NULL);
}
