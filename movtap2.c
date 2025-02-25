#include <stdio.h>
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

#define TAG "ADXL345_APP"

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
// GPIO Configuration for Interrupt
#define INT1_GPIO         14 
#define INT2_GPIO         26  

// Queue and Timer Handles
QueueHandle_t tap_event_queue, mov_event_queue;
TimerHandle_t tap_timer;
bool first_tap_detected = false;

void tap_timer_callback(TimerHandle_t xTimer) {
    first_tap_detected = false;  // Timer expired, no second tap ? it's a single tap
    printf("Single Tap Detected!\n");
}

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

esp_err_t i2c_write_register(uint8_t i2c_reg, uint8_t data_wr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADXL345_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, i2c_reg, true);
    i2c_master_write_byte(cmd, data_wr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_register(uint8_t i2c_reg, uint8_t *data_rd) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
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

void adxl345_init() {
    struct {
        uint8_t reg;
        uint8_t value;
    } adxl_config[] = {
        { PWR_CTL_REG, 0x28 },
        { THRESH_TAP, 0x35 },
        { DUR, 0x10 },
        { LATENT, 0x20 },
        { WINDOW, 0x90 },
        { TAP_AXES, 0x01 },
        { INT_ENABLE, 0x78 },
        { INT_MAP, 0x18 },
        { DATA_FORMAT, 0x0B },
        { THRESH_ACT, 0x1D },
        { THRESH_INACT, 0x02 },
        { TIME_INACT, 0x04 },
        { ACT_INACT_CTL, 0x66 }
    };

    for (int i = 0; i < sizeof(adxl_config) / sizeof(adxl_config[0]); i++) {
        i2c_write_register(adxl_config[i].reg, adxl_config[i].value);
    }
}

void detect_tap_event() {
    uint8_t int_status;
    i2c_read_register(ADXL345_INT_SOURCE, &int_status);
    printf("int source:0x%02X\n",int_status);

    if (int_status & (1 << 5)) {  // Double Tap Detected
        xTimerStop(tap_timer, 0);
        first_tap_detected = false;
        printf("Double Tap Detected!\n");
    } 
    else if (int_status & (1 << 6)) {  // Single Tap Detected
        if (!first_tap_detected) {
            first_tap_detected = true;
            xTimerStart(tap_timer, 0);  // Start timer for single tap verification
        }
    }
}
void detect_mov_event() {
   uint8_t int_status;
   i2c_read_register(ADXL345_INT_SOURCE, &int_status);
   printf("int source:0x%02X\n",int_status);
   if (int_status & (1 << 4)) {
        printf("Activity Detected!\n");
    }else if (int_status & (1 << 3)) {
        printf("Inactivity Detected!\n");
    }
}

void IRAM_ATTR tap_isr_handler(void *arg) {
    uint8_t tap_type = 1;
    xQueueSendFromISR(tap_event_queue, &tap_type, NULL);
}
void IRAM_ATTR mov_isr_handler(void*arg){
    uint8_t mov_type = 1;
    xQueueSendFromISR(mov_event_queue, &mov_type,NULL);
}

void gpio_init() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << INT1_GPIO)|(1ULL << INT2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INT1_GPIO, tap_isr_handler, (void*)INT1_GPIO);
    gpio_isr_handler_add(INT2_GPIO, mov_isr_handler, (void*)INT2_GPIO);
}

void tap_event_task(void *pvParameters) {
    uint8_t tap_type;
    while (1) {
        xQueueReceive(tap_event_queue, &tap_type, portMAX_DELAY);
        detect_tap_event();
    }
}
void mov_event_task(void*pvParameters){
    uint8_t mov_type;
    while(1){
         xQueueReceive(mov_event_queue,&mov_type,portMAX_DELAY);
         detect_mov_event();
         }
}

void app_main() {
    i2c_master_init();
    gpio_init();
    adxl345_init();
    tap_event_queue = xQueueCreate(10, sizeof(uint8_t));
    mov_event_queue = xQueueCreate(10, sizeof(uint8_t));
    tap_timer = xTimerCreate("TapTimer", pdMS_TO_TICKS(200), pdFALSE, 0, tap_timer_callback);

    if (tap_event_queue == NULL || tap_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create queue or timer!");
        return;
    }
    xTaskCreate(tap_event_task, "Tap_Event_Task", 4096, NULL, 5, NULL);
    xTaskCreate(mov_event_task, "Mov_Event_Task" , 4096,NULL,5,NULL);
}