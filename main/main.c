#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "RPR-0521";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0                // I2Cポート番号
#define I2C_MASTER_FREQ_HZ 400000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define ALS_DATA0_LSB_REG 0x46 // ALS DATA0 測定結果(low byte)
#define RPR0521RS_ADDR 0x38
#define ACK_CHECK_EN 0x1

static esp_err_t read_als_value(uint16_t *als_value)
{
    uint8_t data_rd[2] = {0}; // ALS DATA0 測定結果(low byte)とALS DATA0 測定結果(high byte)を格納するためのバッファ
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RPR0521RS_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ALS_DATA0_LSB_REG, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RPR0521RS_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, 2, I2C_MASTER_LAST_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_LOGI(TAG, "msb:%d, lsb:%d", data_rd[1], data_rd[0]);

    *als_value = ((uint16_t)data_rd[1] << 8) | data_rd[0];
    return ESP_OK;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t write_rpr0521_config(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t value)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RPR0521RS_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t rpr0521_init(void)
{
    esp_err_t ret;
    // setup MODE_CONTROL
    ret = write_rpr0521_config(I2C_MASTER_NUM, 0x41, 0x8A);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MODE_CONTROL setting failed");
    }
    // setup ALS_PS_CONTROL
    // ret = setup_sensor_config(I2C_MASTER_NUM, 0x42, 0x02);
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "ALS_PS_CONTROL setting failed");
    // }
    ESP_LOGI(TAG, "configuration initialized successfully");
    return ret;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(rpr0521_init());

    int count = 0;
    while (count < 15)
    {
        uint16_t als_value;
        ESP_ERROR_CHECK(read_als_value(&als_value));
        printf("als value:%d\n", als_value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        count++;
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
