#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "RPR-0521rs";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define ALS_DATA0_LSB_REG 0x46 // RPR0521RSセンサーのALSデータレジスタの低位バイト(LSB)のアドレス
#define RPR0521RS_ADDR 0x38
#define ACK_CHECK_EN 0x1

static esp_err_t read_als_data0_value(uint16_t *als_value)
{
    uint8_t data_rd[2] = {0};
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
    if (ret == ESP_OK)
    {
        uint8_t als_data0_lsb = data_rd[0];
        uint8_t als_data0_msb = data_rd[1];
        *als_value = ((uint16_t)als_data0_msb << 8) | als_data0_lsb;
    }
    return ret;
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK)
        return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
}

esp_err_t configure_rpr0521rs(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RPR0521RS_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t rpr0521_init(void)
{
    // MODE_CONTROLの設定値
    // ALS_EN:ALS 測定オン
    // PS_EN:PS スタンバイ
    // PS_PULSE:PS LED パルス幅 typ:200us
    // PS Operating mode:ノーマルモード
    // Measurement time:ALS(400ms) PS(standby)
    if (configure_rpr0521rs(I2C_MASTER_NUM, 0x41, 0x8A) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure MODE_CONTROL");
        return ESP_FAIL;
    }
    // ALS_PS_CONTROLの設定値(default value 0x02)
    // ALS DATA0 GAIN:ALS Gain x1
    // ALS DATA1 GAIN:ALS Gain x1
    // LED CURRENT:100mA
    if (configure_rpr0521rs(I2C_MASTER_NUM, 0x42, 0x02) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure ALS_PS_CONTROL");
        return ESP_FAIL;
    }
    return ESP_OK;
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
        esp_err_t ret = read_als_data0_value(&als_value);
        if (ret == ESP_OK)
            ESP_LOGI(TAG, "ALS Reading: %d lux", als_value);
        else
            ESP_LOGE(TAG, "Failed to read ALS value");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        count++;
    }
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
