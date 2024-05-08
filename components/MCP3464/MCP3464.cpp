#include "MCP3464.hpp"

MCP3464::MCP3464(spi_host_device_t bus, gpio_num_t cs){
    // デバイスの初期化
    memset(&dev_adc,0,sizeof(dev_adc));
    dev_adc.clock_speed_hz = 20*1000*1000;
    dev_adc.mode = 3;
    dev_adc.spics_io_num = cs;
    dev_adc.queue_size = 1;

    err = spi_bus_add_device(bus, &dev_adc, &_spi);
    ESP_ERROR_CHECK(err);

    vTaskDelay(1/portTICK_PERIOD_MS);
    //uint8_t fullreset_reg = 0b01 << 6 | 0b1110 << 2 | 0b00;
    uint8_t fullreset_reg = 0xE << 2 | ( 0x02 | (0x01 << 6));
    uint8_t ans = read(fullreset_reg);

    vTaskDelay(pdTICKS_TO_MS(1));

    uint8_t reserved_reg = 0b01 << 6 | 0xF << 2 | 0b01;
    uint16_t device_id = read(reserved_reg);
}

MCP3464::~MCP3464(){}


void MCP3464::write(void* pData, uint8_t size){
    spi_transaction_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd.length = size * 8;
    cmd.tx_buffer = pData;
    err = spi_device_polling_transmit(_spi, &cmd);
    ESP_ERROR_CHECK(err);

    vTaskDelay(pdMS_TO_TICKS(10));
    return;
}

uint8_t MCP3464::read(uint8_t reg){
    spi_transaction_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    cmd.length = 8;
    cmd.tx_data[0] = reg;
    err = spi_device_polling_transmit(_spi, &cmd);
    ESP_ERROR_CHECK(err);

    ESP_LOGI("MCP3464", "send: 0x%x", cmd.tx_data[0]);
    ESP_LOGI("MCP3464", "recv[0]: 0x%x", cmd.rx_data[0]);
    ESP_LOGI("MCP3464", "recv[1]: 0x%x", cmd.rx_data[1]);
    ESP_LOGI("MCP3464", "recv[2]: 0x%x", cmd.rx_data[2]);
    ESP_LOGI("MCP3464", "recv[3]: 0x%x", cmd.rx_data[3]);

    return cmd.rx_data[1];
}

uint16_t MCP3464::read16(uint8_t reg){
    spi_transaction_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    cmd.length = 24;
    cmd.tx_data[0] = reg;
    err = spi_device_polling_transmit(_spi, &cmd);
    ESP_ERROR_CHECK(err);

    return cmd.rx_data[1] << 8 | cmd.rx_data[2];

}

void MCP3464::GetData(t_sens_data *_sens)
{
    sens = _sens;
}

/*float MCP3464::BatteryVoltage()
{
    int _raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1, VBATT_CHANNEL, &_raw));
    return (float)((float)(_raw) * (2.2 / 1000.0));
}

void MCP3464::ReadSensor(int *sensors, uint8_t mask)
{
    if (mask & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_FL.channel, &sensors[0]));
    if ((mask >> 1) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_L.channel, &sensors[1]));
    if ((mask >> 2) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_R.channel, &sensors[2]));
    if ((mask >> 3) & 1)
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, led_FR.channel, &sensors[3]));
        
}

void MCP3464::SetIRLED(uint8_t led)
{
    gpio_set_level(led_FR.pin, led & 1);
    gpio_set_level(led_R.pin, (led >> 1) & 1);
    gpio_set_level(led_L.pin, (led >> 2) & 1);
    gpio_set_level(led_FL.pin, (led >> 3) & 1);
    
}

void MCP3464::WallSensor()
{

    ReadSensor(before, 0b1111); // 全消灯での値を取得

    SetIRLED(0b1001);            // fl,fr点灯
    ets_delay_us(300);           // 100us待つ
    ReadSensor(sensors, 0b1001); // fl,fr点灯での値を取得

    SetIRLED(0b0110);                   // l,r点灯
    ets_delay_us(300);                  // 100us待つ
    ReadSensor(sensors, 0b0110);        // l,r点灯での値を取得
    SetIRLED(0b0000);                   // 全消灯
    //vTaskDelay(1 / portTICK_PERIOD_MS); // 1ms待つ

    sens->wall.val.fl = sensors[0] - before[0];
    sens->wall.val.l = sensors[1] - before[1];
    sens->wall.val.r = sensors[2] - before[2];
    sens->wall.val.fr = sensors[3] - before[3];
}*/


void MCP3464::adc_loop()
{
    /*while (1)
    {
        WallSensor();
        sens->BatteryVoltage = BatteryVoltage();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }*/
}
