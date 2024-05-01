#include "MA730.hpp"

MA730::MA730(spi_host_device_t bus, gpio_num_t cs)
{
    // SPIデバイスの設定
    memset(&dev_enc, 0, sizeof(dev_enc));
    dev_enc.clock_speed_hz = 25 * 1000 * 1000; // Max 25MHz
    // dev_enc.mode = 1;                        // Mode 0 CPOL=0 CPHA=0, Mode 3 CPOL=1 CPHA=1    要らないかもしれない
    dev_enc.spics_io_num = cs;
    dev_enc.queue_size = 1;
    // dev_enc.cs_ena_pretrans = 4;    // 送信前に cs がアクティブになる SPI ビットサイクルの量（0-16）。半二重トランザクションでのみ動作する  多分要らない

    err = spi_bus_add_device(bus, &dev_enc, &_spi);
    ESP_ERROR_CHECK(err);

    printf("encoder init\n");
}

MA730::~MA730() {}

uint16_t MA730::read()
{
    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd.length = 16;
    cmd.tx_data[0] = 0;
    cmd.tx_data[1] = 0;
    err = spi_device_polling_transmit(_spi, &cmd);
    assert(err == ESP_OK);

    return cmd.rx_data[0] << 8 | cmd.rx_data[1];
}

uint16_t MA730::readAngle()
{
    return read() >> 2;
}

uint16_t MA730::OperateRegisters(const uint8_t command, const uint8_t address, const uint8_t data)
{
    memset(&cmd, 0, sizeof(cmd));
    cmd.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd.length = 16;
    cmd.tx_data[0] = (command << 5) | address;
    cmd.tx_data[1] = data;
    err = spi_device_polling_transmit(_spi, &cmd);
    assert(err == ESP_OK);

    return cmd.rx_data[0] << 8 | cmd.rx_data[1];
}

uint8_t MA730::ReadRegister(uint8_t address, uint8_t data)
{
    OperateRegisters(READ_COMMAND, address, data);
    return read() >> 8;
}

uint8_t MA730::WriteRegister(uint8_t address, uint8_t data)
{
    OperateRegisters(WRITE_COMMAND, address, data);
    return read() >> 8;
}

/*void MA730::GetData(t_sens_data *_sens)
{
    sens = _sens;
}*/