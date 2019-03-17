#ifndef __SPI_GPIO_H
#define __SPI_GPIO_H

void init_spi_gpio();
int send_command(uint8_t command, uint32_t argument, uint8_t crc);

int init_sd_card();

void read_sector(uint32_t sector, uint8_t buf[512]);
void write_sector(uint32_t sector, const uint8_t buf[512]);


#endif
