
#include "stm32f2xx_hal.h"
#include "stm32f2xx_hal_gpio.h"

#include <stdio.h>

uint8_t spi_gpio_cs;
uint8_t spi_gpio_clk;
uint8_t spi_gpio_data_in;
uint8_t spi_gpio_data_out;

uint8_t g_block_buffer[512] = {};

static void init_spi_gpio_pins() {

    GPIO_InitTypeDef spi_gpio;

    spi_gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    spi_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    spi_gpio.Pull = GPIO_NOPULL;
    spi_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOC, &spi_gpio);

    spi_gpio.Pin = GPIO_PIN_2;

    HAL_GPIO_Init(GPIOD, &spi_gpio);

    spi_gpio.Pin = GPIO_PIN_8;
    spi_gpio.Mode = GPIO_MODE_INPUT;
    spi_gpio.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC, &spi_gpio);
}


void init_spi_gpio() {

    init_spi_gpio_pins();

}

void enable_cs() {
    GPIOC->BSRR = GPIO_PIN_11 << 16;
    spi_gpio_cs = 0;
}

void disable_cs() {
    GPIOC->BSRR = GPIO_PIN_11;
    spi_gpio_cs = 1;
}

void set_clk() {
    GPIOC->BSRR = GPIO_PIN_12;
    spi_gpio_clk = 1;
}

void clear_clk() {
    GPIOC->BSRR = GPIO_PIN_12 << 16;
    spi_gpio_clk = 0;
}

void set_data() {
    GPIOD->BSRR = GPIO_PIN_2;
    spi_gpio_data_out = 1;
}

void clear_data() {
    GPIOD->BSRR = GPIO_PIN_2 << 16;
    spi_gpio_data_out = 0;
}

void spi_delay() {

    // Turns out function call overhead is enough of a delay

    volatile int i;
    for (i = 0; i < 10; i++)
    {}
}

uint8_t get_data() {
    if (GPIOC->IDR & GPIO_PIN_8) {
        spi_gpio_data_in = 1;
        return 1;
    } else {
        spi_gpio_data_in = 0;
        return 0;
    }
}


// MSB first
uint8_t send_byte(uint8_t data_in) {

    uint8_t data_out = 0;

    if (data_in & 0x80) {
        set_data();
    } else {
        clear_data();
    }

    data_in <<= 1;
    data_in |= 1;

    int count = 8;
    while(count--) {

        spi_delay();

        // Latch
        set_clk();
        data_out <<= 1;
        data_out |= get_data();

        spi_delay();

        // Shift
        if (data_in & 0x80) {
            set_data();
        } else {
            clear_data();
        }
        data_in <<= 1;

        clear_clk();
    }

    spi_delay();
    spi_delay();

    return data_out;
}

static int read_data_packet(uint8_t block_buffer[512]) {

    uint8_t resp;

    // Wait until we recieve the token
    do {
        resp = send_byte(0xFF);
    } while (resp == 0xFF);

    int i;
    for (i = 0; i < 512; i++) {
        block_buffer[i] = send_byte(0xFF);
    }

    // Read CRC16
    resp = send_byte(0xFF);
    resp = send_byte(0xFF);

    return 0;
}

static void send_command_helper(uint8_t command, uint32_t argument, uint8_t crc) {

    uint16_t cmd_high = 0x4000 |
                        ((command & 0x3F) << 8) |
                        ((argument >> 24) & 0xFF);

    uint32_t cmd_low = (argument << 8) |
                       ((crc & 0x7F) << 1) | // Placeholder for CRC
                       1;

    clear_clk();
    enable_cs();

    uint8_t resp;
    do {
        resp = send_byte(0xFF);
    } while (resp != 0xFF);

    send_byte((cmd_high >> 8) & 0xFF);
    send_byte(cmd_high & 0xFF);

    send_byte((cmd_low >> 24) & 0xFF);
    send_byte((cmd_low >> 16) & 0xFF);
    send_byte((cmd_low >> 8) & 0xFF);
    send_byte((cmd_low) & 0xFF);
}


int send_command(uint8_t command, uint32_t argument, uint8_t crc) {

    send_command_helper(command, argument, crc);

    set_data();

    disable_cs();

    return 0;
}

int send_command_resp1(uint8_t command, uint32_t argument, uint8_t crc) {

    send_command_helper(command, argument, crc);

    set_data();

    int sd_timeout = 100;
    while (sd_timeout) {

        spi_delay();

        if (get_data() == 0) {
            break;
        }

        set_clk();

        spi_delay();

        clear_clk();
        
        sd_timeout--;
    }
    
    if (sd_timeout == 0) {
        printf("SPI Timeout\r\n");
        disable_cs();
        return -1;
    }

    uint8_t resp = send_byte(0xFF);

    disable_cs();

    return resp;
}

int send_command_resp1_nocs(uint8_t command, uint32_t argument, uint8_t crc) {

    send_command_helper(command, argument, crc);

    set_data();

    int sd_timeout = 60;
    while (sd_timeout) {

        spi_delay();

        if (get_data() == 0) {
            break;
        }

        set_clk();

        spi_delay();

        clear_clk();
        
        sd_timeout--;
    }
    
    if (sd_timeout == 0) {
        printf("SPI Timeout\r\n");
        disable_cs();
        return -1;
    }

    uint8_t resp = send_byte(0xFF);

    return resp;
}

int send_command_resp3(uint8_t command, uint32_t argument, uint8_t crc, uint8_t resp[5]) {

    send_command_helper(command, argument, crc);

    set_data();

    int sd_timeout = 60;
    while (sd_timeout) {

        spi_delay();

        if (get_data() == 0) {
            break;
        }

        set_clk();

        spi_delay();

        clear_clk();
        
        sd_timeout--;
    }
    
    if (sd_timeout == 0) {
        printf("SPI Timeout\r\n");
        disable_cs();
        return -1;
    }

    resp[0] = send_byte(0xFF);
    resp[1] = send_byte(0xFF);
    resp[2] = send_byte(0xFF);
    resp[3] = send_byte(0xFF);
    resp[4] = send_byte(0xFF);

    //resp[0] >>= 1;

    disable_cs();

    return 0;
}

int send_command_data(uint8_t command, uint32_t argument, uint8_t crc, uint8_t buf[512]) {

    send_command_helper(command, argument, crc);

    set_data();

    int sd_timeout = 60;
    while (sd_timeout) {

        spi_delay();

        if (get_data() == 0) {
            break;
        }

        set_clk();

        spi_delay();

        clear_clk();
        
        sd_timeout--;
    }

    uint8_t resp = send_byte(0xFF);

    read_data_packet(buf);

    return resp;
}

int send_data(const uint8_t buf[512]) {

    // Assume CS is alread enabled
    
    send_byte(0xFF);

    uint8_t resp;

    do {
        resp = send_byte(0xFF);
    } while (resp != 0xFF);

    // Data Packet
    send_byte(0xFE);

    int i;
    for (i = 0; i < 512; i++) {
        send_byte(buf[i]);
    }

    // CRC
    send_byte(0);
    send_byte(0);

    uint8_t data_resp = send_byte(0xFF);

    if ((data_resp & 0x1F) != 0x5) {
        printf("Invalid data response: %.2X\r\n", data_resp & 0x1F);
        return -1;
    }

    do {
        resp = send_byte(0xFF);
    } while (resp != 0xFF);

    disable_cs();

    return 0;
}

int init_sd_card() {

    set_data();
    disable_cs();

    int clks;
    for (clks = 80; clks; clks--) {
        spi_delay();
        set_clk();
        spi_delay();
        clear_clk();
    }

    int resp = send_command_resp1(0, 0, 0x4A);
    if (resp != 0x01) {
        return -1;
    }

    spi_delay();
    spi_delay();
    spi_delay();
    spi_delay();

    uint8_t ocr_resp[5] = {};
    (void)ocr_resp[0];

    int i;
    for (i = 0; i < 5; i++) {
        if (send_command_resp3(8, 0x1AA, 67, ocr_resp) != 0){
        //if (send_command_resp3(58, 0, 0x3A, ocr_resp) != 0){
            //return -2;
        }

        printf("OCR: %.2X %.2X %.2X %.2X %.2X\r\n",
               ocr_resp[0], ocr_resp[1], ocr_resp[2], ocr_resp[3], ocr_resp[4]);

        spi_delay();
        spi_delay();
        spi_delay();
        spi_delay();
    }

    for (i = 0; i < 10; i++) {

        spi_delay();
        spi_delay();
        spi_delay();
        spi_delay();

        resp = send_command_resp1(55, 0, 0);
        printf("Resp 55 %d: %.2X\r\n", i,  resp);

        spi_delay();
        spi_delay();
        spi_delay();
        spi_delay();

        resp = send_command_resp1(41, 0x40000000, 0);
        printf("Resp A41 %d: %.2X\r\n", i, resp);

        if (resp == 0) {
            break;
        }

        int j;
        for (j = 0; j < 1000000; j++)
        {}

    }

    if (send_command_resp3(58, 0, 0, ocr_resp) != 0) {
        return -3;
    }


    printf("OCR: %.2X %.2X %.2X %.2X %.2X\r\n",
           ocr_resp[0], ocr_resp[1], ocr_resp[2], ocr_resp[3], ocr_resp[4]);

    return 0;
}

void read_sector(uint32_t sector, uint8_t buf[512]) {
    send_command_data(17, sector, 0, buf);
}

void write_sector(uint32_t sector, const uint8_t buf[512]) {
    uint8_t resp = send_command_resp1_nocs(24, sector, 0);
    if (resp == 0) {

    } else {
        printf("Error Writing to sd: %.2X\r\n", resp);
        disable_cs();
    }

    send_data(buf);
}


