

#ifndef __ONE_WIRE__
#define __ONE_WIRE__

#include <stdbool.h>

#define MAX_DEVICE_TREE_SIZE 512

struct OneWireDevice {
	uint8_t id[8];
};

enum OneWireCmds {
	OW_SEARCH_ROM_CMD = 0xF0,
	OW_READ_ROM_CMD = 0x33,
	OW_MATCH_ROM_CMD = 0x55,
	OW_SKIP_ROM_CMD = 0xCC
};

enum DS18B20Cmds {
	DS18B20_CONVERT_CMD = 0x44,
	DS18B20_WRITE_SCRATCHPAD = 0x4E,
	DS18B20_READ_SCRATCHPAD = 0xBE
};

void one_wire_init();

bool one_wire_reset();
//void one_wire_write_byte(uint8_t b);

void one_wire_write_bit(uint8_t b);
void one_wire_write_byte(uint8_t byte);

uint8_t one_wire_read_bit();
uint8_t one_wire_read_byte();

void one_wire_read_rom(uint8_t* rx, uint32_t rxSize);

void one_wire_skip_rom_cmd(uint8_t cmd);
void one_wire_address_device(struct OneWireDevice* device);

uint16_t one_wire_read_temp(struct OneWireDevice* device);

int32_t one_wire_discover(struct OneWireDevice* devices, uint32_t num_devices);



#endif