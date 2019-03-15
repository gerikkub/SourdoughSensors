
#include "main.h"
#include "one_wire.h"

struct OneWireDeviceTree {
	struct OneWireDevice device;
	uint32_t valid_bits;

	struct OneWireDeviceTree* zero, *one;
};

static struct OneWireDeviceTree device_tree[MAX_DEVICE_TREE_SIZE];
static uint32_t device_tree_alloc_idx;

static void tmr_us_init() {

	__HAL_RCC_TIM2_CLK_ENABLE();

	TIM2->CR1 = TIM_CR1_DIR | // Downcounter
				TIM_CR1_URS; // Only generate update event on overflow

	TIM2->PSC = 47;

	// TIM2->CCMR1 = TIM_CCMR1_CC2S_0;
	// TIM2->CCER = TIM_CCER_CC2P | TIM_CCER_CC2NP;
	// TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_TS_1;
}

static void tmr_us_delay_noblock(uint32_t us) {
	TIM2->ARR = us;

	// CC2S = 01 in CCMR1
	// CC2P = 0 and CC2NP = 0 in CCER
	// TS=110 in SMCR
	// SMS to 110 in SMCR

	// Enable the timer
	TIM2->CR1 |= TIM_CR1_CEN;

	TIM2->EGR |= TIM_EGR_UG;
	TIM2->SR = 0;

	// TIM2->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
}

static uint32_t tmr_us_get_cnt() {
	return TIM2->CNT;
}

static bool tmr_us_delay_check() {
	// return TIM2->CNT == 0;
	return TIM2->SR & TIM_SR_UIF;
}

static void tmr_us_delay(uint32_t us) {

	tmr_us_delay_noblock(us);

	// Wait until timer hits zero
	while (!tmr_us_delay_check()) {
	}
}

static void write_bit_to_id(struct OneWireDevice* device, uint32_t position, uint8_t value) {

	uint32_t byte_num = position / 8;
	uint32_t bit_num = position % 8;

	if (value == 0) {
		device->id[byte_num] &= ~(1 << bit_num);
	} else {
		device->id[byte_num] |= (1 << bit_num);
	}
}

static uint8_t get_bit_from_id(struct OneWireDevice* device, uint32_t position) {

	uint32_t byte_num = position / 8;
	uint32_t bit_num = position % 8;

	return (device->id[byte_num] >> bit_num) & 1;
}

static struct OneWireDeviceTree* allocate_node(struct OneWireDevice device, uint32_t valid_bits) {

	if (device_tree_alloc_idx >= MAX_DEVICE_TREE_SIZE) {
		return NULL;
	}

	struct OneWireDeviceTree* new_node = &device_tree[device_tree_alloc_idx];
	device_tree_alloc_idx++;

	new_node->device = device;
	new_node->valid_bits = valid_bits;

	new_node->zero = NULL;
	new_node->one = NULL;

	return new_node;
}

void one_wire_init() {

	tmr_us_init();

	device_tree_alloc_idx = 0;

	struct OneWireDevice dummy_device;
	int i;
	for (i = 0; i < 8; i++) {
		dummy_device.id[i] = 0;
	}

	allocate_node(dummy_device, 0);
}

bool one_wire_reset() {

	uint32_t low_pulse_start = 0;
	bool found_presence = false;

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_RESET);

	tmr_us_delay(500);

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_SET);

	// Wait for rise time
	tmr_us_delay(15);

	tmr_us_delay_noblock(500);

	while (!tmr_us_delay_check()) {
		if (HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin) == GPIO_PIN_RESET) {
			if (low_pulse_start == 0) {
				// Get the start time of the low pulse
				low_pulse_start = tmr_us_get_cnt();
			}
		} else {
			if (low_pulse_start != 0) {
				uint32_t low_pulse_end = tmr_us_get_cnt();

				uint32_t pulse_width = low_pulse_start - low_pulse_end;

				if (pulse_width > 60) {
					found_presence = true;
				} else {
					// Too short of a pulse. Check again.
					low_pulse_start = 0;
				}
			}
		}
	}

	return found_presence;
}

void one_wire_write_bit(uint8_t bit) {

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_RESET);

	if (bit == 0) {
		tmr_us_delay(100);
	} else {
		tmr_us_delay(10);
	}

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_SET);

	if (bit == 0) {
		tmr_us_delay(10);
	} else {
		tmr_us_delay(150);
	}
}

void one_wire_write_byte(uint8_t byte) {

	int i;
	for (i = 0; i < 8; i++) {
		one_wire_write_bit(byte & (1 << i));
	}
}


uint8_t one_wire_read_bit() {

	uint8_t ret;

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_RESET);

	tmr_us_delay(5);

	HAL_GPIO_WritePin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin, GPIO_PIN_SET);

	tmr_us_delay(15);

	ret = HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin) == GPIO_PIN_SET ?
			1 : 0;

	tmr_us_delay_noblock(100);

	while (HAL_GPIO_ReadPin(ONE_WIRE_GPIO_Port, ONE_WIRE_Pin) == GPIO_PIN_RESET) {
		if (tmr_us_get_cnt() == 0) {
			// Timeout
			ret = 0xFF;
			break;
		}
	}

	while (!tmr_us_delay_check()) {
	}

	return ret;
}

uint8_t one_wire_read_byte() {

	uint8_t ret = 0;
	uint8_t tmp;
	int i;

	for (i = 0; i < 8; i++) {
		tmp = one_wire_read_bit();

		ret |= (tmp & 1) << i;
	}

	return ret;
}

void one_wire_read_rom(uint8_t* rx, uint32_t rxSize) {

	if (rxSize < 8) {
		return;
	}

	one_wire_reset();

	one_wire_write_byte(0x33);

	int i;
	for (i = 0; i < 8; i++) {
		rx[i] = one_wire_read_byte();
	}
}

void one_wire_skip_rom_cmd(uint8_t cmd) {

	one_wire_reset();

	one_wire_write_byte(OW_SKIP_ROM_CMD);

	one_wire_write_byte(cmd);
}

void one_wire_address_device(struct OneWireDevice* device) {
	one_wire_reset();

	one_wire_write_byte(OW_MATCH_ROM_CMD);

	int i;
	for (i = 0; i < 64; i++) {
		one_wire_write_bit(get_bit_from_id(device, i));
	}
}

uint16_t one_wire_read_temp(struct OneWireDevice* device) {

	uint8_t scratchpad[9];

	//one_wire_skip_rom_cmd(0x44);
	one_wire_address_device(device);

	one_wire_write_byte(DS18B20_CONVERT_CMD);

	uint8_t tmp;

	do {
		tmp = one_wire_read_bit();
	} while (tmp == 0);

	// Read scratchpad
	// one_wire_skip_rom_cmd(0xBE);
	one_wire_address_device(device);
	one_wire_write_byte(DS18B20_READ_SCRATCHPAD);

	int i;
	for (i = 0; i < 9; i++) {
		scratchpad[i] = one_wire_read_byte();
	}

	return scratchpad[0] | (scratchpad[1] << 8);
}

static bool one_wire_search_send_bit(uint8_t select_bit, uint8_t* wrote_bit) {

	uint8_t first, second;
	bool ret_conflict;

	first = one_wire_read_bit();
	second = one_wire_read_bit();

	if (first != second) {
		ret_conflict = false;
		*wrote_bit = first;
		one_wire_write_bit(first);
	} else {
		ret_conflict = true;
		*wrote_bit = select_bit;
		one_wire_write_bit(select_bit);
	}

	return ret_conflict;
}


int32_t one_wire_discover(struct OneWireDevice* devices, uint32_t num_devices) {

	struct OneWireDeviceTree* tree_stack[MAX_DEVICE_TREE_SIZE];
	uint32_t tree_stack_idx;
	struct OneWireDeviceTree* curr_node;
	int i;
	uint32_t curr_device = 0;
	uint8_t read_bit;
	uint32_t bit_num;
	bool status;

	for (i = 0; i < MAX_DEVICE_TREE_SIZE; i++) {
		tree_stack[i] = NULL;
	}

	curr_node = &device_tree[0];
	tree_stack_idx = 0;

	bit_num = 0;

	one_wire_reset();

	one_wire_write_byte(OW_SEARCH_ROM_CMD);

	while (curr_device < num_devices) {
		while (bit_num < 64) {
			// Discovering devices
			status = one_wire_search_send_bit(0, &read_bit);
			if (status == false) {
				// No conflict
				write_bit_to_id(&curr_node->device, bit_num, read_bit);
			} else {
				curr_node->valid_bits = bit_num;
				curr_node->zero = allocate_node(curr_node->device, curr_node->valid_bits);

				tree_stack[tree_stack_idx] = curr_node;
				tree_stack_idx++;
				if (tree_stack_idx > MAX_DEVICE_TREE_SIZE) {
					// Fault. Return nothing
					return -1;
				}

				curr_node = curr_node->zero;

				write_bit_to_id(&curr_node->device, bit_num, 0);
			}

			bit_num++;
		}

		curr_node->valid_bits = bit_num;

		// Discovered a device
		devices[curr_device] = curr_node->device;
		curr_device++;

		if (tree_stack_idx == 0) {
			// Done. Exit early
			break;
		}

		tree_stack_idx--;
		curr_node = tree_stack[tree_stack_idx];

		one_wire_reset();

		one_wire_write_byte(OW_SEARCH_ROM_CMD);

		for (i = 0; i < curr_node->valid_bits; i++) {
			uint8_t dummy;
			one_wire_search_send_bit(get_bit_from_id(&curr_node->device, i), &dummy);
		}

		curr_node->one = allocate_node(curr_node->device, curr_node->valid_bits);
		bit_num = curr_node->valid_bits;
		curr_node = curr_node->one;

		one_wire_search_send_bit(1, &read_bit);
		write_bit_to_id(&curr_node->device, bit_num, 1);
		bit_num++;
	}

	return curr_device;
}