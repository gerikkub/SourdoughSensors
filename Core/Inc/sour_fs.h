
#ifndef __SOUR_FS_H__
#define __SOUR_FS_H__

#include <stdint.h>
#include <stdbool.h>

struct TrialEntry {
	uint32_t ts;
	uint32_t temp_dn[5];
	bool have_img;
	uint32_t img_num;
};

uint32_t create_trial();

bool add_trial_entry(uint32_t trial_num, struct TrialEntry* entry, uint8_t* img_data, uint32_t img_len);

#endif