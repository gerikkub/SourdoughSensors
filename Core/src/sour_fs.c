
#include "sour_fs.h"
#include "ff.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

static const char* trial_path = "SOUR/TRIAL";

static bool open_or_create_dir(const char* path, DIR* dir) {

	FRESULT stat;

	stat = f_opendir(dir, path);

	if (stat == FR_OK) {
		return true;
	} else if (stat == FR_NO_PATH) {
		// Create the directory

		stat = f_mkdir(path);

		if (stat != FR_OK) {
			return false;
		} else {
			stat = f_opendir(dir, path);

			return stat == FR_OK;
		}
	} else {
		return false;
	}
}

uint32_t create_trial() {

	DIR sour_dir;
	FILINFO file;
	FRESULT stat;

	uint32_t max_trial_num = 0;
	uint32_t trial_num;
	char filepath_str[265] = {0};

	// Enumerate current files
	if (!open_or_create_dir("SOUR", &sour_dir)) {
		return 0;
	}

	do {
		stat = f_readdir(&sour_dir, &file);

		if (stat != FR_OK) {
			f_closedir(&sour_dir);
			return 0;
		}

		if (file.fname[0] != '\0') {
			sscanf(file.fname, "TRIAL%lu", &trial_num);

			if (trial_num > max_trial_num) {
				max_trial_num = trial_num;
			}
		}
	} while (file.fname[0] != '\0');

	f_closedir(&sour_dir);

	snprintf(filepath_str, sizeof(filepath_str),
					"%s%lu",
					trial_path, max_trial_num + 1);

	if (!open_or_create_dir(filepath_str, &sour_dir)) {
		return 0;
	}

	f_closedir(&sour_dir);

	return max_trial_num + 1;
}

bool add_trial_entry(uint32_t trial_num, struct TrialEntry* entry, uint8_t* img_data, uint32_t img_len) {

	FIL file;
	FRESULT stat;
	char entry_str[512] = {0};
	char filepath_str[265] = {0};
	uint32_t len;
	UINT len_written;

	len = snprintf(entry_str, sizeof(entry_str),
					"%lu, %lu, %lu, %lu, %lu, %lu, %u, %lu\n",
					entry->ts,
					entry->temp_dn[0],
					entry->temp_dn[1],
					entry->temp_dn[2],
					entry->temp_dn[3],
					entry->temp_dn[4],
					entry->have_img,
					entry->img_num
					);

	snprintf(filepath_str, sizeof(filepath_str),
					"%s%lu/DATA.CSV",
					trial_path, trial_num);

	stat = f_open(&file, filepath_str, FA_OPEN_ALWAYS | FA_WRITE);

	if (stat != FR_OK) {
		return false;
	}

	stat = f_lseek(&file, f_size(&file));

	if (stat != FR_OK) {
		return false;
	}

	stat = f_write(&file, entry_str, len, &len_written);

	if (stat != FR_OK) {
		f_close(&file);
		return false;
	}

	f_close(&file);

	if (img_data != NULL &&
		entry->have_img) {

		snprintf(filepath_str, sizeof(filepath_str),
						"%s%lu/IMG%lu.jpg",
						trial_path, trial_num,
						entry->img_num);

		stat = f_open(&file, filepath_str, FA_CREATE_NEW | FA_WRITE);

		if (stat != FR_OK) {
			return false;
		}

		stat = f_write(&file, img_data, img_len, &len_written);

		if (stat != FR_OK) {
			f_close(&file);
			return false;
		}

		f_close(&file);
	}

	return true;
}

