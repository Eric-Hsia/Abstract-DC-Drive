#ifndef _FLASH_H_
#define _FLASH_H_

#include <common_defs.h>

bool read_settings_flash(struct pid_settings *settings, enum pid_types type);

bool write_settings_flash(struct pid_settings *settings, enum pid_types type);

#endif // _FLASH_H_ 
