/*
 * This file handles working with flash and settings
 */
#include "flash.h"
#include <abstractFLASH.h>

/** Size of a page in words */
const uint16_t page_size = 1024 / 4;
const int32_t *pid_speed_flash = 0x08000000 + 50 * 1024;
const int32_t *pid_current_flash = 0x08000000 + 51 * 1024;
const int32_t *pid_position_flash = 0x08000000 + 52 * 1024;


static bool all_ones(struct pid_settings *settings)
{
    int32_t *array = (int32_t *)settings;
    uint16_t N = sizeof(*settings) / sizeof(int32_t);
    for (uint16_t i = 0; i < N; i++) {
        if (array[i] != -1)
            return false;
    }
    return true;
}

static const int32_t *addr_conv(enum pid_types type)
{
    switch (type) {
        case SPEED_PID:
            return pid_speed_flash;
            break;
        case CURRENT_PID:
            return pid_current_flash;
            break;
        case POSITION_PID:
            return pid_position_flash;
            break;
        default:
            return false;
    }
}

/**
 * Read settings of PID regulator from FLASH
 * 
 * :param settings: Pointer to a :c:type:`pid_settings` where data will we written
 * :param type: Type of PID :c:type:`pid_types`.
 * :return: True if settings were read and False if settings werent found.
 */
bool read_settings_flash(struct pid_settings *settings, enum pid_types type)
{
    const int32_t *addr = addr_conv(type);

    struct pid_settings buff = {0};
    uint16_t step = sizeof(buff) / sizeof(int32_t);
    for (uint16_t i = 0; i < page_size; i++) {
        abst_flash_read(addr + i * step, step, (int32_t *)&buff);
        
        if (all_ones(&buff) && i == 0)
            return false; // Settings are empty
        
        if (all_ones(&buff) || i == page_size - step) {
            abst_flash_read(addr + (i - 1) * step, step, (int32_t *)&buff); // Read previous
            *settings = buff;
            return true;
        }
    }
    
    return false;
}
/**
 * Write settings of PID regulator into a FLASH
 * 
 * :param settings: Pointer to a :c:type:`pid_settings` where data is stored
 * :param type: Type of PID :c:type:`pid_types`.
 * :return: True if operation success, False otherwise.
 */
bool write_settings_flash(struct pid_settings *settings, enum pid_types type)
{
    const int32_t *addr = addr_conv(type);
    
    struct pid_settings buff = {0};
    uint16_t step = sizeof(buff) / sizeof(int32_t);
    uint16_t i;
    for (i = 0; i < page_size; i++) {
        abst_flash_read(addr + i * step, step, (int32_t *)&buff);
        
        if (all_ones(&buff)) {
            break;
        }
    }
    if (i == page_size - 1) {
        uint32_t erase_status = abst_flash_erase_page(addr);
        if (erase_status != ABST_OK)
            return false;
        i = 0;
    }
    
    abst_flash_write(addr + i * step, step, (int32_t *)settings);
    return true;
 } 
