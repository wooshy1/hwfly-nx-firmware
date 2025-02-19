/*
 * Copyright (c) 2018 naehrwert
 * Copyright (c) 2018-2020 Atmosphère-NX
 * Copyright (c) 2018-2020 CTCaer
 * Copyright (c) 2020 Spacecraft-NX
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <stdbool.h>
#include <stdarg.h>
#include "utils.h"
#include "di.h"
#include "pmc.h"
#include "timers.h"
#include "car.h"
#include "btn.h"
#include "lib/log.h"
#include "lib/vsprintf.h"
#include "display/video_fb.h"
#include "i2c.h"
#include "max77620.h"
#include "fuse.h"
#include "fs_utils.h"

#include <inttypes.h>

void wait(uint32_t microseconds) {
    uint32_t old_time = TIMERUS_CNTR_1US_0;
    while (TIMERUS_CNTR_1US_0 - old_time <= microseconds) {
        /* Spin-lock. */
    }
}

__attribute__((noreturn)) void panic(uint32_t val) {
    // Set panic code.
    PMC(APBDEV_PMC_SCRATCH200) = val;
    //PMC(APBDEV_PMC_CRYPTO_OP) = PMC_CRYPTO_OP_SE_DISABLE;
    TMR(TIMER_WDT4_UNLOCK_PATTERN) = TIMER_MAGIC_PTRN;
    TMR(TIMER_TMR9_TMR_PTV) = TIMER_EN | TIMER_PER_EN;
    TMR(TIMER_WDT4_CONFIG)  = TIMER_SRC(9) | TIMER_PER(1) | TIMER_PMCRESET_EN;
    TMR(TIMER_WDT4_COMMAND) = TIMER_START_CNT;

    while (true) {
        /* Wait for reboot. */
    }
}

#if 0
__attribute__((noreturn)) void watchdog_reboot(void) {
    volatile watchdog_timers_t *wdt = GET_WDT(4);
    wdt->PATTERN = WDT_REBOOT_PATTERN;
    wdt->COMMAND = 2; /* Disable Counter. */
    GET_WDT_REBOOT_CFG_REG(4) = 0xC0000000;
    wdt->CONFIG = 0x8019; /* Full System Reset after Fourth Counter expires, using TIMER(9). */
    wdt->COMMAND = 1; /* Enable Counter. */
    while (true) {
        /* Wait for reboot. */
    }
}

__attribute__((noreturn)) void pmc_reboot(uint32_t scratch0) {
    APBDEV_PMC_SCRATCH0_0 = scratch0;

    /* Reset the processor. */
    APBDEV_PMC_CONTROL = BIT(4);

    while (true) {
        /* Wait for reboot. */
    }
}

__attribute__((noreturn)) void wait_for_button_and_reboot(void) {
    uint32_t button;
    while (true) {
        button = btn_read();
        if (button & BTN_POWER) {
            pmc_reboot(0);
        }
    }
}

__attribute__((noreturn)) void fatal_error(const char *fmt, ...) {
    /* Forcefully initialize the screen if logging is disabled. */
    if (log_get_log_level() == SCREEN_LOG_LEVEL_NONE) {
        /* Zero-fill the framebuffer and register it as printk provider. */
        video_init((void *)0xC0000000);

        /* Initialize the display. */
        display_init();

        /* Set the framebuffer. */
        display_init_framebuffer((void *)0xC0000000);

        /* Turn on the backlight after initializing the lfb */
        /* to avoid flickering. */
        display_backlight(true);
    }
    
    /* Override the global logging level. */
    log_set_log_level(SCREEN_LOG_LEVEL_ERROR);
    
    /* Display fatal error. */
    va_list args;
    print(SCREEN_LOG_LEVEL_ERROR, "Fatal error: ");
    va_start(args, fmt);
    vprint(SCREEN_LOG_LEVEL_ERROR, fmt, args);
    va_end(args);
    print(SCREEN_LOG_LEVEL_ERROR | SCREEN_LOG_LEVEL_NO_PREFIX,"\nPress POWER to reboot\n");
    
    /* Wait for button and reboot. */
    wait_for_button_and_reboot();
}

__attribute__((noinline)) bool overlaps(uint64_t as, uint64_t ae, uint64_t bs, uint64_t be)
{
    if(as <= bs && bs <= ae)
        return true;
    if(bs <= as && as <= be)
        return true;
    return false;
}
#endif

__attribute__((noreturn)) void power_off(void) {
    uint8_t val = MAX77620_ONOFFCNFG1_PWR_OFF;
    i2c_send(I2C_5, MAX77620_PWR_I2C_ADDR, MAX77620_REG_ONOFFCNFG1, &val, 1);
    while (1);
}

__attribute__((noreturn)) void power_off_reset(void) {
    // Enable/Disable soft reset wake event.
    uint32_t reg;
    i2c_query(I2C_5, MAX77620_PWR_I2C_ADDR, MAX77620_REG_ONOFFCNFG2, &reg, 1);
    // Do not wake up after power off.
    reg &= ~(MAX77620_ONOFFCNFG2_SFT_RST_WK | MAX77620_ONOFFCNFG2_WK_ALARM1 | MAX77620_ONOFFCNFG2_WK_ALARM2);
    i2c_send(I2C_5, MAX77620_PWR_I2C_ADDR, MAX77620_REG_ONOFFCNFG2, &reg, 1);

    power_off();
}

#define MAX77620_RTC_UPDATE0_REG    0x04
#define  MAX77620_RTC_WRITE_UPDATE  BIT(0)
#define  MAX77620_RTC_READ_UPDATE   BIT(4)
#define MAX77620_RTC_NR_TIME_REGS   7
#define MAX77620_ALARM1_SEC_REG     0x0E
#define  MAX77620_RTC_ALARM_EN_MASK BIT(7)

void max77620_rtc_stop_alarm(void)
{
    uint8_t val = MAX77620_RTC_READ_UPDATE;

    // Update RTC regs from RTC clock.
    i2c_send(I2C_5, MAX77620_RTC_I2C_ADDR, MAX77620_RTC_UPDATE0_REG, &val, 1);

    // Stop alarm for both ALARM1 and ALARM2. Horizon uses ALARM2.
    for (int i = 0; i < (MAX77620_RTC_NR_TIME_REGS * 2); i++)
    {
        i2c_query(I2C_5, MAX77620_RTC_I2C_ADDR, MAX77620_ALARM1_SEC_REG + i, &val, 1);
        val &= ~MAX77620_RTC_ALARM_EN_MASK;
        i2c_send(I2C_5, MAX77620_RTC_I2C_ADDR, MAX77620_ALARM1_SEC_REG + i, &val, 1);
    }

    // Update RTC clock from RTC regs.
    val = MAX77620_RTC_WRITE_UPDATE;
    i2c_send(I2C_5, MAX77620_RTC_I2C_ADDR, MAX77620_RTC_UPDATE0_REG, &val, 1);
}

void autohosoff(void)
{
    uint32_t hosWakeup;
    i2c_query(I2C_5, MAX77620_PWR_I2C_ADDR, MAX77620_REG_IRQTOP, &hosWakeup, 1);
    if (hosWakeup & MAX77620_IRQ_TOP_RTC_MASK)
    {
        unmount_sd();
        max77620_rtc_stop_alarm();
        power_off_reset();
    }
}

#define SE_CTX_SAVE_AUTO ((volatile uint32_t) 0x70012074)
#define  SE_CTX_SAVE_AUTO_ENABLE    (1 << 0)
#define  SE_CTX_SAVE_AUTO_LOCK      (1 << 8)
 
int is_mariko()
{
  return fuse_get_reserved_odm(4) << 12 >> 28 || ((SE_CTX_SAVE_AUTO & (SE_CTX_SAVE_AUTO_ENABLE | SE_CTX_SAVE_AUTO_LOCK)) == (SE_CTX_SAVE_AUTO_ENABLE | SE_CTX_SAVE_AUTO_LOCK));
}
