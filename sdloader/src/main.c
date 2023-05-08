/*
 * Copyright (c) 2018-2020 Atmosphère-NX
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

#include "utils.h"
#include "hwinit.h"
#include "di.h"
#include "timers.h"
#include "fs_utils.h"
#include "chainloader.h"
#include "lib/fatfs/ff.h"
#include "lib/log.h"
#include "lib/vsprintf.h"
#include "display/video_fb.h"
#include "btn.h"
#include "fuse.h"
#include "sdram.h"
#include "sdmmc/mmc.h"

extern void (*__program_exit_callback)(int rc);

static void *g_framebuffer;

static sdmmc_t emmc_sdmmc;

static void setup_display(void) {
    sdram_init();

    g_framebuffer = (void *) 0xC0000000;

    /* Zero-fill the framebuffer and register it as printk provider. */
    video_init(g_framebuffer);

    /* Initialize the display. */
    display_init();

    /* Set the framebuffer. */
    display_init_framebuffer(g_framebuffer);
}

static void exit_callback(int rc) {
    if (rc == 0)
        relocate_and_chainload();
    power_off();
}

static int load_payload(const char *path) {
    FILINFO info;
    size_t size;

    /* Check if the binary is present. */
    if (f_stat(path, &info) != FR_OK) {
        //print(1, "Payload not found!\n");
        return -2;
    }

    size = (size_t)info.fsize;

    if (size > 0x1F000)
        return -3;

    /* Try to read the binary. */
    if (read_from_file((void *)0x40021000, size, path) != size) {
        //print(SCREEN_LOG_LEVEL_WARNING, "Failed to read payload (%s)!\n", path);
        return -2;
    }

    g_chainloader_entry.src_address  = 0x40021000;
    g_chainloader_entry.size         = size;

    return 0;
}

static void draw_square(int off_x, int off_y, int x, int y, int multi, int color)
{
    int start_x = off_x + (x * multi);
    int start_y = off_y + (y * multi);
    uint32_t *fb = (uint32_t *) g_framebuffer;
#define fb_coord(x, y) (fb[(y) + (1280 - (x)) * (720 + 48)])
    
    for (int i = 0; i < multi; i++)
        for (int j = 0; j < multi; j++)
            if (start_y + j > 0 && start_y + j < 720)
                fb_coord(start_x + i, start_y + j) = color;
}

static void draw_table(const char *msg, int off_x, int off_y, int size)
{    
    int x = 0;
    int y = 0;
    
    for (int i = 0; i < strlen(msg); i++)
    {
        if (msg[i] == '\n')
        {
            x = 0;
            ++y;
            continue;
        }
        
        if (msg[i] != '*')
        {
            int color = 0xFFFFFF;
            switch(msg[i])
            {
                // Only Black & White
                case ' ': color = 0x000000; break;
                case 'O': color = 0xFFFFFF; break;
            }
            draw_square(off_x, off_y, x, y, size, color);
        }
        
        ++x;
    }
}

const char *no_sd =
    "O   O OOOOO  OOOOO OOOO \n"
    "OO  O O   O  O     O   O\n"
    "O O O O   O  OOOOO O   O\n"
    "O  OO O   O      O O   O\n"
    "O   O OOOOO  OOOOO OOOO \n"
;

const char *big_bin =
    "OOOO  OOO OOOOO  OOOO  OOO O   O\n"
    "O   O  O  O      O   O  O  OO  O\n"
    "OOOO   O  O  OO  OOOO   O  O O O\n"
    "O   O  O  O   O  O   O  O  O  OO\n"
    "OOOO  OOO OOOOO  OOOO  OOO O   O\n"
;

const char *no_bin =
    "O   O OOOOO  OOOO  OOO O   O\n"
    "OO  O O   O  O   O  O  OO  O\n"
    "O O O O   O  OOOO   O  O O O\n"
    "O  OO O   O  O   O  O  O  OO\n"
    "O   O OOOOO  OOOO  OOO O   O\n"
;

static void modchip_send(sdmmc_t *sdmmc, uint8_t *buf)
{
    sdmmc_command_t cmd = {};
    sdmmc_request_t req = {};

    cmd.opcode = MMC_GO_IDLE_STATE;
    cmd.arg = 0xAA5458BA;
    cmd.flags = SDMMC_RSP_R1;
    
    req.data = buf;
    req.blksz = 512;
    req.num_blocks = 1;
    req.is_read = false;
    req.is_multi_block = false;
    req.is_auto_cmd12 = false;
    
    sdmmc_send_cmd(sdmmc, &cmd, &req, 0);
}

int main(void) {
    log_set_log_level(SCREEN_LOG_LEVEL_NONE);

    int ret = 0;
	int mcnosleep = 0;
    uint8_t modchip_buf[512];

    nx_hwinit();

    fuse_init();

    sdmmc_init(&emmc_sdmmc, SDMMC_4, SDMMC_VOLTAGE_1V8, SDMMC_BUS_WIDTH_1BIT, SDMMC_SPEED_MMC_IDENT);
	
	uint32_t btn = btn_read();

    /* First sleep command tells chip glitch success */	
	modchip_buf[0] = 0x55;
	modchip_send(&emmc_sdmmc, modchip_buf);

    if (!mount_sd())
    {
        mdelay(500);
        if (!mount_sd())
            ret = -1;
    }

    /* Boot to OFW (Normal) if VOL_UP and VOL_DOWN is pressed */

    if (btn & BTN_VOL_UP && btn & BTN_VOL_DOWN)
    {
        ret = 1;
    }

    /* keep chip awake if VOL_UP is pressed */
    if (btn & BTN_VOL_UP && !(btn & BTN_VOL_DOWN))
    {
        mcnosleep = 1;
    }

    /* send sleep command */
	if (mcnosleep == 0)
	{
	modchip_buf[0] = 0x55;
	modchip_send(&emmc_sdmmc, modchip_buf);
	}
	
    autohosoff();

    if (ret == 0)
    {
		ret = load_payload("payload.bin");
    }

    if (ret != 0)
    {
        setup_display();

        memset(g_framebuffer, 0x00, (720 + 48) * 1280 * 4);

        if (ret == -1)
            draw_table(no_sd, 50, 50, 50);
        else if (ret == -2)
            draw_table(no_bin, 52, 52, 42);
        else if (ret == -3)
            draw_table(big_bin, 48, 48, 37);
        else if (ret == 1)
        {
            sdmmc_finish(&emmc_sdmmc);
            unmount_sd();
            panic(0x21); // Bypass fuse programming in package1.
        }

        display_backlight(true);

        while (true)
        {
            uint32_t btn = btn_read();
            if (btn & BTN_POWER)
                break;
			if (btn & BTN_VOL_UP && btn & BTN_VOL_DOWN)
			{
				sdmmc_finish(&emmc_sdmmc);
				unmount_sd();
				panic(0x21); // Bypass fuse programming in package1.
			}
		}

        display_backlight(false);

        display_end();
    }
    
    sdmmc_finish(&emmc_sdmmc);
    unmount_sd();

    __program_exit_callback = exit_callback;
    return ret;
}