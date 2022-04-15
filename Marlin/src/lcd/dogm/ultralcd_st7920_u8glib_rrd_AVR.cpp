/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

// NOTE - the HAL version of the rrd device uses a generic ST7920 device.  See
// the file u8g_dev_st7920_128x64_HAL.cpp for the HAL version.

// NOTE by kai: Feiyu's Screen may be based on UC1701X 
// link : http://jlxlcd.cn/UpFile/201432214278.pdf?msclkid=a149f8fabccc11ecb430781333831da7 or
// http://jlxlcd.cn/UpFile/2015922133914.pdf?msclkid=a1481f8cbccc11eca289857bea07ecfa

#include "../../inc/MarlinConfigPre.h"

#if !defined(U8G_HAL_LINKS) && ANY(__AVR__, ARDUINO_ARCH_STM32, ARDUINO_ARCH_ESP32)

#include "../../inc/MarlinConfig.h"

#if IS_U8GLIB_ST7920

#include "ultralcd_st7920_u8glib_rrd_AVR.h"

#ifndef ST7920_DELAY_1
#ifdef BOARD_ST7920_DELAY_1
#define ST7920_DELAY_1 DELAY_NS(BOARD_ST7920_DELAY_1)
#else
#define ST7920_DELAY_1 CPU_ST7920_DELAY_1
#endif
#endif
#ifndef ST7920_DELAY_2
#ifdef BOARD_ST7920_DELAY_2
#define ST7920_DELAY_2 DELAY_NS(BOARD_ST7920_DELAY_2)
#else
#define ST7920_DELAY_2 CPU_ST7920_DELAY_2
#endif
#endif
#ifndef ST7920_DELAY_3
#ifdef BOARD_ST7920_DELAY_3
#define ST7920_DELAY_3 DELAY_NS(BOARD_ST7920_DELAY_3)
#else
#define ST7920_DELAY_3 CPU_ST7920_DELAY_3
#endif
#endif

// Optimize this code with -O3
#pragma GCC optimize(3)

#ifdef ARDUINO_ARCH_STM32F1
#define ST7920_DAT(V) !!((V)&0x80)
#else
#define ST7920_DAT(V) ((V)&0x80)
#endif
#define ST7920_SND_BIT                          \
    do {                                        \
        WRITE(ST7920_CLK_PIN, LOW);             \
        ST7920_DELAY_1;                         \
        WRITE(ST7920_DAT_PIN, ST7920_DAT(val)); \
        ST7920_DELAY_2;                         \
        WRITE(ST7920_CLK_PIN, HIGH);            \
        ST7920_DELAY_3;                         \
        val <<= 1;                              \
    } while (0)

// Optimize this code with -O3
#pragma GCC optimize(3)

void ST7920_SWSPI_SND_8BIT(uint8_t val) {
    ST7920_SND_BIT;  // 1
    ST7920_SND_BIT;  // 2
    ST7920_SND_BIT;  // 3
    ST7920_SND_BIT;  // 4
    ST7920_SND_BIT;  // 5
    ST7920_SND_BIT;  // 6
    ST7920_SND_BIT;  // 7
    ST7920_SND_BIT;  // 8
}

void lcd_Raddress(uint8_t page, uint8_t column) {
    ST7920_CS();
    column = column;
    page = page;
    ST7920_SET_CMD();
    ST7920_WRITE_BYTE(0xb0 + page);

    ST7920_WRITE_BYTE(((column >> 4) & 0x0f) + 0x10);
    ST7920_WRITE_BYTE(column & 0x0f);
    ST7920_NCS();
}

uint8_t u8g_dev_rrd_st7920_128x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
    switch (msg) {
        case U8G_DEV_MSG_INIT: {
            OUT_WRITE(ST7920_CS_PIN, LOW);
            OUT_WRITE(ST7920_DAT_PIN, LOW);
            OUT_WRITE(ST7920_CLK_PIN, HIGH);
            OUT_WRITE(ST7920_DC_PIN, LOW);
            ST7920_NCS();
            u8g_Delay(2);
            ST7920_CS();
            ST7920_SET_CMD();
            ST7920_WRITE_BYTE(0xe2);  // software reset
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x2c);
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x2e);  
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x2f);  
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x23);  
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x81);
            u8g_Delay(5);
            ST7920_WRITE_BYTE(0x28);
            u8g_Delay(2);
            ST7920_WRITE_BYTE(0xa2);
            ST7920_WRITE_BYTE(0xc8);
            ST7920_WRITE_BYTE(0xa0);
            ST7920_WRITE_BYTE(0x40);
            ST7920_WRITE_BYTE(0xaf);

            ST7920_NCS();
        } break;

        case U8G_DEV_MSG_STOP:
            break;

        case U8G_DEV_MSG_PAGE_NEXT: {
            u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
            uint8_t y = pb->p.page_y0 / 8;
            auto ptr = (uint8_t *)pb->buf;
            if (y > 31) {
                y = 4;
            }
            for (uint16_t i = 0; i < 4; i++) {
                lcd_Raddress(y, 0);
                ST7920_CS();
                ST7920_SET_DAT();
                // 按字节切列
                for (uint8_t j = 0; j < 16; j++) {
                    uint16_t offset_base = j + (i << 7);
                    // 取每行的hor位，拼接出一个转置的字节并发送
                    for (uint8_t k = 0, hor = 0x80; k < 8; k++, hor >>= 1) {
                        uint8_t char_to_send = 0;
                        for (uint8_t l = 0, m = 0x01; l < 128; l += 16, m <<= 1) {
                            (ptr[l + offset_base] & hor) ? (char_to_send |= m) : 0;
                        }
                        ST7920_WRITE_BYTE(char_to_send);
                    }
                }
                ST7920_NCS();
                y++;
            }
        } break;
    }
#if PAGE_HEIGHT == 8
    return u8g_dev_pb8h1_base_fn(u8g, dev, msg, arg);
#elif PAGE_HEIGHT == 16
    return u8g_dev_pb16h1_base_fn(u8g, dev, msg, arg);
#else
    return u8g_dev_pb32h1_base_fn(u8g, dev, msg, arg);
#endif
}

uint8_t u8g_dev_st7920_128x64_rrd_buf[(LCD_PIXEL_WIDTH) * (PAGE_HEIGHT) / 8] U8G_NOCOMMON;
u8g_pb_t u8g_dev_st7920_128x64_rrd_pb = {{PAGE_HEIGHT, LCD_PIXEL_HEIGHT, 0, 0, 0}, LCD_PIXEL_WIDTH, u8g_dev_st7920_128x64_rrd_buf};
u8g_dev_t u8g_dev_st7920_128x64_rrd_sw_spi = {u8g_dev_rrd_st7920_128x64_fn, &u8g_dev_st7920_128x64_rrd_pb, &u8g_com_null_fn};

#pragma GCC reset_options

#if ENABLED(LIGHTWEIGHT_UI)
#include "../../HAL/shared/HAL_ST7920.h"
void ST7920_cs() { ST7920_CS(); }
void ST7920_ncs() { ST7920_NCS(); }
void ST7920_set_cmd() { ST7920_SET_CMD(); }
void ST7920_set_dat() { ST7920_SET_DAT(); }
void ST7920_write_byte(const uint8_t val) { ST7920_WRITE_BYTE(val); }
#endif

#endif  // IS_U8GLIB_ST7920
#endif  // !U8G_HAL_LINKS && (__AVR__ || ARDUINO_ARCH_STM32 ||
        // ARDUINO_ARCH_ESP32)
