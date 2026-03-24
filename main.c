/**
 * Copyright (C) 2021, Mate Kukri
 * Based on "pico-serprog" by Thomas Roth
 *
 * Licensed under GPLv3
 *
 * Also based on stm32-vserprog:
 * https://github.com/dword1511/stm32-vserprog
 */

#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "tusb.h"
#include "serprog.h"

#define CDC_ITF 0
#define SPI_IF spi0
#define SPI_BAUD 10000000
#define SPI_CS 5
#define SPI_MISO 4
#define SPI_MOSI 3
#define SPI_SCK 2

/*
 * Conservative but much faster than the original 32-byte path.
 * 4096 bytes keeps RAM use reasonable while reducing protocol overhead.
 */
#define MAX_BUFFER_SIZE 4096
#define MAX_OPBUF_SIZE 4096
#define SERIAL_BUFFER_SIZE 0xFFFF

static uint8_t tx_buffer[MAX_BUFFER_SIZE];
static uint8_t rx_buffer[MAX_BUFFER_SIZE];
static uint8_t opbuf[MAX_OPBUF_SIZE];
static uint32_t opbuf_pos = 0;
static bool outputs_enabled = true;

static void enable_spi(uint baud)
{
    gpio_init(SPI_CS);
    gpio_put(SPI_CS, 1);
    gpio_set_dir(SPI_CS, GPIO_OUT);

    spi_init(SPI_IF, baud);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
}

static void disable_spi(void)
{
    gpio_init(SPI_CS);
    gpio_init(SPI_MISO);
    gpio_init(SPI_MOSI);
    gpio_init(SPI_SCK);

    gpio_set_pulls(SPI_CS, 0, 0);
    gpio_set_pulls(SPI_MISO, 0, 0);
    gpio_set_pulls(SPI_MOSI, 0, 0);
    gpio_set_pulls(SPI_SCK, 0, 0);

    spi_deinit(SPI_IF);
}

static inline void set_output_state(bool enabled)
{
    outputs_enabled = enabled;
    if (enabled) {
        enable_spi(spi_get_baudrate(SPI_IF));
    } else {
        disable_spi();
    }
}

static inline void cs_select(uint cs_pin)
{
    sleep_us(1);
    gpio_put(cs_pin, 0);
    sleep_us(1);
}

static inline void cs_deselect(uint cs_pin)
{
    sleep_us(1);
    gpio_put(cs_pin, 1);
    sleep_us(1);
}

static void wait_for_read(void)
{
    do {
        tud_task();
    } while (!tud_cdc_n_available(CDC_ITF));
}

static inline void readbytes_blocking(void *buf, uint32_t len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len) {
        wait_for_read();
        uint32_t r = tud_cdc_n_read(CDC_ITF, p, len);
        p += r;
        len -= r;
    }
}

static inline uint8_t readbyte_blocking(void)
{
    uint8_t b;
    readbytes_blocking(&b, 1);
    return b;
}

static void wait_for_write(void)
{
    do {
        tud_task();
    } while (!tud_cdc_n_write_available(CDC_ITF));
}

static inline void sendbytes_blocking(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len) {
        wait_for_write();
        uint32_t avail = tud_cdc_n_write_available(CDC_ITF);
        uint32_t chunk = len < avail ? len : avail;
        if (!chunk) {
            tud_task();
            continue;
        }
        uint32_t w = tud_cdc_n_write(CDC_ITF, p, chunk);
        p += w;
        len -= w;
        tud_cdc_n_write_flush(CDC_ITF);
    }
}

static inline void sendbyte_blocking(uint8_t b)
{
    sendbytes_blocking(&b, 1);
}

static inline void send_u16_le(uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
    sendbytes_blocking(b, sizeof(b));
}

static inline void send_u24_le(uint32_t v)
{
    uint8_t b[3] = {
        (uint8_t)(v & 0xFF),
        (uint8_t)((v >> 8) & 0xFF),
        (uint8_t)((v >> 16) & 0xFF)
    };
    sendbytes_blocking(b, sizeof(b));
}

static void command_loop(void)
{
    uint baud = spi_get_baudrate(SPI_IF);

    for (;;) {
        switch (readbyte_blocking()) {
        case S_CMD_NOP:
            sendbyte_blocking(S_ACK);
            break;

        case S_CMD_Q_IFACE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking(0x01);
            sendbyte_blocking(0x00);
            break;

        case S_CMD_Q_SERBUF:
            sendbyte_blocking(S_ACK);
            send_u16_le(SERIAL_BUFFER_SIZE);
            break;

        case S_CMD_Q_OPBUF:
            sendbyte_blocking(S_ACK);
            send_u16_le(MAX_OPBUF_SIZE);
            break;

        case S_CMD_Q_RDNMAXLEN:
        case S_CMD_Q_WRNMAXLEN:
            sendbyte_blocking(S_ACK);
            send_u24_le(MAX_BUFFER_SIZE);
            break;

        case S_CMD_Q_CMDMAP: {
            static const uint32_t cmdmap[8] = {
                (1u << S_CMD_NOP) |
                (1u << S_CMD_Q_IFACE) |
                (1u << S_CMD_Q_CMDMAP) |
                (1u << S_CMD_Q_PGMNAME) |
                (1u << S_CMD_Q_SERBUF) |
                (1u << S_CMD_Q_BUSTYPE) |
                (1u << S_CMD_Q_OPBUF),
                (1u << (S_CMD_Q_WRNMAXLEN - 32)) |
                (1u << (S_CMD_R_BYTE - 32)) |
                (1u << (S_CMD_R_NBYTES - 32)) |
                (1u << (S_CMD_O_INIT - 32)) |
                (1u << (S_CMD_O_WRITEB - 32)) |
                (1u << (S_CMD_O_EXEC - 32)) |
                (1u << (S_CMD_SYNCNOP - 32)) |
                (1u << (S_CMD_Q_RDNMAXLEN - 32)),
                (1u << (S_CMD_S_BUSTYPE - 64)) |
                (1u << (S_CMD_O_SPIOP - 64)) |
                (1u << (S_CMD_S_SPI_FREQ - 64)) |
                (1u << (S_CMD_S_PIN_STATE - 64)),
                0, 0, 0, 0, 0
            };
            sendbyte_blocking(S_ACK);
            sendbytes_blocking(cmdmap, sizeof(cmdmap));
            break;
        }

        case S_CMD_Q_PGMNAME: {
            static const char progname[16] = "pico-serprog";
            sendbyte_blocking(S_ACK);
            sendbytes_blocking(progname, sizeof(progname));
            break;
        }

        case S_CMD_Q_BUSTYPE:
            sendbyte_blocking(S_ACK);
            sendbyte_blocking((1 << 3));
            break;

        case S_CMD_SYNCNOP:
            sendbyte_blocking(S_NAK);
            sendbyte_blocking(S_ACK);
            break;

        case S_CMD_S_BUSTYPE:
            if (readbyte_blocking() & (1 << 3))
                sendbyte_blocking(S_ACK);
            else
                sendbyte_blocking(S_NAK);
            break;

        case S_CMD_S_PIN_STATE:
            if (readbyte_blocking())
                set_output_state(true);
            else
                set_output_state(false);
            sendbyte_blocking(S_ACK);
            break;

        case S_CMD_O_SPIOP: {
            uint32_t slen = 0, rlen = 0;
            readbytes_blocking(&slen, 3);
            readbytes_blocking(&rlen, 3);
            slen &= 0x00FFFFFFu;
            rlen &= 0x00FFFFFFu;

            if (!outputs_enabled) {
                sendbyte_blocking(S_NAK);
                break;
            }

            if (slen > MAX_BUFFER_SIZE) {
                sendbyte_blocking(S_NAK);
                break;
            }

            if (slen) {
                readbytes_blocking(tx_buffer, slen);
            }

            cs_select(SPI_CS);
            if (slen) {
                spi_write_blocking(SPI_IF, tx_buffer, slen);
            }

            sendbyte_blocking(S_ACK);

            while (rlen) {
                uint32_t chunk = rlen > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : rlen;
                spi_read_blocking(SPI_IF, 0, rx_buffer, chunk);
                sendbytes_blocking(rx_buffer, chunk);
                rlen -= chunk;
            }

            cs_deselect(SPI_CS);
            break;
        }

        case S_CMD_S_SPI_FREQ: {
            uint32_t want_baud;
            readbytes_blocking(&want_baud, 4);
            if (!want_baud) {
                sendbyte_blocking(S_NAK);
                break;
            }
            baud = spi_set_baudrate(SPI_IF, want_baud);
            sendbyte_blocking(S_ACK);
            sendbytes_blocking(&baud, 4);
            break;
        }

        case S_CMD_R_BYTE: {
            uint32_t addr = 0;
            uint8_t data;
            readbytes_blocking(&addr, 3);
            cs_select(SPI_CS);
            spi_write_blocking(SPI_IF, (uint8_t *)&addr, 3);
            spi_read_blocking(SPI_IF, 0, &data, 1);
            cs_deselect(SPI_CS);
            sendbyte_blocking(S_ACK);
            sendbyte_blocking(data);
            break;
        }

        case S_CMD_R_NBYTES: {
            uint32_t addr = 0, len = 0;
            readbytes_blocking(&addr, 3);
            readbytes_blocking(&len, 3);
            cs_select(SPI_CS);
            spi_write_blocking(SPI_IF, (uint8_t *)&addr, 3);
            sendbyte_blocking(S_ACK);
            while (len) {
                uint32_t chunk = len > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : len;
                spi_read_blocking(SPI_IF, 0, rx_buffer, chunk);
                sendbytes_blocking(rx_buffer, chunk);
                len -= chunk;
            }
            cs_deselect(SPI_CS);
            break;
        }

        case S_CMD_O_WRITEB: {
            uint32_t addr;
            uint8_t byte;
            if (opbuf_pos + 5 > MAX_OPBUF_SIZE) {
                sendbyte_blocking(S_NAK);
                break;
            }
            readbytes_blocking(&addr, 3);
            byte = readbyte_blocking();
            opbuf[opbuf_pos++] = S_CMD_O_WRITEB;
            memcpy(&opbuf[opbuf_pos], &addr, 3);
            opbuf_pos += 3;
            opbuf[opbuf_pos++] = byte;
            sendbyte_blocking(S_ACK);
            break;
        }

        case S_CMD_O_INIT:
            opbuf_pos = 0;
            memset(opbuf, 0, sizeof(opbuf));
            sendbyte_blocking(S_ACK);
            break;

        case S_CMD_O_EXEC: {
            uint32_t i = 0;
            if (opbuf_pos == 0) {
                sendbyte_blocking(S_NAK);
                break;
            }
            sendbyte_blocking(S_ACK);
            while (i < opbuf_pos) {
                uint8_t cmd = opbuf[i++];
                if (cmd == S_CMD_O_WRITEB) {
                    uint32_t addr;
                    uint8_t byte;
                    memcpy(&addr, &opbuf[i], 3);
                    i += 3;
                    byte = opbuf[i++];
                    cs_select(SPI_CS);
                    spi_write_blocking(SPI_IF, (uint8_t *)&addr, 3);
                    spi_write_blocking(SPI_IF, &byte, 1);
                    cs_deselect(SPI_CS);
                } else {
                    sendbyte_blocking(S_NAK);
                    break;
                }
            }
            opbuf_pos = 0;
            break;
        }

        default:
            sendbyte_blocking(S_NAK);
            break;
        }
    }
}

int main(void)
{
    tusb_init();
    enable_spi(SPI_BAUD);
    command_loop();
}
