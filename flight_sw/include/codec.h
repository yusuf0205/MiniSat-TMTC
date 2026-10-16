/*
 * codec.h - CRC-16 and big-endian encode/decode helpers.
 */
#ifndef CODEC_H
#define CODEC_H

#include <stdint.h>
#include <stddef.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
 * Check value: crc16_ccitt("123456789", 9) == 0x29B1. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* A cursor over a byte buffer. Every put/get moves 'pos' forward and
 * sets 'error' instead of running past 'cap' (no buffer overflows). */
typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   pos;
    int      error;
} cursor_t;

void     cur_init(cursor_t *c, uint8_t *buf, size_t cap);
void     put_u8 (cursor_t *c, uint8_t v);
void     put_u16(cursor_t *c, uint16_t v);
void     put_u32(cursor_t *c, uint32_t v);
void     put_i16(cursor_t *c, int16_t v);
void     put_f32(cursor_t *c, float v);
void     put_bytes(cursor_t *c, const uint8_t *src, size_t n);
uint8_t  get_u8 (cursor_t *c);
uint16_t get_u16(cursor_t *c);
uint32_t get_u32(cursor_t *c);
int16_t  get_i16(cursor_t *c);
float    get_f32(cursor_t *c);
void     get_bytes(cursor_t *c, uint8_t *dst, size_t n);
size_t   cur_remaining(const cursor_t *c);

#endif
