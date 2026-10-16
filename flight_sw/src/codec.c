/*
 * codec.c - CRC-16/CCITT-FALSE and a bounds-checked big-endian cursor.
 */
#include <string.h>
#include "codec.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

void cur_init(cursor_t *c, uint8_t *buf, size_t cap)
{
    c->buf = buf; c->cap = cap; c->pos = 0; c->error = 0;
}

size_t cur_remaining(const cursor_t *c) { return c->cap - c->pos; }

static int need(cursor_t *c, size_t n)
{
    if (c->error || c->pos + n > c->cap) { c->error = 1; return 0; }
    return 1;
}

void put_u8(cursor_t *c, uint8_t v)   { if (need(c, 1)) c->buf[c->pos++] = v; }
void put_u16(cursor_t *c, uint16_t v) { put_u8(c, (uint8_t)(v >> 8)); put_u8(c, (uint8_t)v); }
void put_u32(cursor_t *c, uint32_t v) { put_u16(c, (uint16_t)(v >> 16)); put_u16(c, (uint16_t)v); }
void put_i16(cursor_t *c, int16_t v)  { put_u16(c, (uint16_t)v); }
void put_f32(cursor_t *c, float v)
{
    uint32_t u; memcpy(&u, &v, 4);          /* bit-exact, no aliasing tricks */
    put_u32(c, u);
}
void put_bytes(cursor_t *c, const uint8_t *src, size_t n)
{
    if (n == 0) return;                      /* memcpy(NULL, 0) is undefined in C */
    if (need(c, n)) { memcpy(c->buf + c->pos, src, n); c->pos += n; }
}

uint8_t get_u8(cursor_t *c)   { return need(c, 1) ? c->buf[c->pos++] : 0; }
uint16_t get_u16(cursor_t *c) { uint16_t h = get_u8(c); return (uint16_t)((h << 8) | get_u8(c)); }
uint32_t get_u32(cursor_t *c) { uint32_t h = get_u16(c); return (h << 16) | get_u16(c); }
int16_t get_i16(cursor_t *c)  { return (int16_t)get_u16(c); }
float get_f32(cursor_t *c)    { uint32_t u = get_u32(c); float f; memcpy(&f, &u, 4); return f; }
void get_bytes(cursor_t *c, uint8_t *dst, size_t n)
{
    if (n == 0) return;
    if (need(c, n)) { memcpy(dst, c->buf + c->pos, n); c->pos += n; }
    else memset(dst, 0, n);
}
