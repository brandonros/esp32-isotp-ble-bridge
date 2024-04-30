#ifndef MATH_H
#define MATH_H

#include <stdint.h>

void write_uint32_be(uint32_t value, uint8_t *output);
uint32_t read_uint32_be(const uint8_t *data);
uint16_t read_uint16_be(const uint8_t *data);

#endif
