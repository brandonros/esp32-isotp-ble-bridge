#include "math.h"

void write_uint32_be(uint32_t value, uint8_t *output) {
  output[3] = value & 0xFF;
  output[2] = (value >> 8) & 0xFF;
  output[1] = (value >> 16) & 0xFF;
  output[0] = (value >> 24) & 0xFF;
}

uint32_t read_uint32_be(const uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}

uint16_t read_uint16_be(const uint8_t *data) {
  return data[1] | (data[0] << 8);
}
