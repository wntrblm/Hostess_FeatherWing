#pragma once

#include <stdint.h>
#include <stdio.h>

inline void print_bytes(uint8_t *bytes, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%2x ", bytes[i]);
    }
    printf("\r\n");
}