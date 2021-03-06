#include "pico/stdlib.h"


	
const uint8_t picotext_width = 68;
const uint8_t picotext_height = 47;

const uint8_t picotext_bitmap [] = 
{
    // 'pico-text', 68x47px
    0xfe, 0xfe, 0x86, 0x86, 0x86, 0xce, 0xfc, 0x78, 0x00, 0x00, 0x00, 0x30, 0x30, 0xf3, 0xf3, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0xc0, 0xe0, 0x70, 0x30, 0x30, 0x70, 0x60, 0x40, 0x00, 0x00, 0xc0, 0xe0, 
    0x70, 0x30, 0x30, 0x70, 0xe0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x1f, 0x1f, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 
    0x18, 0x1f, 0x1f, 0x18, 0x18, 0x00, 0x00, 0x00, 0x07, 0x0f, 0x1c, 0x18, 0x18, 0x1c, 0x0c, 0x04, 
    0x00, 0x00, 0x07, 0x0f, 0x1c, 0x18, 0x18, 0x1c, 0x0f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf8, 0x1c, 0x0c, 0x0c, 0x1c, 0x18, 0x10, 
    0x00, 0x00, 0x80, 0xc0, 0xe0, 0x60, 0x60, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x60, 0x60, 0xe6, 
    0xe6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0x60, 0x60, 0xe0, 0xc0, 0x80, 0x00, 0x00, 
    0x80, 0xc0, 0xe0, 0x60, 0x60, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0xe0, 0xe0, 0xc0, 0xe0, 0x60, 0x60, 
    0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x38, 0x30, 
    0x33, 0x1b, 0x3f, 0x3f, 0x00, 0x00, 0x0f, 0x1f, 0x3e, 0x36, 0x36, 0x36, 0x17, 0x07, 0x00, 0x00, 
    0x00, 0x30, 0x30, 0x3f, 0x3f, 0x30, 0x30, 0x00, 0x00, 0x00, 0x0f, 0x1f, 0x38, 0x30, 0x30, 0x98, 
    0xff, 0xff, 0x00, 0x00, 0x0f, 0x1f, 0x3e, 0x36, 0x36, 0x36, 0x17, 0x07, 0x00, 0x00, 0x3f, 0x3f, 
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0xe0, 0xf0, 0x38, 0x18, 0x18, 0x38, 0x30, 0x20, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xc0, 0xc0, 0xc0, 
    0x80, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x00, 0x00, 0xc0, 0xc3, 
    0x83, 0xc3, 0xc3, 0xc3, 0x81, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf8, 0xf8, 0xc0, 0xc0, 0xc0, 0x00, 
    0x00, 0x00, 0x00, 0x80, 0xc0, 0xc0, 0xc0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x80, 0xc0, 
    0xc0, 0xc0, 0xc0, 0xc0, 0x1f, 0x3f, 0x70, 0x60, 0x60, 0x70, 0x30, 0x10, 0x00, 0x00, 0x1f, 0x3f, 
    0x71, 0x60, 0x60, 0x71, 0x3f, 0x1f, 0x00, 0x00, 0x1f, 0x3f, 0x70, 0x60, 0x60, 0x30, 0x7f, 0x7f, 
    0x00, 0x00, 0x7f, 0x7f, 0x01, 0x00, 0x00, 0x01, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x7f, 
    0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x1f, 0x3f, 0x7d, 0x6c, 0x6c, 0x6d, 0x2f, 0x0f, 0x00, 0x00, 
    0x7f, 0x7f, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00

};


const bitmap picotext = {(uint8_t *) picotext_bitmap, picotext_width, picotext_height};