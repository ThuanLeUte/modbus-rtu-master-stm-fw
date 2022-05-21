#ifndef __WORD_H_
#define __WORD_H_

#include "mcu_application.h"

/** @ingroup util_word
    Return low word of a 32-bit integer.

    @param uint32_t ww (0x00000000..0xFFFFFFFF)
    @return low word of input (0x0000..0xFFFF)
*/
static inline uint16_t lowWord(uint32_t ww)
{
  return (uint16_t) ((ww) & 0xFFFF);
}


/** @ingroup util_word
    Return high word of a 32-bit integer.

    @param uint32_t ww (0x00000000..0xFFFFFFFF)
    @return high word of input (0x0000..0xFFFF)
*/
static inline uint16_t highWord(uint32_t ww)
{
  return (uint16_t) ((ww) >> 16);
}


static inline uint8_t lowByte(uint16_t ww)
{
  return (uint8_t) ((ww) & 0x00FF);
}


static inline uint8_t highByte(uint16_t ww)
{
  return (uint8_t) ((ww) >> 8);
}

static inline uint16_t word(uint8_t H_Byte,uint8_t L_Byte)
{
	uint16_t word;
	word = (uint16_t)(H_Byte<<8);
	word = word + L_Byte;
  return word;
}

#define bitSet(value, bit)  ((value) |= (1UL << (bit))) 
#define bitClear(value, bit)  ((value) &= ~(1UL << (bit)))

#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitRead(value, bit)  (((value) >> (bit)) & 0x01) 

#endif // __WORD_H_
