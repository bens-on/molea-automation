#ifndef COMPAT_PGMSPACE_H
#define COMPAT_PGMSPACE_H

#include <Arduino.h>
#include <string.h>

// Many non-AVR cores don't have <pgmspace.h>.
// FastCRC includes it anyway. This shim makes it compile.

#ifndef PROGMEM
  #define PROGMEM
#endif

#ifndef PGM_P
  typedef const char* PGM_P;
#endif

#ifndef pgm_read_byte
  #define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

#ifndef pgm_read_word
  #define pgm_read_word(addr) (*(const uint16_t *)(addr))
#endif

#ifndef pgm_read_dword
  #define pgm_read_dword(addr) (*(const uint32_t *)(addr))
#endif

#ifndef pgm_read_float
  #define pgm_read_float(addr) (*(const float *)(addr))
#endif

#ifndef memcpy_P
  #define memcpy_P(dest, src, n) memcpy((dest), (src), (n))
#endif

#ifndef strcpy_P
  #define strcpy_P(dest, src) strcpy((dest), (src))
#endif

#endif // COMPAT_PGMSPACE_H