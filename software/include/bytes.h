#ifndef _BYTES_H
#define _BYTES_H



#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)))
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


uint32_t  BytesToUINT32(uint8_t *bys);
uint16_t  BigBytesToUINT16(uint8_t *bys);
int16_t BigBytesToINT16(uint8_t *bys);
uint16_t BytesToUINT16(uint8_t *bys);
int16_t BytesToINT16(uint8_t *bys);
float BytesToFloat(uint8_t *bys);
#endif

