#include <libpynq.h>
#ifndef trh_h
#define trh_h

typedef struct{
    int x;
    int y;
    char mes[7];
    uint8_t size[4];

} coordinates;

typedef struct{
    int x;
    int y;
    char mes[8];
    int obj;
    int colour;
} coord_ext;

void* recv(void* coord);

void* send(void* coord);

void coord_to_string(coord_ext* coord);

#endif