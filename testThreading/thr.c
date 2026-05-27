#include<stdio.h>
#include "trh.h"

//change size
void to_xy (coordinates* coord)
{
  int h = 100;
  coord->x = 0;
  coord->y = 0;

  for(int i = 0; i < 3; i++)
  {
    coord->x += h * (coord->mes[i] - 48);
    coord->y += h * (coord->mes[i+3] - 48);
    h /= 10;
  }
}
//change size
void coord_to_string(coord_ext* coord)
{
  int x = coord->x;
  int y = coord->y;
  coord->mes[6] = 48 + coord->obj;
  coord->mes[7] = 48 + coord->colour;
  for(int i = 0; i < 3; i++)
  {
    coord->mes[2-i] = (uint8_t)(x % 10) + 48;
    coord->mes[5-i] = (uint8_t)(y % 10) + 48;
    y /= 10;
    x = x/10;
  }
}

void* recv(void* coord)
{
  coordinates* c = (coordinates*) coord;
  while(1) 
  {
    printf("threadx: %d\n", c->x);
    if(uart_has_data(UART0))
    {
      for(int i = 0; i < 4; i++)
        c->size[i] = uart_recv(UART0);

      //printf("strlen1: %d\n", c->size[0]);
      for(int i = 0; i < c->size[0]; i++)
        c->mes[i] = uart_recv(UART0);

      to_xy(c);
      c->mes[c->size[0]] = '\0';

      printf("mes: %s\n", c->mes);
    }
    sleep_msec(1000);
  }
  return NULL;
}

void* send(void* coord)
{
  coord_ext* c = (coord_ext*) coord;
  while(1)
  {
    coord_to_string(c);
    uint8_t d = 8;
    uart_send(UART0, d);
    d = 0;
    for(int i = 0; i < 3; i++)
      uart_send(UART0, d);

    d = 8;
    //char s[9] = "12312340";
    for(int i = 0; i < d; i++)
      uart_send(UART0, c->mes[i]);

    sleep_msec(4000);//VITAL
  }
  return NULL;
}

void* afuckingfunc()
{
    printf("nothing\n");
    return NULL;
}