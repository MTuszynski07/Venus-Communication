#include<string.h>
#include<stdlib.h>
#include <pthread.h>
#include "trh.h"

int main(void) {
  
  pynq_init();
  switchbox_set_pin(IO_AR0 ,SWB_UART0_RX);
  switchbox_set_pin(IO_AR1 ,SWB_UART0_TX);
  uart_init(UART0);
  uart_reset_fifos(UART0); 

  //char e[5][9] = {"12313430", "50050031", "70070030", "30030044", "60030045"}; //used for send
  pthread_t tr2, tr1;
  coord_ext* c_send = malloc(sizeof(coord_ext));
  c_send->x = 123;
  c_send->y = 123;
  c_send->obj = 4;
  c_send->colour = 0;

  coordinates* c_recv = malloc(sizeof(coordinates));
  c_recv->x = 0;

  pthread_create(&tr1, NULL, recv, (void*)c_recv);
  pthread_create(&tr2, NULL, send, (void*)c_send);
  
  pthread_detach(tr1);
  pthread_detach(tr2);
  while(1)
  {
    printf("x: %d\ny: %d\n", c_recv->x, c_recv->y);
    sleep_msec(2000);
  }

  printf("x: %d\ny: %d\n", c_recv->x, c_recv->y);

  free(c_send);
  free(c_recv);
  uart_destroy(UART0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
