#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include "system.h"
#include "sys/alt_irq.h"
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_timer_regs.h"


int main()
{
  printf("Terminal start\n");

  FILE* ser = fopen("/dev/uart_0", "w");
  if(ser){
      printf("Opened UART\n");
  } else {
	  printf("Failed to open UART\n");
      while (1);
  }

  int count = 0;


  while(1) {

	  if (count < 1000){
		  char test[] = "worked";
		  printf("%d", sizeof(test));
		  fwrite(test, 1, sizeof(), ser);
		  count++;
	  } else {
		  count = 0;

	  }

  }


  fclose(ser);

  return 0;
}
