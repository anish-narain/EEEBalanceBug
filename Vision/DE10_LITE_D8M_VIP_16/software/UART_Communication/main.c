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

  FILE* ser = fopen("/dev/uart_0", "r+");
  if(ser){
      printf("Opened UART\n");
  } else {
	  printf("Failed to open UART\n");
      while (1);
  }

  while(1) {

	  char buf[20];
	  int val = 0;

	  if (fgets(buf, 20, ser) != NULL) {

		  printf("message received: %s\n", buf);

		  int isNum = 1;
		  for (int i=0; (i<20) & (buf[i] != '\n'); i++) {
			  isNum = isdigit(buf[i]);
		  }

		  printf("isNum =  %d\n", isNum);

		  if (isNum) {
			  printf(buf);
			  val = atoi(buf);
			  val++;
			  fprintf(ser, "%d\n", val);
		  }

	  }

  }


  fclose(ser);

  return 0;
}
