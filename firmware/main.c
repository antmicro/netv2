#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <uart.h>
#include <time.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include "flags.h"
#include <console.h>
#include <system.h>

#include "processor.h"


int main(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();

	hdmi_out_setup(3);
	hdmi_in_setup(3);
	
	while(1) {
		hdmi_service();
	}

	return 0;
}
