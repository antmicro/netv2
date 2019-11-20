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

#define RES_800x600x60 3
#define RES_1920x1080x60 11

int main(void)
{
	irq_setmask(0);
	irq_setie(1);
	uart_init();

	hdmi_out_setup(RES_800x600x60);
	hdmi_in_setup(RES_800x600x60);
	
	while(1) {
		hdmi_service();
	}

	return 0;
}
