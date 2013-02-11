/*
 * Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of libmc1322x: see http://mc1322x.devl.org
 * for details.
 *
 *
 */

#include <mc1322x.h>
#include <board.h>
#include <stdio.h>

#include "config.h"
#include "adc.h"

#define ADC_IRQ_FIFO_STATUS (1 << 15)

#define ADC_CH0  (1 << 0)
#define ADC_CH1  (1 << 1)
#define ADC_CH2  (1 << 2)
#define ADC_CH3  (1 << 3)
#define ADC_CH4  (1 << 4)
#define ADC_CH5  (1 << 5)
#define ADC_CH6  (1 << 6)
#define ADC_CH7  (1 << 7)
#define ADC_CH8  (1 << 8)
#define ADC_BATT (1 << 8)

#define SELECT_CHAN (ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3 | ADC_CH4)

#define NUM_CHAN_MAX 10
#define EXTRA_CHAN    2 // counter and timestamp
uint8_t NUM_CHAN;


uint16_t ring_buf[32 * 1024];

#define NUM_RECS (sizeof(ring_buf) / (NUM_CHAN * sizeof(uint16_t)))

volatile uint16_t head = 0, tail = 0;


volatile uint16_t isr_count = 0;
volatile uint16_t status1 = 0, status2 = 0;
volatile uint16_t status_last = 0;

uint16_t ring_used(void) {
	uint16_t latch_head, latch_tail;
	latch_head = head;
	latch_tail = tail;
	if (latch_head >= latch_tail) return latch_head - latch_tail;
	else return NUM_RECS + latch_head - latch_tail;
}

uint16_t ring_free(void) {
	return NUM_RECS - 1 - ring_used();
}

void daq_init(void) {
	uint8_t n;
	NUM_CHAN = EXTRA_CHAN;
	for (n=0; n<=8; n++)
		if (SELECT_CHAN & (1 << n)) NUM_CHAN++;
}

void adc_isr(void) {
	uint16_t value, value2;
	uint16_t cnt;
	
	//cnt = CRM->RTC_COUNT;
	cnt = *TMR0_CNTR;
	
	value = ADC->IRQ;
	
	value2 = ADC->FIFO_STATUS;
	value2 = ADC->TRIGGERS;
	
	if (value & ADC_IRQ_FIFO_STATUS) {
		// A FIFO level interrupt has occurred.
		
	//printf("ADC->IRQ = %04x, ", value);
	/*
	switch(isr_count) {
	case 0: status1 = value; break;
	case 1: status2 = value; break;
	default: status_last = value; break;
	}
	*/
	//adc_service();
	
	
	if (ring_free() > 0) {
	//if (next_head != tail) {
		uint16_t value;
		uint8_t channel;
		ring_buf[head * NUM_CHAN + 0] = isr_count;
		ring_buf[head * NUM_CHAN + 1] = cnt;
		while (ADC->FIFO_STATUSbits.EMPTY == 0) {
			value = ADC->FIFO_READ;
			channel = value >> 12;
			//if (channel < NUM_ADC_CHAN) adc_reading[channel] = value & 0xFFF;
			if (channel + EXTRA_CHAN >= NUM_CHAN) continue;
			ring_buf[head * NUM_CHAN + channel + EXTRA_CHAN] = value & 0xFFF;
		}
		head = (head + 1) % NUM_RECS;
	} else {
		ADC_flush();
	}
	
	} // End ADC_IRQ_FIFO_STATUS
	
	//ADC->IRQ = value; // clear pending interrupts.
	isr_count++;
	//adc_init();

	//ADC->CONTROL ^= 0x0001;
	//ADC->CONTROL ^= 0x0001;
	
	ADC->IRQ = 0xF000; // clear pending interrupts.
}

int main(void)
{
	uint8_t c;
	int flag = 0;

#define COUNT_MODE 1      /* use rising edge of primary source */
#define PRIME_SRC  0xf    /* Perip. clock with 128 prescale (for 24Mhz = 187500Hz)*/
#define SEC_SRC    0      /* don't need this */
#define ONCE       0      /* keep counting */
#define LEN        1      /* count until compare then reload with value in LOAD */
#define DIR        0      /* count up */
#define CO_INIT    0      /* other counters cannot force a re-initialization of this counter */
#define OUT_MODE   0      /* OFLAG is asserted while counter is active */

	*TMR_ENBL = 0;                     /* tmrs reset to enabled */
	*TMR0_SCTRL = 0;
	*TMR0_LOAD = 0;                    /* reload to zero */
	*TMR0_COMP_UP = 0xFFFF;             /* trigger a reload at the end */
	*TMR0_CMPLD1 = 0xFFFF;              /* compare 1 triggered reload level, 10HZ maybe? */
	*TMR0_CNTR = 0;                    /* reset count register */
	*TMR0_CTRL = (COUNT_MODE<<13) | (PRIME_SRC<<9) | (SEC_SRC<<7) | (ONCE<<6) | (LEN<<5) | (DIR<<4) | (CO_INIT<<3) | (OUT_MODE);
	*TMR_ENBL = 0xf;                   /* enable all the timers --- why not? */
	
	trim_xtal();
	uart_init(UART1, 115200);
	
	daq_init();
	
	adc_init();
	
	ADC->FIFO_CONTROL = NUM_CHAN - EXTRA_CHAN + 1; // Why do we need to add one here. Is the documentation wrong?
	
	ADC_flush();
	enable_irq(ADC);
/*
	printf("adc test\r\n");
	
	
	#define dump(x) printf("&%14s = %p\r\n", #x, &(ADC->x))

dump(COMP[0]);
dump(COMP[1]);
dump(COMP[2]);
dump(COMP[3]);
dump(COMP[4]);
dump(COMP[5]);
dump(COMP[6]);
dump(COMP[7]);
dump(BAT_COMP_OVER);
dump(BAT_COMP_UNDER);
dump(SEQ_1);
dump(SEQ_2);
dump(CONTROL);
dump(TRIGGERS);
dump(PRESCALE);
dump(reserved1);
dump(FIFO_READ);
dump(FIFO_CONTROL);
dump(FIFO_STATUS);
dump(reserved2[0]);
dump(reserved2[1]);
dump(reserved2[2]);
dump(reserved2[3]);
dump(reserved2[4]);
dump(SR_1_HIGH);
dump(SR_1_LOW);
dump(SR_2_HIGH);
dump(SR_2_LOW);
dump(ON_TIME);
dump(CONVERT_TIME);
dump(CLOCK_DIVIDER);
dump(reserved3);
dump(OVERRIDE);
dump(IRQ);
dump(MODE);
dump(RESULT_1);
dump(RESULT_2);

	
	printf("\x1B[2J"); // clear screen
*/

	for (c=0; c<=8; c++) {
		if (SELECT_CHAN & (1 << c))
			adc_setup_chan(c);
	}

	//for(;;);
/*
	for(;;) {
		printf("\x1B[H"); // cursor home
		printf("# Value\r\n");
		for (c=0; c<NUM_ADC_CHAN; c++) {
			//adc_service();
			printf("%u %04u %04u mV\r\n", c, adc_reading[c], adc_voltage(c));
		}
		printf("vbatt: %04u mV\r\n", adc_vbatt);
		printf("isr_count = %05u\r\n", isr_count); 
		printf("ADC->IRQ = %04x\r\n", ADC->IRQ);
		//ADC->IRQ |= 0xF000;
		//ADC->IRQ &= 0x0FFF;
		printf("ADC->SEQ_1 = %04x\r\n", ADC->SEQ_1);
		printf("ADC->CONTROL = %04x\r\n", ADC->CONTROL);
		printf("ADC->MODE = %04x\r\n", ADC->MODE);
		printf("status1 = %04x\r\n", status1);
		printf("status2 = %04x\r\n", status2);
		printf("status_last = %04x\r\n", status_last);
		enable_irq(ADC);
	}
*/
	for(;;) {
		//if (tail == head) continue;
		while(ring_used() > 0) {
			for (c=0; c<NUM_CHAN; c++) {
				printf("%5u,", ring_buf[tail * NUM_CHAN + c]);
			}
			tail = (tail + 1) % NUM_RECS;
			printf("\r\n");
		}
		/*
		if (flag < 1) {
			printf("head=%05u, tail=%05u,isr_count=%05u\r\n", head, tail, isr_count);
			flag++;
		}*/
	}
}
