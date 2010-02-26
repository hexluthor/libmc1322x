#ifndef CONFIG_H
#define CONFIG_H

/* Baud rate */
/* INC = 767; MOD = 9999 works: 115200 @ 24 MHz 16 bit sample */
#define INC 767
#define MOD 9999

/* use uart1 for console */
#define uart_init uart1_init

/* nvm-read */
#define READ_ADDR 0x1F000
#define READ_NBYTES 1024

/* nvm-write */
#define WRITE_NBYTES 8
#define WRITE_ADDR 0x1e000
#define WRITEVAL0 0xdeadbeef 
#define WRITEVAL1 0xdeadbeef

/* romimg */
#define DUMP_BASE 0x00000000
#define DUMP_LEN  0x00014000

#endif
