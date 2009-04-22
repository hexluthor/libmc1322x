#ifndef __SYS_INTERRUPT_H
#define __SYS_INTERRUPT_H

/* Returns true if it handled an active interrupt */
typedef int (*SystemInterruptFunc)();

typedef struct _SystemInterruptHandler SystemInterruptHandler;
struct _SystemInterruptHandler
{
  SystemInterruptHandler *next;
  SystemInterruptFunc handler;
};

void
sys_interrupt_enable();

void
sys_interrupt_disable();

void
sys_interrupt_append_handler(SystemInterruptHandler *handler);

void
sys_interrupt_prepend_handler(SystemInterruptHandler *handler);

void
sys_interrupt_remove_handler(SystemInterruptHandler *handler);

#endif /* __SYS_INTERRUPT_H */
