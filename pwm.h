#ifndef __PWM__
#define __PWM__

extern void pwm_init();

extern void pwm_set(unsigned freq, unsigned duty);

extern void pwm_get(unsigned *freq, unsigned *duty);

#endif //__PWM__
