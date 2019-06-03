#ifndef __ADC_EX__
#define __ADC_EX__

extern void     adcs_ex_init();

extern void     adcs_ex_do_samples();
extern void     adcs_ex_second_boardary();

extern unsigned adcs_ex_get_count();

extern void     adcs_ex_adc_log(unsigned adc);
extern void     adcs_ex_log();

#endif //__ADC_EX__
