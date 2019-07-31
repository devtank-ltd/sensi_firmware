#ifndef __ADC_EX__
#define __ADC_EX__

extern void     adcs_ex_init();

extern void     adcs_ex_do_samples();
extern void     adcs_ex_second_boardary();

extern unsigned adcs_ex_get_count();

extern void     adcs_ex_adc_log(unsigned adc);
extern void     adcs_ex_log();
extern int64_t adcs_ex_read_value(int32_t adc_offset);

#endif //__ADC_EX__
