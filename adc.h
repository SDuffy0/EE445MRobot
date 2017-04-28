// ADC.h
// Runs on LM4F120/TM4C123
// Provide a function that can initialize Timer2 or software triggered interrupts
// SS3 conversions and request an interrupt when the conversion
// is complete.
// Daniel Valvano
// May 2, 2015

#define NUM_IR 4

void ADC0_SS2_4Channels_TimerTriggered_Init(uint32_t period);

void ADC_GetData(uint16_t data[NUM_IR]);


