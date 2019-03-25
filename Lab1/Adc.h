/*
 * Adc.h
 *
 *  Created on: Mar 23, 2019
 *      Author: doate
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

void AdcIsr(void);

#endif /* ADC_H_ */
