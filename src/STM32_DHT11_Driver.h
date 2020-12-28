#ifndef STM32_DHT11_DRIVER_H
#define STM32_DHT11_DRIVER_H
#include <Arduino.h>

struct DHT11_Data{
	bool response_chk;
	uint32_t _response_time;
	int32_t data[40];
	int32_t comm_time;
	float temperature;
	float humidity;
};

void initDHT11(uint32_t dhtPin);
DHT11_Data readDHT11(uint32_t dhtPin);

void configureMicroTimer(uint32_t dhtPin);

void dht_pin_ISR(void);
void timer_overflow_ISR(void);

#endif
