#include <Arduino.h>
#include "STM32_DHT11_Driver.h"

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);

	initDHT11(PB1);
	
	Serial1.begin(115200);
	delay(5000);
	Serial1.printf("Starting\n");
	delay(1000);
}

void loop() {
	DHT11_Data senseData;
	senseData = readDHT11(PB1);
	// Serial1.printf("Response: %s,\tTemperature: %f\tHumidity: %f\tComm Time: %d\n", senseData.response_chk ? "OK":"Failed", senseData.temperature, senseData.humidity, senseData.comm_time);
	Serial1.print("Response: ");
	Serial1.print(senseData.response_chk ? "OK":"Failed");
	Serial1.print("\tT: ");
	Serial1.print(senseData.temperature);
	Serial1.print("\tH: ");
	Serial1.print(senseData.humidity);
	Serial1.print("\tComm Time: ");
	Serial1.println(senseData.comm_time);
	delay(2000);
}