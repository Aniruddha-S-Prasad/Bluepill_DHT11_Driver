#include "STM32_DHT11_Driver.h"


HardwareTimer *microTimer;
uint32_t timerChannel;

uint32_t DHT11_data_buffer[45]; 
volatile uint32_t bitCounter = 0;

/**
 * \brief Reads the data from the DHT11 Sensor
 *
 * \param dhtPin The pin to which the one-wire the DHT11 sensor is connected
 * \return DHT11_Data struct containing temperature and humidity floats
 */
DHT11_Data readDHT11(uint32_t dhtPin) {
	DHT11_Data newData;
	// Initialize DHT11 communications
	pinMode(dhtPin, OUTPUT);
	digitalWrite(dhtPin, LOW);
	delay(20);
	pinMode(dhtPin, INPUT);
	
	bitCounter = 0;
	microTimer->resume();
	microTimer->setCount(0, MICROSEC_FORMAT);

	delayMicroseconds(4250);

	int32_t h_int = 0, h_frac = 0, t_int = 0, t_frac = 0, checksum = 0;
    newData.response_chk = true;
	for (uint32_t ctr = 0; ctr < 40; ctr++){

		newData.data[ctr] = DHT11_data_buffer[ctr + 2] - DHT11_data_buffer[ctr + 1];
		if ((newData.data[ctr] > 60) && (newData.data[ctr] < 100)){
			newData.data[ctr] = 0;
		}
		else if((newData.data[ctr] > 100) && (newData.data[ctr] < 135)){
			newData.data[ctr] = 1;
		}
        else {
            newData.response_chk = false;
        }

		if (ctr < 8){
			h_int |= newData.data[ctr]<<(7 - ctr); 
		}
		else if (ctr < 16){
			h_frac |= newData.data[ctr]<<(15 - ctr); 
		}
		else if (ctr < 24){
			t_int |= newData.data[ctr]<<(23 - ctr); 
		}
		else if (ctr < 32){
			t_frac |= newData.data[ctr]<<(31 - ctr); 
		}
		else if (ctr < 40){
			checksum |= newData.data[ctr]<<(39 - ctr); 
		}
	}

    newData.comm_time = DHT11_data_buffer[41];
    newData._response_time = DHT11_data_buffer[1];

    if ((newData._response_time > 160) || (newData._response_time < 100)){
        newData.response_chk = false;
    }
    

	if((h_int + h_frac + t_int + t_frac) != checksum){
		// Report Failed
		newData.response_chk = false;
	}

	newData.temperature = t_int + (float)t_frac*0.1;
	newData.humidity = h_int + (float)h_frac*0.1;

	return newData;
}

void dht_pin_ISR(void){
	DHT11_data_buffer[bitCounter] = microTimer->getCaptureCompare(timerChannel, MICROSEC_COMPARE_FORMAT);
	bitCounter++;
}

void timer_overflow_ISR(void){
	microTimer->pause();
	return;
}

void configureMicroTimer(uint32_t dhtPin){
	TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(dhtPin), PinMap_PWM);
	timerChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(dhtPin), PinMap_PWM));
	
	microTimer = new HardwareTimer(instance);
	microTimer->setMode(timerChannel, TIMER_INPUT_CAPTURE_FALLING, dhtPin);
	microTimer->setOverflow(4200, MICROSEC_FORMAT);
	microTimer->attachInterrupt(timerChannel, dht_pin_ISR);
	microTimer->attachInterrupt(timer_overflow_ISR);
	microTimer->resume();
}

void initDHT11(uint32_t dhtPin){
	pinMode(dhtPin, INPUT);
	configureMicroTimer(dhtPin);
	microTimer->pause();
}