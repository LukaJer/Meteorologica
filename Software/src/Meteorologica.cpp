#include <RH_RF95.h>
#include <Arduino.h>
#include <RHEncryptedDriver.h>
#include <Speck.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <LaCrosse_TX23.h>
#include "STM32LowPower.h"

// General Pin Definitions
#define RFM95_RST PA3
#define RFM95_CS PA4
#define RFM95_INT PA5
#define MISO PA6
#define MOSI PA2
#define SCK PA1
#define I2C_SDA PA10
#define I2C_SCL PA9

#define F_CPU 16000000

// Uncomment used Sensors:
//#define WIND
//#define BME280
//#define RMTSENS
//#define SHT31
#define RAIN

// Radiohead RF95 Driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);			// Instanciate a LoRa driver
Speck myCipher;								// Instanciate a Speck block ciphering
RHEncryptedDriver myDriver(rf95, myCipher); // Instantiate the driver with those two

// Bosch BME280 Definition
#ifdef BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
float Temp, Hum, Press;

#endif // BME280

// Sensorion SHT31 Definition
#ifdef SHT31
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float Temp1, Hum1;
#endif // SHT31

// MaXIM DS18B20 Definition
#ifdef RMTSENS
#define RemoteTempPin PA15
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(RemoteTempPin);
DallasTemperature RemoteSensor(&oneWire);
#endif // RMTSENS

// LaCrosse TX23 Definition
#ifdef WIND
#define Wind_Pin PB8
LaCrosse_TX23 anemometer = LaCrosse_TX23(Wind_Pin);
float speed, speed_old;
int dir, dir_old;
#endif // WIND

// Generic Rain Interrupt Sensor Definition
#ifdef RAIN
#define rainPin PA0 // fixed!!
volatile uint8_t rainPulses = 0;
void rainfct();
#endif // RAIN

float frequency = 868.0;																// LoRa Frequency
unsigned char encryptkey[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // The very secret key
String name = "WSDev";																	// Name of the Weather Station
uint8_t HWMessageLen, nrWakeup = 0;
DynamicJsonDocument payload(512);			  // JSON Definition (might be shortened)
int SendInterval_LP = 600, SendInterval = 120; //  (Low Power) Send Intervall in Seconds
int ValChng = 0;							  // Tracks if values have changed
float VBatt, VSol;

void setup()
{
	// Set UART Pins
	Serial.setRx(PB7);
	Serial.setTx(PB6);
	Serial.begin(9600);

	payload["Name"] = name;
	Serial.println("WeatherLoRa Client: " + name);
	delay(100);

	// Set I2C Pins
	Wire.setSDA(I2C_SDA);
	Wire.setSCL(I2C_SCL);
	Wire.begin();

// Sensor Initalisation
#ifdef BME280
	unsigned status;
	status = bme.begin(0x76);
	if (!status)
	{
		Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
		Serial.print("SensorID was: 0x");
		Serial.println(bme.sensorID(), 16);
		Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
		Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
		Serial.print("        ID of 0x60 represents a BME 280.\n");
		Serial.print("        ID of 0x61 represents a BME 680.\n");
		while (1)
			delay(10);
	}
#endif // BME280

#ifdef SHT31
	if (!sht31.begin(0x44))
	{ // Set to 0x45 for alternate i2c addr
		Serial.println("Couldn't find SHT31");
		while (1)
			delay(1);
	}
#endif // SHT31

#ifdef RMTSENS
	RemoteSensor.begin();
#endif // RMTSENS

	// Set SPI Pins
	SPI.setMISO(MISO);
	SPI.setMOSI(MOSI);
	SPI.setSCLK(SCK);
	delay(10);

	// LoRa Initalisation
	if (!rf95.init())
	{
		Serial.println("LoRa init failed");
		while (1) // Halt Startup if RF95 not found
			delay(1);
	}
	rf95.setFrequency(frequency);
	rf95.setTxPower(13);
	myCipher.setKey(encryptkey, sizeof(encryptkey));

	LowPower.begin();
// Enable Rain Interrupt
#ifdef RAIN
	pinMode(rainPin, INPUT_FLOATING);
	LowPower.attachInterruptWakeup(rainPin, rainfct, RISING, DEEP_SLEEP_MODE);
#endif // RAIN

	Serial.println("Setup completed");
	delay(1000);
}

void loop()
{

	float buffer = 0;

	delay(50);
	Serial.print("Wake-Up Nr: ");
	Serial.println(nrWakeup);
	// Keeping Track of #Wakeups
	nrWakeup++;
	// Data gets send every 10th wake-up independent wether data has changed: Heartbeat
	if (!(nrWakeup % 10))
	{
		ValChng = 1;
	}
	delay(50);
	Serial.print("Rainpulse: ");
	Serial.println(rainPulses);
// Read Wind Data
#ifdef WIND
	if (anemometer.read(speed, dir))
	{
		Serial.println("Wind found");
		/* Wind Data gets only sent if speed is greater 0 and
		Speedchange > 0.5 or
		ValChng = 1 or
		Direction Change > 0.5
		*/
		if (abs(speed_old - speed) > 0.5 || ValChng || ((abs(dir_old - dir) > 0.5) && speed > 0))
		{
			ValChng = 1;
			payload["Speed"] = speed;
			speed_old = speed;
			payload["Dir"] = dir * 22.5;
			dir_old = dir;
		}
	}
#endif // WIND

// Read BME280 Data
#ifdef BME280
	bme.takeForcedMeasurement();
	/* BEM280 Data gets only sent if
	Temperaturechange > 0.5°C or
	Humiditychange > 5%
	*/
	buffer = bme.readTemperature() / 1.0;
	if (abs(Temp - buffer) > 0.5 || ValChng)
	{
		ValChng = 1;
		payload["Temp"] = buffer;
		Temp = buffer;
	}
	buffer = roundf(bme.readHumidity() * 100) / 100;
	if (abs(Hum - buffer) > 5 || ValChng)
	{
		ValChng = 1;
		payload["Hum"] = buffer;
		Hum = buffer;
	}

	buffer = roundf(bme.readPressure()) / 100;
	if (abs(Press - buffer) > 0.5 || ValChng)
	{
		ValChng = 1;
		payload["Press"] = buffer;
		Press = buffer;
	}
#endif // BME280

// Read SHT 31 Data
#ifdef SHT31
	/* SHT31 Data gets only sent if
	Temperaturechange > 0.5°C or
	Humiditychange > 5%
	*/
	buffer = sht31.readTemperature();
	if (abs(Temp1 - buffer) > 0.5 || ValChng)
	{
		ValChng = 1;
		payload["Temp1"] = buffer;
		Temp1 = buffer;
	}

	buffer = sht31.readHumidity();
	if (abs(Hum1 - buffer) > 5 || ValChng)
	{
		ValChng = 1;
		payload["Hum1"] = buffer;
		Hum1 = buffer;
	}
#endif // SHT31

// Read Remote Sensor Data
#ifdef RMTSENS
	float RemoteTemp(NAN);
	RemoteSensor.requestTemperatures();
	RemoteTemp = RemoteSensor.getTempCByIndex(0);
	if (RemoteTemp != -127)
	{
		payload["RmtTmp"] = RemoteTemp;
	}
#endif // RMTSENS

#ifdef RAIN
	payload["RainC"] = rainPulses;
#endif // RAIN

	if (!ValChng)
	{
		// Don't send anything if no new Data or not a 10th wakeup
		Serial.println("No new Data, back to zzzz");
		delay(50);

		LowPower.deepSleep(SendInterval * 1000);
	}
	else
	{
		rainPulses = 0; // Reset RainPulses

		// Read Battery and Solar Voltages (8bit)
		VBatt = analogRead(PA7) * 4.3 / 1024;
		VSol = analogRead(PB0) * 10.9 / 1024;
		payload["VBatt"] = trunc(VBatt * 100) / 100;
		if (floor(VSol) > 0)
		{
			payload["VSol"] = trunc(VSol * 100) / 100;
		}

		size_t len = 32;
		char message[256]; // SerializedJSON, replace with len?
		len = payload.memoryUsage();
		Serial.println(len);
		serializeJson(payload, message); // Create String (Char Array) from JSON
		uint8_t data[len + 1]; // RFM95 input

		for (uint8_t i = 0; i <= len; i++) // Copy and Typecast Data from JSONString to uint8_t Array
		{
			data[i] = (uint8_t)message[i];
		}
		data[len + 1] = '\0';
		myDriver.send(data, sizeof(data)); // Send out ID + Sensor data to LoRa gateway
		Serial.print("Sent: ");
		delay(50);
		Serial.println((char *)&data);
		delay(50);
		rf95.sleep(); // Put RF95 to sleep
		ValChng = 0;
		delay(100);
		// Check if Battery is above 3V, if not use LowPower Intervall for Wakups, current not used!
		if (VBatt > 3.0)
			LowPower.deepSleep(SendInterval * 1000);
		else
		{
			LowPower.deepSleep(SendInterval_LP * 1000);
			ValChng = 1;
		}
	}
}


/* 
Rain Interrupt triggers this function
Increment Number of Rain Pulses an go back to Sleep
*/
#ifdef RAIN
void rainfct()
{
	rainPulses++;
	delay(50);
	LowPower.deepSleep();
}
#endif // RAIN
