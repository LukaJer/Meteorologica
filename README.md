# Meteorologica

## A ultra-low-power, flexible Environment Monitioring Platform  
### Hardware Desctiption:  
- uC: STM32G071
- LoRa Module: RFM95
- power management: BQ25060 (4.4V - 10.2V input)
- I2C 
- 5 GPIOs: (PA0 (int0), PA8, PA15, PB3, PB5)
- JST ZH 1.5mm connectors
- 1.27mm header for for programming and debugging (SWD+UART)
#### Detailed Description  
It's centered around a STM32G071 microcontroller and uses LoRa for efficient and longish range communication. Currently it sends to a custom Gateway which forwards the data to a MQTT server but the code can be easily adapted to use another LoRa infrastructure like The Things Network. The LoRa Module is a RFM95.  
A 18650 cell (other Li Ion batterys can be used too) is used for power  which can be charged from a powersource (4.4V - 10.2V) or via USB-C. The charging is done by a TI BQ25060 which offers semi-MPPT functionality. The uC can read the solar and battery voltage.  
There are 5 GPIOs available (PA0 (int0), PA8, PA15, PB3, PB5) and a I2C bus. All GPIOs have footprints for pull-up/-down resistors or capacitors and 3V3 and GND connections next to them. JST ZH 1.5mm connectors are used and a 1.27mm header is present for programming and debugging (SWD+UART). Each sensor can be either powered directly from 3V3 or a toggleable 3V3 bus . NOTE: the toggleable bus is implemented in Hardware but untested and has currently software support.

### Software Description:
The project is written in Arduino.  
Each Station has it's own name so multiple can be used on a single Gateway. 
The station wakes up in specified Intervalls (2min) and checks if any of the sensor values have changed, if so it sends the data as a JSON to the Gatway. The data is send every 10th wakeup no matter the values as a heartbeat. 
Currently
- Bosch BME280
- Sensiron SHT31
- Lacrosse TX23
- MaXIM DS18B20
- generic simple rain sensor
are implemented an can be enabled by uncommenting code.
