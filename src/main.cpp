#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"
#include "Adafruit_SSD1306.h"
#include <OneWire.h>
#include <DallasTemperature.h>


DFRobot_ESP_EC ec;
DFRobot_ESP_PH_WITH_ADC ph;
uint8_t ec_pin = A6;
uint8_t ph_pin = A4;
uint8_t o2_pin = A7;
String amer;
uint8_t SoilMoisture_pin = A5;
uint8_t temperature_pin = 25;
double _moisture, sensor_analog;
// Set up a oneWire instance to communicate with any OneWire devices
OneWire oneWire(temperature_pin);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
float voltage, phValue, temperature = 25, ecValue, DOvalue;


#define DO_PIN A1

#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530, 11260, 11010,
                               10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270, 9080, 8900, 8730, 8570, 8410, 8250,
                               8110, 7960, 7820, 7690, 7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530,
                               6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c) {
#if TWO_POINT_CALIBRATION == 0
    uint16_t V_saturation = (uint32_t) CAL1_V + (uint32_t) 35 * temperature_c - (uint32_t) CAL1_T * 35;
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

void setup() {
    Serial.begin(115200);
    EEPROM.begin(32);//needed EEPROM.begin to store calibration k in eeprom
    ec.begin();
    ph.begin();
    sensors.begin();
}

float readTemperature() {
    sensors.requestTemperatures();
    if (sensors.getTempCByIndex(0) == -127) {
        Serial.print("Celsius temperature (Not Sure): ");
        Serial.println(CAL1_T);
        return CAL1_T;

    } else {
        Serial.print("Celsius temperature: ");
        // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
        Serial.print(sensors.getTempCByIndex(0));
        Serial.print(" - Fahrenheit temperature: ");
        Serial.println(sensors.getTempFByIndex(0));

        return sensors.getTempCByIndex(0);
    }
}

float readEC(uint8_t EC_PIN) {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) //time interval: 1s
    {

        timepoint = millis();

        voltage = analogRead(EC_PIN) / 4096.0F * 5000;


        ecValue = ec.readEC(voltage, readTemperature()); // convert voltage to EC with temperature compensation
        Serial.print("EC:");
        Serial.print(ecValue, 4);
        Serial.println("ms/cm");
    }
    //ec.calibration(voltage, temperature); // calibration process by Serail CMD
    return ecValue;
}

float readPh(uint8_t PH_PIN) {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) //time interval: 1s
    {
        timepoint = millis();


        voltage = (analogRead(PH_PIN) / 4096.0F) * 5000;

        phValue = -ph.readPH(voltage, readTemperature()); // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.println(phValue);
    }
    //ph.calibration(voltage, temperature); // calibration process by Serail CMD
    return phValue;
}

float readO2(uint8_t DO_PIN) {

    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) //time interval: 1s
    {
        timepoint = millis();
        Temperaturet = (uint8_t) readTemperature();
        ADC_Raw = analogRead(DO_PIN);
        ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
        DOvalue = readDO(ADC_Voltage, Temperaturet);
        Serial.println("DO: " + (String) DOvalue);
    }
    return DOvalue;
}

double readSoilMoisture(uint8_t pin) {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > 1000U) //time interval: 1s
    {
        timepoint = millis();
        sensor_analog = analogRead(pin);
        _moisture = (100 - ((sensor_analog / 4095.00) * 100));
        Serial.print("Moisture = ");
        Serial.print(_moisture);  /* Print Temperature on the serial window */
        Serial.println("%");
    }
    return _moisture;
}

void loop() {
    readEC(ec_pin);
    readPh(ph_pin);
    readO2(o2_pin);
    readSoilMoisture(SoilMoisture_pin);
}