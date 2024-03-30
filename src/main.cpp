#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"
#include "Adafruit_SSD1306.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "time.h"

// WiFi credentials
#define WIFI_SSID "GARGOURI"
#define WIFI_PASSWORD "05254872"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "system1@login.com"
#define USER_PASSWORD "system1"

// Firebase credentials
#define API_KEY "AIzaSyDV924r10N-EcnC-RbeOVyjZldoncvOe6g "                                             // Replace with your Firebase API key
#define DATABASE_URL "https://smart-irrigation-74f17-default-rtdb.europe-west1.firebasedatabase.app/"  // Replace with your Firebase database URL

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String ECPath = "/ec";
String moisPath = "/moisture";
String tempPath = "/temperature";
String timePath = "/timestamp";
String o2Path = "/o2";
String pHPath = "/pH";

// Parent Node (to be updated in every loop)
String parentPath;

int timestamp;
FirebaseJson json;

const char *ntpServer = "pool.ntp.org";
// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 5000;
bool signupOK = false;

unsigned long getTime() {
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        //Serial.println("Failed to obtain time");
        return (0);
    }
    time(&now);
    return now;
}


DFRobot_ESP_EC ec;
DFRobot_ESP_PH_WITH_ADC ph;

uint8_t ec_pin = A6;
uint8_t ph_pin = A4;
uint8_t o2_pin = A7;
uint8_t SoilMoisture_pin = A5;
uint8_t temperature_pin = 25;

#define SCREEN_WIDTH 128  // OLED display width
#define SCREEN_HEIGHT 64  // OLED display height

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

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

void affiche1(String s) {
    // text display tests
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.setTextSize(1);
    //**//*display.setCursor(25, 20);*//**//*
    display.print(s);
    display.display();
    display.clearDisplay();

}

void setup() {
    Serial.begin(115200);
    EEPROM.begin(32);//needed EEPROM.begin to store calibration k in eeprom
    ec.begin();
    ph.begin();
    sensors.begin();
    configTime(0, 0, ntpServer);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Address 0x3C for 128x32
    display.clearDisplay();
    display.display();
    /*display.setFont(&FreeSerifBold9pt7b);*/
    affiche1("READY");
    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected to WiFi with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("signUp OK");
        signupOK = true;
    } else {
        Serial.printf("%s\n", config.signer.signupError.message.c_str());
    }

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    // Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);

    // Getting the user UID might take a few seconds
    Serial.println("Getting User UID");
    while ((auth.token.uid) == "") {
        Serial.print('.');
        delay(1000);
    }
    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);

    // Update database path
    databasePath = "/SystemsData/" + uid + "/readings";

    setCpuFrequencyMhz(81);
}

void send_to_firebase(float ec, float o2, float ph, float moisture, float tem) {
    if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();

        //Get current timestamp
        timestamp = getTime();
        Serial.print("time: ");
        Serial.println(timestamp);


        parentPath = databasePath + "/" + String(timestamp);


        json.set(ECPath.c_str(), String(ec));
        json.set(pHPath.c_str(), String(ph));
        json.set(o2Path.c_str(), String(o2));
        json.set(tempPath.c_str(), String(tem));
        json.set(moisPath.c_str(), String(moisture));
        json.set(timePath, String(timestamp));
        Serial.printf("Set json... %s\n",
                      Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());

    }
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
    if (millis() - timepoint > timerDelay) //time interval: 1s
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
    if (millis() - timepoint > timerDelay) //time interval: 1s
    {
        timepoint = millis();


        voltage = (analogRead(PH_PIN) / 4096.0F) * 5000;

        phValue = abs(ph.readPH(voltage, readTemperature())); // convert voltage to pH with temperature compensation
        Serial.print("pH:");
        Serial.println(phValue);
    }
    //ph.calibration(voltage, temperature); // calibration process by Serail CMD
    return phValue;
}

float readO2(uint8_t DO_PIN) {

    static unsigned long timepoint = millis();
    if (millis() - timepoint > timerDelay) //time interval: 1s
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

float readSoilMoisture(uint8_t pin) {
    static unsigned long timepoint = millis();
    if (millis() - timepoint > timerDelay) //time interval: 1s
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
    affiche1("Temperature: " + (String) readTemperature() + " C" + '\n' + "EC: " + (String) readEC(ec_pin) + " ms/cm" +
             '\n' + "Dissolved O2: " + (String) readO2(o2_pin) + '\n' + "pH: " + (String) readPh(ph_pin) + '\n' +
             "Soil Moisture: " + (String) readSoilMoisture(SoilMoisture_pin) + " %");

    /*readPh(ph_pin);
    readO2(o2_pin);
    readSoilMoisture(SoilMoisture_pin);*/
    /*send_to_firebase(readEC(ec_pin), readO2(o2_pin), readPh(ph_pin), readSoilMoisture(SoilMoisture_pin),
                     readTemperature());*/

}