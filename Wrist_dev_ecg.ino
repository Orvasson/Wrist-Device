
/*

  For MAX30003 (ECG 1-lead):
  //    Arduino connections:
  //
  //  |MAX30003 pin label| Pin Function         |Arduino Connection|
  //  |----------------- |:--------------------:|-----------------:|
  //  | MISO             | Slave Out            |  D12             |
  //  | MOSI             | Slave In             |  D11             |
  //  | SCLK             | Serial Clock         |  D13             |
  //  | CS               | Chip Select          |  D7              |
  //  | VCC              | Digital VDD          |  MCP1700 1.8V    |
  //  | GND              | Digital Gnd          |  GND             |
  //  | FCLK             | 32K CLOCK            |  -               |
  //  | INT1             | Interrupt1           |  D2              |
  //  | INT2             | Interrupt2           |  -               |

  For SEN15805(TMP117) (Temperature 0.01 accuracy):
  //    Arduino connections:
  //
  //  |SEN15805 pin label| Pin Function         |Arduino Connection|
  //  |----------------- |:--------------------:|-----------------:|
  //  | GND              | Digital Gnd          |  GND             |
  //  | 3.3V             | Digital VDD          |  MCP1700 1.8V    |
  //  | SDA              | Serial Data          |  A4 (or SDA)     |
  //  | SCL              | Serial Clock         |  A5 (or SCL)     |
  //  | INT              | Interrupt            |  --              |

  For Button(Switch):
  Connected to MCP1700 1.8V, GND and D4. From Button to GND connected a 10KΩ resistance.
  When Pressed runs MAX30003 for 45sec

  For MAX30101 (HeartRate and SP02 measurements):

  Hardware Connections (Breakoutboard to Arduino):
  -3.3V = 3.3V
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -1.8V
  -INT = Not connected

  The MAX30101 Breakout can handle 1.8V or 3.3V I2C logic.

  //The MAX30101 has many settings. By default we select:
  // Sample Average = 4
  // Mode = MultiLED
  // ADC Range = 16384 (62.5pA per LSB)
  // Sample rate = 50

  Information on:
  1)How to read BLE values:               https://www.asciitable.com/
  2)Fixed UUIDs :                         https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
                                         https://github.com/NordicSemiconductor/bluetooth-numbers-database/blob/master/v1/characteristic_uuids.json
  3)SEN15805 Library:                     https://github.com/sparkfun/SparkFun_TMP117_Arduino_Library
  4)MAX30101 Library:                     https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
  5)MAX30003 Library:                     https://github.com/Protocentral/protocentral_max30003
  6)NRF52 Timer Library:                  https://github.com/khoih-prog/NRF52_TimerInterrupt#features


*/
/////////////////////////////////////////
//Libraries and init var for TIMERS of nrf52
// These define's must be placed at the beginning before #include "NRF52TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// For Nano33-BLE, don't use Serial.print() in ISR as system will definitely hang.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include "NRF52_MBED_TimerInterrupt.h"

//TIMER1
#define TIMER1_INTERVAL_MS        500  //margin of error
#define TIMER1_DURATION_MS        60000 // duration of "delay" on timer1 (milliseconds)
volatile uint32_t preMillisTimer1 = 0;
static bool toggle1 = false;

//Duration of SP02
#define TIMER2_DURATION_MS        2500 // duration of sp02 measurements 


//Duration of ECG
#define TIMER3_DURATION_MS        7500 // duration of ECG (milliseconds)aka 45sec
int countecg = 0;
int countHR = 0;
int countRR = 0;


// Depending on the board, you can select NRF52 Hardware Timer from NRF_TIMER_1,NRF_TIMER_3,NRF_TIMER_4 (1,3 and 4)
// If you select the already-used NRF_TIMER_0 or NRF_TIMER_2, it'll be auto modified to use NRF_TIMER_1

// Init NRF52 timer NRF_TIMER4
NRF52_MBED_Timer ITimer1(NRF_TIMER_4);


void TimerHandler1()
{
  preMillisTimer1 = millis();

}

//libraries for SERIAL, MAX30105, SEN15805 (TMP117), HR algorithm and SP02 algorithm
#include <Wire.h>
#include "MAX30105.h"
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor
#include "heartRate.h"
#include "spo2_algorithm.h"

//init for SP02 algorithm
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


//init for HR algorithm
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//Init for MAX30003 for 1-lead ECG
#include<SPI.h>
#include "protocentral_Max30003.h"

MAX30003 max30003;  //Initialize sensor MAX30003 for ECG

// The default address of the device is 0x48 = (GND)
TMP117 sensor; // Initalize sensor for temperature

//The default address of the device is 0x57
MAX30105 particleSensor;  //Initialize sensor for HR and SP02



//STORED on Ram for BLE
int16_t stored_temp = 0; // value that we sent through BLE
int32_t stored_sp02[25];  //value to calculate the mean from the measurements

long int stored_ecgdata;  //the measurements for 30sec ECG
uint8_t stored_MAX30003_HRdata; //the Heart Rate measurements for 30sec from MAX30003
uint16_t stored_RRdata;    //Measurements of time From R to R peak on 30sec ECG (ms)


//attempt to store shit in txt file
#include <iostream>   //fixed
//for precision 0.01
#include <iomanip>      // std::setprecision 

//BLE library
#include <ArduinoBLE.h>

// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.heart_rate_measurement.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.heart_rate_control_point.xml
// https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.body_sensor_location.xml

#define BLE_UUID_HEART_RATE_SERVICE               "180D"       // from 16-bit UUID NumbersDocument
#define BLE_UUID_HEART_RATE_MEASURMENT            "2A37"        // from 16-bit UUID NumbersDocument
#define BLE_UUID_BODY_SENSOR_LOCATION             "2A38"        // from 16-bit UUID NumbersDocument
//#define BLE_UUID_HEART_RATE_CONTROL_POINT         "2A39"        // from 16-bit UUID NumbersDocument
#define BLE_UUID_HEART_RR_INTERVAL                "0201"    //read only no security, up to length 2 byte 



//----------------------------------------------------------------------------------------------------------------------
// BLE Heart Rate Measurment from MAX30101
//----------------------------------------------------------------------------------------------------------------------

// Constants for flags in Heart Rate Measurement (see XML link) my flag = 0b00010111
#define HRM_VALUE_FORMAT_8BIT                     0
#define HRM_VALUE_FORMAT_16BIT                    1
#define HRM_SENSOR_CONTACT_NOT_SUPPORTED          ( 0 << 1 )
#define HRM_SENSOR_CONTACT_NOT_DETECTED           ( 2 << 1 )
#define HRM_SENSOR_CONTACT_DETECTED               ( 3 << 1 )
#define HRM_ENERGY_EXPENDED_NOT_PRESENT           ( 0 << 3 )
#define HRM_ENERGY_EXPENDED_PRESENT               ( 1 << 3 )
#define HRM_RR_INTERVAL_PRESENT                   ( 1 << 4 )
#define HRM_RR_INTERVAL_NOT_PRESENT               ( 0 << 4 )


enum { BODY_SENSOR_LOCATION_OTHER = 0,
       BODY_SENSOR_LOCATION_CHEST,
       BODY_SENSOR_LOCATION_WRIST,
       BODY_SENSOR_LOCATION_FINGER,
       BODY_SENSOR_LOCATION_HAND,
       BODY_SENSOR_LOCATION_EAR_LOBE,
       BODY_SENSOR_LOCATION_FOOT
     };
//for Heartbeat
typedef struct __attribute__( ( packed ) )
{
  uint8_t flags ;
  uint16_t heartRate;
  uint16_t RR_interval;

} heart_rate_measurment;


union heart_rate_measurment_u
{
  struct __attribute__( ( packed ) )
  {
    heart_rate_measurment values;
  };
  uint8_t bytes[ sizeof( heart_rate_measurment ) ];
} ;

union heart_rate_measurment_u heartBeat = { .values = { .flags = 0b11110111, .heartRate = 0, .RR_interval = 0 } };

uint16_t RR_inte = 0;
uint16_t heartRate11 ;
uint16_t RR_int;
uint8_t flags = 0b11110111;

//#define BLE_DESCRIPTOR_SP02_UUID         "2902"   //from 16-bit UUID NumbersDocument

BLEService heartRateService( BLE_UUID_HEART_RATE_SERVICE );
BLECharacteristic heartRateCharacteristic( BLE_UUID_HEART_RATE_MEASURMENT, BLERead | BLENotify, sizeof heartBeat.bytes );
//BLEDescriptor  heartRateDescriptorCharacteristic( "0001", "Heart Rate Values" );
BLEIntCharacteristic RRCharacteristic( BLE_UUID_HEART_RR_INTERVAL, BLERead | BLENotify );
//BLEDescriptor  RRDescriptorCharacteristic( "0002" , "RR interval  Values" );
BLEUnsignedCharCharacteristic bodySensorLocationCharacteristic( BLE_UUID_BODY_SENSOR_LOCATION, BLERead );

//for SP02 and Temperature Service
//https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.plx_spot_check_measurement.xml
//https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.temperature.xml
//https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Descriptors/org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml

#define BLE_Temperature_Service_UUID     "1809"   //from 16-bit UUID NumbersDocument
#define BLE_SP02_Service_UUID            "1822"   //from 16-bit UUID NumbersDocument
#define BLE_ECG_Service_UUID             "2D0D"   //from 16-bit UUID NumbersDocument


#define BLE_Temperature_UUID             "2A6E"        //from 16-bit UUID NumbersDocument
#define BLE_SP02_UUID                    "2A5E"       //from 16-bit UUID NumbersDocument

#define BLE_ECG_UUID                      "2D37"     //from 16-bit UUID NumbersDocument
#define BLE_ECG_POSITIVE_UUID             "22220020-1c35-402b-b938-af832d35a1c3"
#define BLE_ECG_NEGATIVE_UUID             "33330020-1c35-402b-b938-af832d35a1c3"
#define BLE_ECG_UUID2                      "51160020-1c35-402b-b938-af832d35a1c3"     //from 16-bit UUID NumbersDocument
#define BLE_ECG_UUID3                      "52a5a59f-e814-4523-8489-f3c99e993d9b"     //from 16-bit UUID NumbersDocument
//----------------------------------------------------------------------------------------------------------------------
// BLE Sp02 Measurment from MAX30101
//---------------------------------------------------------------------------------------------------

//
int stored_final_sp02 = 0; // value that we sent through BLE
//int stored_PR_sp02;
//uint8_t flags1 = 0b11100010;
//uint16_t flags_measurement = 0x1F;   //ob0000000000011111;
//----------------------------------------------------------------------------------------------------------------------
// BLE Temperature Measurment and ECG measurement-RR interval-HeartRate from MAX30003
//----------------------------------------------------------------------------------------------------------------------
BLEService temperatureService( BLE_Temperature_Service_UUID );  // Service for temperature measurements
BLEService spo2Service( BLE_SP02_Service_UUID );  //Service for SP02 measurements
BLEService ECGService( BLE_ECG_Service_UUID ); // Service for ECG / HR/ RR

//Temperature Characteristic
BLEIntCharacteristic temperatureCharacteristic( BLE_Temperature_UUID , BLERead | BLENotify ); // remote clients will only be able to read this
//SP02 Characteristic
BLEIntCharacteristic spo2Characteristic( BLE_SP02_UUID, BLERead | BLENotify );
//BLECharacteristic.hasDescriptor(BLE_Temperature_UUID);
//ECG Characteristics
//BLELongCharacteristic ecgCharacteristic( BLE_ECG_UUID  , BLERead | BLENotify );
BLEIntCharacteristic ecgPositiveCharacteristic( BLE_ECG_POSITIVE_UUID , BLERead | BLENotify);
BLEIntCharacteristic ecgNegativeCharacteristic( BLE_ECG_NEGATIVE_UUID, BLERead | BLENotify);
//BLEDescriptor  ecgDescriptorCharacteristic( "2901", "ECG Values" );
//BLEIntCharacteristic ecgHeartRateCharacteristic ( BLE_ECG_UUID2  , BLERead | BLENotify );
//BLEDescriptor  ecgDescriptorHRCharacteristic( "2901", "Heart Rate  Values" );
//BLEIntCharacteristic ecgRRCharacteristic ( BLE_ECG_UUID3  , BLERead | BLENotify );
//BLEDescriptor  ecgDescriptorRRCharacteristic( "2902", "RR interval  Values" );




//button pressed for ECG
// constants won't change. They're used here to set pin numbers:
const int buttonPin = D8;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status

//to send heart rate via ble every 1 secon
long timerHeartRate;
long timerHeart;
long timerECG;
uint32_t negative_ecg, positive_ecg;

void setup()
{


  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);  //D4 pin is used for button

  //MAX30101 INIT for HEARTBEAT
  Serial.begin(115200);
  // while (!Serial);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  ///SETUP configuration vars for HEARTBEAT///
  //We need Use 6.4mA for LED drive,Red+IR, sampleRate=50, adcRange = 16384, sampleAverage = 4, pulseWidth=?
  Serial.println("Initializing for HeartBeat");
  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //END_HeartBeat//

  //MAX30101 Init for SP02 algorithm
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  //  while (!Serial);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    // while (1);
  }

  ///SETUP configuration vars for SP02 ///
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  int pulseWidth = 411; //Options: 69 get us 15 bit res, 118 get us 16bit res, 215 get us 17 bit res, 411 get us 18bit res
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //END_SP02//

  //TEMPERATURE CODE Init

  Serial.begin(115200);
  // while (!Serial);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  //Check if TMP117 is working and start the readings
  Serial.println("TMP117 Init for Readings");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin measurements");
  }
  else
  {
    Serial.println("Device failed to setup- Freezing code.");
    // while (1); // Runs forever
  }

  //END_TEMP//

  /// TIMER Init for Timer1 for SP02 and Temperature Measurements

  // Interval in microsecs
  if (ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1))
  {
    preMillisTimer1 = millis();
    Serial.print(F("Starting ITimer1 OK, millis() = ")); Serial.println(preMillisTimer1);
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  //END_Timer1


  //MAX30003 INIT
  Serial.begin(57600); //Serial begin
  //while (!Serial);

  pinMode(MAX30003_CS_PIN, OUTPUT);
  digitalWrite(MAX30003_CS_PIN, HIGH); //disable device

  SPI.begin();

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  delay(100);

  bool ret = max30003.max30003ReadInfo();
  if (ret) {
    Serial.println("Max30003 read ID Success");
  } else {

    while (!ret) {
      //stay here untill the issue is fixed.
      ret = max30003.max30003ReadInfo();
      Serial.println("Failed to read ID, please make sure all the pins are connected");
      delay(10000);
    }
  }

  // Serial.println("Initialising the chip ...");
  max30003.max30003Begin();   // initialize MAX30003 for plot
  // max30003.max30003BeginRtorMode();   // initialize MAX30003 for RR interval and HR


  //BLE
  Serial.begin(9600);    // initialize serial communication
  //while (!Serial);

  if (!BLE.begin()) {   // initialize BLE
    Serial.println("starting BLE failed!");
    while (1);
  }
  //set advertised local name and service
  BLE.setDeviceName("Wrist Device");
  BLE.setLocalName("Wrist Device");  // Set name for connection

  //setAdvertisedServices
  BLE.setAdvertisedService( temperatureService ); // Advertise service
  BLE.setAdvertisedService( spo2Service ); // Advertise service
  BLE.setAdvertisedService( heartRateService );
  BLE.setAdvertisedService( ECGService );

  //BLE and characteristics for temperatureService
  temperatureService.addCharacteristic(temperatureCharacteristic); // Add characteristic to service

  //BLE and characteristics for spo2Service
  spo2Service.addCharacteristic( spo2Characteristic );

  //Characteristics for Heart Rate Service
  heartRateService.addCharacteristic( heartRateCharacteristic );
  heartRateService.addCharacteristic( RRCharacteristic );
  heartRateService.addCharacteristic( bodySensorLocationCharacteristic );

  //Characteristics for ECG / HR / RR
  // ECGService.addCharacteristic( ecgCharacteristic );
  ECGService.addCharacteristic( ecgPositiveCharacteristic );
  ECGService.addCharacteristic( ecgNegativeCharacteristic );
  // ECGService.addCharacteristic( ecgHeartRateCharacteristic );
  // ECGService.addCharacteristic( ecgRRCharacteristic );

  //add Services
  BLE.addService( temperatureService );
  BLE.addService( spo2Service );
  BLE.addService( heartRateService );
  BLE.addService( ECGService );

  // set the initial value for the Heart Rate characeristics
  heartRateCharacteristic.writeValue( heartBeat.bytes, sizeof heartBeat.bytes );
  RRCharacteristic.writeValue( RR_inte );
  bodySensorLocationCharacteristic.writeValue( BODY_SENSOR_LOCATION_WRIST );

  // set the initial value for the SP02 characteristic
  // SP02_Mes.values.flags1 = flags1;
  //SP02_Mes.values.flags_measurement = flags_measurement;

  spo2Characteristic.writeValue( stored_final_sp02 );


  BLE.advertise();  // Start advertising
  Serial.print("Peripheral device MAC: ");
  Serial.println(BLE.address());
  Serial.println("Waiting for connections...");

  timerHeartRate = millis();
  timerHeart = millis();
  timerECG = millis();

}


void loop()
{

  BLEDevice central = BLE.central();                    //Waits for BLE Central device to connect

  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      //   Serial.println("im still running...");
      buttonState = digitalRead(buttonPin);

      if (buttonState == LOW) {


        /////  TIMER1 LOOP  //////

        static unsigned long lastTimer1   = 0;
        static bool timer1Stopped         = false;

        int count = 0;
        int sum_sp02;

        if (millis() - lastTimer1 > TIMER1_DURATION_MS)
        {
          lastTimer1 = millis();

          if (timer1Stopped)
          {
            preMillisTimer1 = millis();
            //   Serial.print(F("Start ITimer1, millis() = ")); Serial.println(preMillisTimer1);
            ITimer1.restartTimer();

            PPG_meas();
            temperatures();


            //store sp02 in RAM
            for (int i = 1; i <= 25; i++)
            {
              if (stored_sp02[i] > 40 && stored_sp02[i] <= 100) {


                count++;
                sum_sp02 = (stored_sp02[i] + sum_sp02);
                stored_final_sp02 = (sum_sp02 / count); //final measurement of sp02 ,average of max 25 measurements
                //   stored_PR_sp02 = stored_final_sp02 ;
                if (i == 25) {
                  count = 0;
                }
              }
              if (stored_sp02[i] == -999) {
                continue;
              }

            }
            count = 0;
            sum_sp02 = 0;

            Serial.println("Oi stored times einai: ");
            Serial.println("Temp, sp02");
            // Serial.println(stored_temp, 4);
            Serial.println(stored_final_sp02);



            //set the initial value for charasteristics
            temperatureCharacteristic.writeValue( stored_temp); //temperature value
            spo2Characteristic.writeValue( stored_final_sp02 );

          }
          else
          {

            //   Serial.print(F("Stop ITimer1, millis() = ")); Serial.println(millis());
            ITimer1.stopTimer();

          }

          timer1Stopped = !timer1Stopped;
        }

        // If timer is still on going
        heartbeat();
        heartBeat.values.heartRate = heartRate11;
        heartBeat.values.RR_interval = RR_int ;
        //
        timerHeartRate = millis();
        if (timerHeartRate > timerHeart + 850) {
          heartRateCharacteristic.writeValue( heartBeat.bytes, sizeof heartBeat.bytes );
          RRCharacteristic.writeValue( RR_inte );
          timerHeart = millis();
        }

        //        if(timerHeartRate > timerECG + 80000){
        //          ECG();
        //          timerECG = millis();
        //        }

      }
      else { //if button is pressed do 45sec of ECG

        Serial.println("Start ECG measurement");
        ECG();
        countecg = 0;   //counter of ECG samples during 30 sec
        countHR = 0; //counter for HR from electrodes
        countRR = 0; // counter for RR interval
        Serial.println("End of 45-sec ECG");
      }
    }

  }

  Serial.println("Disconnected...");
}

void temperatures() {

  sensor.setOneShotMode(); // Sets mode register value to be 0b11

  delay(200);

  float sum = 0;
  float tempCelcius = 0.0; //the printed value of temperature
  float thermo[10];

  //Temp prints

  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    // float tempC = sensor.readTempC();
    for (int iota = 1; iota < 11; iota++)
    {
      float tempC = sensor.readTempC();
      thermo[iota] = tempC;
      sum = sum + thermo[iota];
      tempCelcius = (sum / iota);
    }
    // Print temperature in °C

    Serial.print("Temperature in Celsius after 10 readings: ");
    Serial.println(tempCelcius, 4);

    // Keep precision of 0.01 to send in BLE
    std::cout << std::setprecision(3) << tempCelcius << '\n';
    std::cout << std::fixed;
    //   Serial.println(tempCelcius);

    //store in global variable
    stored_temp = (tempCelcius * (100));

    sensor.setShutdownMode(); // Sets mode register value to be 0b01

  }

}

void heartbeat() {

  //Heart beat calc and prints
  long irValue = particleSensor.getIR();

  if (irValue > 50000) {

    heartBeat.values.flags = 0b11100111;

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 220 && beatsPerMinute > 40)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    //  Serial.print("IR=");
    //  Serial.print(irValue);
    //    Serial.print(", BPM=");
    //    Serial.print(beatsPerMinute);
    Serial.print(" Avg BPM=");
    Serial.print(beatAvg);

    heartRate11 =  beatAvg ;

    if ( heartRate11 > 40) {
      uint16_t  RR =  (60000 / heartRate11);
      //  Serial.print(" RR Interval is:"); Serial.print(RR);
      RR_int = RR;
      RR_inte = RR;
    }


    // Serial.print(" No finger?");
  } else {
    //  Serial.println("No finger");
    RR_inte = 0;
    RR_int = 0;
    beatAvg = 0;
    heartRate11 = 0;
    heartBeat.values.flags = 0b11100001;
  }
  Serial.println();

}

void PPG_meas() {

  //Sent to BLE that SP02 is ongoing
  // SP02_Mes.values.flags_measurement = 0x203F;     //ob0010000000111111;
  //  spo2Characteristic.writeValue( SP02_Mes.bytes1, sizeof SP02_Mes.bytes1 );

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    if (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //    Serial.print(F("red="));
    //    Serial.print(redBuffer[i], DEC);
    //    Serial.print(F(", ir="));
    //    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  //Continuously taking samples from MAX30105.  Heart rate and SpO2 are calculated every 1 second
  static unsigned long sp02Timer1   = 0;
  static unsigned long sp02Timer2   = 0;

  sp02Timer1 = millis();
  sp02Timer2 = millis();

  while (sp02Timer1 < sp02Timer2 + TIMER2_DURATION_MS)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data


      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //PRINTS SP02 until Timer2 stops
      //      Serial.print(F(", SPO2="));
      //      Serial.print(spo2, DEC);
      //
      //      Serial.print(F(", SPO2Valid="));
      //      Serial.println(validSPO2, DEC);

      stored_sp02[i - 74] = spo2;

      sp02Timer1 = millis();  // timer to exit out of sp02 measurements

    }


  }
  //  if(validSPO2 == 1){
  //     SP02_Mes.values.flags_measurement = 0x49F;    //ob0000010010011111;
  //     if( spo2 >= 93 ){
  //       SP02_Mes.values.flags_measurement = 0x69F; //ob0000011010011111;
  //     }
  //     else if( spo2 <= 75 ){
  //        SP02_Mes.values.flags_measurement =  0x449F; //ob0100010010011111;
  //     }
  //  }else{
  //    SP02_Mes.values.flags_measurement = 0xC01F; //ob1100000000011111;
  //  }
  //  Serial.println("We got Sp02 boi");

}

void ECG()
{
  static unsigned long ecgTimer1   = 0;
  static unsigned long ecgTimer2   = 0;

  ecgTimer1 = millis();
  ecgTimer2 = millis();

  while (ecgTimer1 < ecgTimer2 + TIMER3_DURATION_MS)
  {
    int i = 0;
    max30003.getEcgSamples();   //It reads the ecg sample and stores it to max30003.ecgdata .

    Serial.println(max30003.ecgdata);
    stored_ecgdata = max30003.ecgdata;
//    if (i == 0) {
//      if (stored_ecgdata < 0 ) {
//        negative_ecg = stored_ecgdata * (-1);
//
//        ecgNegativeCharacteristic.writeValue( negative_ecg );
//      }
//    }
    //if ( ecgTimer1 > ecgTimer2 + 5000) {
   // if( i >=7){
      //testing for ecg cause crash
      if (stored_ecgdata < 0 ) {
        negative_ecg = stored_ecgdata * (-1);

        ecgNegativeCharacteristic.writeValue( negative_ecg );
      } else {
        positive_ecg = stored_ecgdata;

        ecgPositiveCharacteristic.writeValue( positive_ecg );
      }
      // ecgCharacteristic.writeValue(stored_ecgdata);
      delay(8);


      countecg++;

   // }
    //for RR interval and HR

    // max30003.max30003BeginRtorMode();   // initialize MAX30003 for RR interval and HR

    // max30003.getHRandRR();   //It will store HR to max30003.heartRate and rr to max30003.RRinterval.
    // Serial.print("Heart Rate  = ");
    //  Serial.println(max30003.heartRate);
    // stored_MAX30003_HRdata = max30003.heartRate;
    // ecgHeartRateCharacteristic.writeValue( stored_MAX30003_HRdata );
    //  countHR++;

    //   Serial.print("RR interval  = ");
    //  Serial.println(max30003.RRinterval);
    // stored_RRdata = max30003.RRinterval;
    //  ecgRRCharacteristic.writeValue( stored_RRdata );
    // countRR++;

    i++;

    ecgTimer1 = millis();


  }
  //  Serial.println("synolo tou countecg se 30sec");
  //  Serial.println(countecg);
  //  Serial.println("synolo tou countHR se 30sec");
  //  Serial.println(countHR);
  //  Serial.println("synolo tou countRR se 30sec");
  //  Serial.println(countRR);
}
