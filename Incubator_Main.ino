
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


float setTemp = 15.0;
float curTemp = 0.0;
String text;
int upperTempThreshold = 20;
int lowerTempThreshold = 10;



const int BUTTON_UP = 2;
const int BUTTON_DWN = 3;
long unsigned int lastPress;
const int debounceTime = 20;

volatile int buttonUP;
volatile int buttonDWN;

int analogPin = A0;
int sensor;
float vout;
float tempInCelsius;
float tempInFahrenheit;

/*--------------------------------------*/


// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;





LiquidCrystal_I2C lcd(0x27,20,4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  /* ----- Initializing LCD display ----- */
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);

  lcd.print("Temp:");

  lcd.setCursor(0,1);
  lcd.print("SetTemp:");
  updateLCDsetTemp( String(setTemp, 1) );
    
  
  /* ----- Assigning In/Output Pins ----- */
  pinMode(BUTTON_UP, INPUT);
  pinMode(BUTTON_DWN, INPUT);
  

  /* ----- Assigning Interrupt Pins ----- */
  attachInterrupt(digitalPinToInterrupt(2), ISR_buttonUP, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ISR_buttonDWN, RISING);



  /*------ Dallas Temperature Sensor Initialization Code -----*/


  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  
} 

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void loop() {
  
  /* ---- Setting the temp up/down ----- */
  if( buttonUP && debounceCheck() && setTemp != upperTempThreshold )
  {
    setTemp = setTemp + 0.5;
    text = String(setTemp, 1);
    updateLCDsetTemp(text);
  }
  if( buttonDWN && debounceCheck() && setTemp != lowerTempThreshold )
  {
    setTemp = setTemp - 0.5;
    text = String(setTemp, 1);
    updateLCDsetTemp(text);
  }

  /* ---- Display the current temp ----- */


  //updateLCDcurTemp( readTemp() );
  delay(500);


  /* ---- Adjusting the current temp --- */
  /*
  if( setTemp > curTemp && heaterFlag == 0 )
  {
    // heat nichrom w/ 5V
    digitalWrite(HIGH, tempSensorPin)
    heaterFlag = 1
  }
  if( setTemp > curTemp && heaterFlag == 0 )
  {
    // check timer
    if( time's up )
    {
      // turn off nichrome
      digitalWrite(LOW, tempSensorPin);
    }
  }
  if( curTemp > setTemp )
  {
    // turn on fan
  }

  */
  /* ---- Alarms (if too hot/cold) ----- */
}

bool debounceCheck()
{
  if( (millis() - lastPress) > debounceTime)
  {
    lastPress = millis(); 
    buttonUP = 0;
    buttonDWN = 0;
    
    return true;
  }
  return false;
}

String readTemp()
{
  /*
  // read the temperature sensor and convert to volts
   sensor = analogRead(analogPin); 
   // vout = (sensor/bits)*volts
   vout = (sensor/1023.0)*5000;


   
   // convert to temperature in degrees C
   tempInCelsius = (vout - 500.0)/10.0; 
   tempInFahrenheit = 1.8*tempInCelsius + 32;

   tempInFahrenheit = tempInFahrenheit + 5;
   
   // print the temperature to the Serial Monitor
   //Serial.println(tempInFahrenheit);
   return String( tempInFahrenheit, 2);
   
  // the delay sets the data collection rate
  // delay(2000); //delay in milliseconds
  */

  sensors.requestTemperatures();
  
  
}

void updateLCDsetTemp(String text)
{
  
  lcd.setCursor(9,1);

  lcd.print(text);
}

void updateLCDcurTemp(String text)
{
 
  lcd.setCursor(9,0);

  lcd.print(text);
}

void ISR_buttonUP()
{
  buttonUP = 1;
}

void ISR_buttonDWN()
{
  buttonDWN = 1;
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
