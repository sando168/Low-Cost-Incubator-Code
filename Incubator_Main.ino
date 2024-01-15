
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);

// Initializing Temperature Variables:
//  setTemp = The set to temperature controlled by the buttons
//  curTemp = current temperature that the sensor is reading
// Dallas temperature sensor is initialized to pin 4
// To change Temp sensor readings to/from Celsius or Fahrenheit
// see: if(Timer1Flag && Timer1seconds == 1) block in loop().

float setTemp = 24.0;
float curTemp = 0.0;
String text;
int upperTempThreshold = setTemp + 1;
int lowerTempThreshold = setTemp - 1;
int maxTemp = 98;
int minTemp = 70;


const int BUTTON_UP = 2;
const int BUTTON_DWN = 3;
unsigned long lastPress;
const int debounceTime = 400;

volatile int buttonUP;
volatile int buttonDWN;

int sensor;
float vout;
float tempInCelsius;
float tempInFahrenheit;

int fanPin = 5;

int alarmPin = 6;
bool alarmState = 0;
unsigned long alarmTimeCheck = 0;
int alarmTime = 1 * 1000;  // alarm turns on for 1 sec then turns of for 1 sec

int relayPin = 7;
unsigned long heatTime = 3 * 1000;    // turn on nichrome for 3 seconds at a time
unsigned long coolTime = 2 * 1000;    // then turn off for 2 seconds at a time
unsigned long timeCheck = 0;
bool heaterON = false;

// LCD timer interrupt code
const uint16_t t1_comp = 62500;  // 62500 cycles = 1 sec
bool Timer1Flag = 0;
volatile int Timer1seconds = 0;


/*--------------------------------------*/


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

void soundAlarm()
{
  tone(alarmPin, 1000);
  delay(1000);  
}

void ISR_buttonUP()
{
  buttonUP = 1;
}

void ISR_buttonDWN()
{
  buttonDWN = 1;
}

ISR(TIMER1_COMPA_vect)
{
   Timer1Flag = 1;
   Timer1seconds++;
}

/*--------------------------------------*/


// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 4 on the Arduino
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;



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
  pinMode(fanPin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  /* ----- Assigning Interrupt Pins ----- */
  attachInterrupt(digitalPinToInterrupt(2), ISR_buttonUP, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ISR_buttonDWN, RISING);

  /*------------------------------------*/
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
  //printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 12);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  /* --------- Ignore this code it's too ugly --------- */
  /* ----- LCD Display Timer1 code (Refresh Rate) ----- */

  // Reset Timer1 Control Register A
  TCCR1A = 0;

  // Set CTC (Clear Timer/Counter1)
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |= (1 << WGM12);

  // Set to prescaler of 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  // Reset Timer1 and set Compare value
  TCNT1 = 0;
  OCR1A = t1_comp;

  // Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  // Enable global interrupts
  sei();
  
} 

void loop() {
  
  /* ---- Setting the temp up/down ----- */
  
  if( buttonUP && debounceCheck() && setTemp != maxTemp )
  {
    setTemp = setTemp + 0.5;
    upperTempThreshold = setTemp + 5;
    lowerTempThreshold = setTemp - 5;
    
    text = String(setTemp, 1);
    updateLCDsetTemp(text);

  }
  if( buttonDWN && debounceCheck() && setTemp != minTemp )
  {
    setTemp = setTemp - 0.5;
    upperTempThreshold = setTemp + 5;
    lowerTempThreshold = setTemp - 5;
    
    text = String(setTemp, 1);
    updateLCDsetTemp(text);
  }
  

  /* ---- Display the current temp ----- */
  
  if(Timer1Flag && Timer1seconds == 1)   // Timer1seconds sets the refresh rate in seconds
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempInCelsius = sensors.getTempC(insideThermometer);
    updateLCDcurTemp( String(tempInCelsius, 2) );
    Timer1Flag = 0;
    Timer1seconds = 0;
  }
  

  /* ---- Adjusting the current temp --- */

  
  if( tempInCelsius < lowerTempThreshold && heaterON == false && (millis() - timeCheck) > coolTime )
  {    
    timeCheck = millis();
    // heat nichrom w/ 12V @ 3A for heatTime seconds
    
    // turn on relay (to allow current through nichrome)
    digitalWrite(relayPin, HIGH);
    
    // turn on fan
    analogWrite(fanPin, 140);
    delay(100);
    analogWrite(fanPin, 40);

    heaterON = true;
  }
  else if( heaterON == true  && (millis() - timeCheck) > heatTime )
  {    
    timeCheck = millis();
    
    // turn off relay and fan after coolTime seconds
    digitalWrite(relayPin, LOW);
    analogWrite(fanPin, 0);
    
    heaterON = false;
  }
  
  
  else if( 0 ) //curTemp > upperTempThreshold )
  {
    // turn on fan
    analogWrite(fanPin, 140);
    delay(100);
    analogWrite(fanPin, 40);
    
  }
  else if( 0 ) //curTemp < setTemp )
  {
    analogWrite(fanPin, 0);
  }
  
  
  /* ---- Alarms (if too hot/cold) ----- */
  
  if( curTemp < minTemp || curTemp > maxTemp && (millis() - alarmTimeCheck) > alarmTime )
  {
    alarmState = !alarmState;
    alarmTimeCheck = millis();
    
    if(alarmState == 1)
    {
      analogWrite(alarmPin, 128);  
    }
    if(alarmState == 0)
    {
      analogWrite(alarmPin, 0);  
    }
  }
  if( curTemp > minTemp || curTemp < maxTemp )
  {
    analogWrite(alarmPin, 0);
  }  
  

    /* ---- Alarms (if too hot/cold) ----- */
  /*
  if( curTemp < minTemp || curTemp > maxTemp )
  {
    analogWrite(alarmPin, 128);  
    delay(1000);
    analogWrite(alarmPin, 0);  
    delay(1000);
  }
    */
  }
  
