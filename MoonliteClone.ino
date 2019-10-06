// Moonlite-compatible stepper controller
// The original code was written by George Carlson in June 2014, so he deserves full credit for this
// This is a more feature rich and compatible version, yet some aspects are still missing
// 
// The version written by Dr. Lutz Bellmann uses the Arduino Uno Rev.3 Board
// The code has been troubleshot against the original and it should be almost compatible to the Moonlite single focuser
// Exeptions:
// Half Step mode not included
// Range is from 0 to 65535 ticks
// no backlight supported
// this is ment to function remotely and uses serial communication either via bluetooth or USB; if you want to use a DIY handcontroller, you have to extend the code
// tested with kstars-bleeding and indilib 
//
// The center concept at 30000 has been skipped, so the full range from 0 to 65535 can be used, yet the default startposition from empty EEPROM is still 30000
// Additional features:
// Current position, stepping delay(speed), and temperature coefficient are saved to EEPROM and read back after power out
// This enables the focuser to remember the last position in the subsequent imaging night.
// Temperature acquisition is done by 1-wire T sensor Dallas DS18B20
// Temperature can be calibrated by command (not saved to EEPROM, yet)
// 2-point calibration can be done with Temperature coefficient in (ticks/degreeC) and Compensation can be switched on/off
// no delay in movement on temperature readout; conversion commands are executed only in idle condition


#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#define ONE_WIRE_BUS 3  //Sensor DS18B20 am digitalen Pin 3
OneWire oneWire(ONE_WIRE_BUS); //
//Übergabe der OnewWire Referenz zum kommunizieren mit dem Sensor.
DallasTemperature sensors(&oneWire);
int sensorCount;
#define HOME 30000
#define MAXCOMMAND 8
#define SCALE 2 /* Moonlite request Temperature reading *2*/

char offs = 0, coef = 0, inChar, cmd[MAXCOMMAND], param[MAXCOMMAND], line[MAXCOMMAND], tempString[6];
unsigned int hashCmd, Current = HOME, Target = HOME;
long pos;
signed long millisLastMove = 0, DistanceToGo = 0;
bool tempComp = 0, isRunning = 0;
int speed = 2, eoc = 0, idx = 0, minSpeed = 2, maxSpeed = 20, tempTemp;


float wADC, lastTemperature;
int speedTable[16] = {2, 2, 4, 8, 10, 20, 0, 0, 0, 0, 20, 10, 8, 4, 2, 2};
int goTable[16] = { -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};

// Motor connections
int motorPin1 = 7; // Blue - 28BYJ48 pin 1
int motorPin2 = 6; // Pink - 28BYJ48 pin 2
int motorPin3 = 5; // Yellow - 28BYJ48 pin 3
int motorPin4 = 4; // Orange - 28BYJ48 pin 4
// Red - 28BYJ48 pin 5 (VCC)

// lookup table for motor phase control
int StepTable[8] = {0b01001, 0b00001, 0b00011, 0b00010, 0b00110, 0b00100, 0b01100, 0b01000};
int phase = 0;

void forwardstep() {

  Current++;
  if (++phase > 7) phase = 0;
  //stepper1.move(1);
  setOutput(phase);
  for (int i = 0; i < speed >> 1; i++) {
    delay(1);
  }
}

void backwardstep()
{
  Current--;
  if (--phase < 0) phase = 7;

  setOutput(phase);
  for (int i = 0; i < speed >> 1; i++) {
    delay(1);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(StepTable[out], 0));
  digitalWrite(motorPin2, bitRead(StepTable[out], 1));
  digitalWrite(motorPin3, bitRead(StepTable[out], 2));
  digitalWrite(motorPin4, bitRead(StepTable[out], 3));
}

float GetTemp(void)
{
  float temperature = 0;
  wADC = sensors.getTempCByIndex(0);
  //Serial.print(wADC);
  temperature = (wADC * SCALE) + offs;
  // The returned temperature is in degrees Celcius * 2 for benefit on MoonLite drivers
  return (temperature);
}

void Tcompensate(void) {
  int newtarget = 0;
  newtarget = Target + ((tempTemp - lastTemperature) * coef)/2;
  Target = newtarget;
  lastTemperature = tempTemp;
}

signed char shexstr2char(char *line) {
  signed char ret = 0;
  ret = strtol(line, NULL, 16);
  return(ret);    
}

long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}

void clearstate(void)
{
  byte address = 0;
  byte endROM = 0xFF;
  for (address =0; address < endROM; address++)
  {
    EEPROM.write(address, 0xFF);
  }
}

void savestate(void)
{
  byte address = 0;
  byte low, high;
  low = Current&0xFF;
  high = (Current>>8)&0xFF;
  EEPROM.write(address, low);
  address++;
  EEPROM.write(address, high);
  address++;
  EEPROM.write(address, speed);
  address++;
  EEPROM.write(address, coef);
}

void readstate(void)
{
  byte address = 0;
  byte low, high;
  low=EEPROM.read(address);
  address++;
  high=EEPROM.read(address);
  Current = low + ((high << 8)&0xFF00);
  Target = Current;
  if (Current == 0xFFFF){
    Current = HOME;
    Target = HOME;
  }
  address++;
  speed=EEPROM.read(address);
  if (speed == 0xFF)
  {
    speed = 2;
  }
  address++;
  coef = EEPROM.read(address);
  if (coef == 0xFF) {
    coef = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// start of program

void setup()

{
  //clearstate(); uncheck to clear EEPROM if you fiddle with the code
  Serial.begin(9600);
  sensors.begin();
  sensors.requestTemperatures(); 
  readstate();
  //setup the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  //pinMode(RUNLED,OUTPUT); /* yellow LED */
  //pinMode(testPin, OUTPUT);

  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();

}

// Forever Loop
void loop() {
  
  DistanceToGo = long(Target) - long(Current);
  if (!Serial.available()) {
    // run the stepper if there's no pending command and if there are pending movements

    if (isRunning) {

      if (DistanceToGo > 0)
        forwardstep();

      if (DistanceToGo < 0)
        backwardstep();

      millisLastMove = millis(); /* reset idle timer */
    }

    else { /* Check to see if idle time is up */

      if ((millis() - millisLastMove) > 2000) {
        // if so, turn off motor
        digitalWrite(motorPin1, 0);
        digitalWrite(motorPin2, 0);
        digitalWrite(motorPin3, 0);
        digitalWrite(motorPin4, 0);
        savestate();
      }

      if (tempComp) {
        Tcompensate();
        DistanceToGo = Target - Current;
        isRunning = 1;
      }

    }

    if (DistanceToGo == 0) {
      // if motion is complete
      //digitalWrite(RUNLED,LOW);
      isRunning = 0;
    }

  }

  else {

    // read the command until the terminating # character

    while (Serial.available() && !eoc) {
      inChar = Serial.read();
      if (inChar != '#' && inChar != ':') {
        line[idx++] = inChar;

        if (idx >= MAXCOMMAND)
          idx = MAXCOMMAND - 1;
      }

      else {
        if (inChar == '#')
          eoc = 1;
      }
    }

  } // end if (!Serial.available())



  // process the command we got

  if (eoc) {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);

    if (len >= 1)
      strncpy(cmd, line, 2);

    if (len > 2)
      strncpy(param, line + 2, len - 2);

    memset(line, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;



    // the stand-alone program sends :C# :GB# on startup

    hashCmd = (byte(cmd[0]) | (byte(cmd[1]) << 8)); /* combine the two command charaters into an unsigned int */
    switch (hashCmd) {
      // GP command Get current position
      case ('P'<<8 | 'G'):
        pos = Current;
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('T'<<8 | 'G'):
        // GT command Get Temperature
        if (isRunning == 0)
        {
          tempTemp = GetTemp();          
        }
        sprintf(tempString, "%04X", tempTemp);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('I'<<8 | 'G'):
        // GI command 01 if motor running, 00 if not
        if (DistanceToGo != 0)
          Serial.print("01");
        else
          Serial.print("00");
        Serial.print("#");
        break;

      case ('B'<<8 | 'G'):
        // GB command Get current backlight value, always 00
        Serial.print("00#");
        break;

      case ('H'<<8 | 'P'):
        // PH command Find motor home
        Current = HOME;
        isRunning = 1;
        //digitalWrite(RUNLED,HIGH);
        break;
      
      case ('O'<<8 | 'P'):
        // PO command adjust temperature offset
        offs = shexstr2char(param);
        break;
        
      case ('V'<<8 | 'G'):
        // GV command Get software version, always 10
        Serial.print("10#");
        break;

      case ('N'<<8 | 'G'):
        // GN command Get new (target) position
        pos = Target;
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('C'<<8 | 'G'):
        // GC command Get temerature coefficient
        sprintf(tempString, "%02X", coef);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('C'<<8 | 'S'):
        // SC command Set temperature coefficient
        coef = shexstr2char(param);
        savestate();
        break;
        
      case ('D'<<8 | 'G'):
        // GD command Get motor speed
        sprintf(tempString, "%02X", speed);
        Serial.print(tempString);
        Serial.print("#");
        break;

      case ('D'<<8 | 'S'):
        // SD command Set motor speed
        speed = hexstr2long(param);
        if (speed < minSpeed)
          speed = minSpeed;
        if (speed > maxSpeed)
          speed = maxSpeed;
        savestate();
        break;

      case ('H'<<8 | 'G'):
        // GH command Get half step mode, always 00
        Serial.print("00#");
        break;

      case ('P'<<8 | 'S'):
        // SP command Set current position
        pos = hexstr2long(param);
//        if (pos == 0)
//          pos = HOME;
        Current = pos;
        savestate();
        break;

      case ('N'<<8 | 'S'):
        // SN command Set new position
        pos = hexstr2long(param);
//        if (pos == 0)
//          pos = HOME;
        Target = pos;
        break;

      case ('G'<<8 | 'F'):
        // FG command Start motor command
        isRunning = 1;
        break;

      case ('Q'<<8 | 'F'):
        // FQ command Stop motor command
        isRunning = 0;
        break;
        
      case ('C'):
        // C command Start temperature conversion, only if not moving
        if (isRunning == 0)
        {
          sensors.requestTemperatures();
        }
        break;

      case ('+'):
        // + command sets temperature compensation active
        tempComp = 1;
        tempTemp = GetTemp();
        lastTemperature = tempTemp;
        break;

      case ('-'):
        // - command sets temperature compensation off
        tempComp = 0;
        break;
    }

  } // end process command

} // end forever loop
