//Versio 1.2, 19.5.2013 || Added PID control for closed loop, but it's hard to tune
#include <EEPROM.h> // This Sketch uses arduinos built in EEPROM
#include <PID_v1.h> // And PID library

/*MAP analog in: 41-1003 (0.2-4.9V) Sensor: MPX4250AP
 //TYPICAL: 51-972 = 20-250kPa
   3      How the display is wired. Numbers represent shift register output numbers
 2[ ]4
   1 
 5[ ]7           These are the characters converted to decimal and listed in a string.                        (4)(7)
   6  o8         eg. "1" in the register outputs is created with output pins 4 and 7 high. In binary this is 00010010 which in dec is 18. (2^5+2^1 = 16+2 = 18) */
//          [n+1]   0    1   2    3    4    5    6    7   8    9  10P   11d  12c  13A  14C  15L 16E  17N  18U 19b  20t  21r  22F  23J 24- 25blank 26m  27i
byte string[28] = {
  126, 18, 188, 182, 210, 230, 238, 50, 254, 246, 248, 158, 140, 250, 108, 76, 236, 122, 94, 206, 200, 136, 232, 22, 128,  0,    170, 2};

// Pins and corresponding pin on arduino
const byte dataPin = 7; // for shift register
const byte latchPin = 9; // for shift register
const byte clockPin = 8; // for shift register
const byte brightnessPin = 11; // to shift registers' output enable
const byte pressurePin = A1; // A1 manifold air pressure sensor is connected here
const byte buzzerPin = 5; // piezo buzzer
const byte solPin = 10; // Wastegate solenoid. This MUST be connected to pin 9 or 10 on Arduino Nano or Uno. Only they can provide 31Hz PWM with analogWrite
const byte buttonPin1 = 3; // upper button on front panel
const byte buttonPin2 = 2; // lower button on front panel
const byte buttonPin3 = 6; // external button for triggering dc scramble
const byte ledPin = 4; // boost scramble indicating led on front panel

// Buttons
byte buttonState1 = 0; // 0 if button is not pressed, 1 if it is, and 2 if button is pressed but needs to be ignored until released and pressed again
byte buttonState2 = 0;
byte buttonState3 = 0;
byte buttonReleased1 = 0; // 1 if button was just released
byte buttonReleased2 = 0;

// Pressure
int pressureIn; // pure signal from MAP sensor 0-1023
byte pressurekPa = 100; // that signal converted to kiloPascals
byte boostTarget[7] = {
  0, 200, 200, 200, 200, 200, 200}; // Memory specific boostTarget in kPa, this will never actually be displayed but (boostTarget - atm) will. EEPROM 10*mem+5
byte oldBoostTarget; // old***'s are used to track if the setting has been changed by user and then is saved to EEPROM
byte a, b, c; // Pressure digits, used to display values correctly with individually controlled 7 segment diplays
byte atm; // normal athmosphere, EEPROM address 4
byte overBoostLimit[7] = {
  0, 200, 200, 200, 200, 200, 200}; // memory specific boost cut limits. EEPROM  address 10*mem+7
byte oldOverBoostLimit = 0;
byte overBoostSet[7] = {
  0, 0, 0, 0, 0, 0, 0}; // Memory specific. 0 = no over boost protection, 1 is opposite. EEPROM address 10*mem+8
byte oldOverBoostSet = 0;
byte overBoost = 0; // 0 if all is ok, 1 if overboost is detected and solenoid needs to be opened
byte boostDown[7] = {
  0, 10, 10, 10, 10, 10, 10};
byte oldBoostDown = 0;
byte boost = 0; // 0 if vacuum and 1 if boost. This tells the solenoid if it needs to work or not
byte obFlag = 0; // used for over boost control
int pressure0;
byte peak = 0;

// Scramble
byte scrambleSec[7] = {
  0, 0, 0, 0, 0, 0, 0}; // Memory specific. Tells how long to keep boost scramble on after trigger button is released. EEPROM 10*mem+4
byte oldScrambleSec;
byte scrambleBoost[7] = {
  0, 0, 0, 0, 0, 0, 0}; // Memory specific. How much boost is added to existing boostTarget[mem] when scramble is active. EEPROM 10*mem+6
byte oldScrambleBoost = 0;

// ClosedLoop
byte dc[7] = {
  0, 10, 10, 10, 10, 10, 10}; // Meomory specific. Amount of dc to be sent to solenoid to achieve boost target. EEPROM 10*mem+1
byte oldDc = 0;
byte gain[7] = {
  0, 0, 0, 0, 0, 0, 0}; // Sets how harsh the dc adjustment is to achieve boost target. EEPROM 10*mem+2
byte oldGain = 0;
int pwm; // This is the final value to send to solenoid

// Mode
byte mainMode = 0; // Keeps track of menus, 0 is main menu and that's why it is default
byte closedLoop[7] = {
  0, 0, 0, 0, 0, 0, 0}; // 1, closed loop is active for memory, 0 means open loop. EEPROM 10*mem+3
byte oldClosedLoop;
byte mem; // 1-6 This tells what memory was last used and is also saved to EEPROM so device remember what memory was used last time and starts from there when turned on again. EEPROM 6
byte oldMem = 7; // This is 7 (larger than max val of mem (=1-6)) to force mainMode 0 to think mem was just changed when device is turned on
byte memDisp = 1; // This is 1 everytime mem is changed to show the number of mem (current memory selected) and it's boost target
byte subMode = 0; // Keeps track of menu #2

// Timers, they keep things up to date and act as timestamps to keep track of when something was done the last time
long Time; //Current time in milliseconds, updated at the beginning of loop()
long debounceTime = 0;
long buttonTime1 = 0;
long buttonTime2 = 0;
long modeTime = 0;
long dispTime = 0;
long memTime = 0;
long solTime = 0;
long scrambleTime = 0;
long vacuumTime = 0;
long kPaTime = 0;

long SerialTime = 0;

// Delays etc
const int modeDuration = 900; // How long to show title of a mode or a menu. Feel free to modify if you feel so
const byte debounceDelay = 30; // time in ms between button check. I recommend keeping this larger than 20ms
const int dispDelay = 50; // Screen refresh every x ms. Too small a value will cause ghosting in slow displays so adjust to your prefrences
const int memDelay = 1000; // how long to show memory setting [mem] and then setting specific boost target. This can also be modified or set to 0 to completely ignore
const int refreshkPa = 100; // how long to wait for next display update of real time boost display on mainMode = 0. If you find your screen changes too fast grow this value

byte dimmer = 0; // keeps track of screen brightness. 0 normal, 1 dimmed, 2 dark. Last setting is saved to EEPROM 7 to retrieve at startup
byte dimmerFlag = 0; // used by the dimmer part of code

// PID specific variables
double Setpoint, Input, Output;
double KpB[7] = {
  10, 10, 10, 10, 10, 10, 10};
double KiB[7] = {
  1, 1, 1, 1, 1, 1, 1};
double KdB[7] = {
  3, 3, 3, 3, 3, 3, 3};
double Kp=1, Ki=0.05, Kd=0.25; //defaults, basically Kx = KxB/10
double oldKpB, oldKiB, oldKdB;

PID SolPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup(){ //////////SETUP

  Serial.begin(115200); // Remove or comment out serial related stuff after testing!! //-----------------------------------------Serial stuff..

  TCCR1B = TCCR1B & 0b11111000 | 0x05; // Set pins 9 and 10 pwm to ~31Hz
  // Set pinModes:
  // Displays pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  // Output pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(solPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(brightnessPin, OUTPUT);
  // Input pins
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  // Pull up for button pins
  digitalWrite(buttonPin1, HIGH);
  digitalWrite(buttonPin2, HIGH);
  digitalWrite(buttonPin3, HIGH);

  analogWrite(solPin, 120); // Test if solenoid is working
  greetings(); // Scroll "MIATA". Check greetings() to modify your own message
  analogWrite(solPin, 0); // Stop solenoid test, this is currently in greetings() to shorten the test

  // Retrieve settings from EEPROM
  atm = EEPROM.read(4); // atmospheric pressure
  mem = EEPROM.read(6); // last memory slot that was used (1-6)
  oldMem = mem;
  dimmer = EEPROM.read(7); // display brightness. Comment out if you want to start with full brightness every time
  //if(dimmer > 2)dimmer = 0; // REMOVE after first time changing the brightness if you like
  peak = EEPROM.read(5);
  memTime = millis(); //prepare to shoe mem display
  setSettings(); // Retrieve memory slot (mem) specific settings

    //initialize PID variables
  Input = 0;
  Setpoint = 0;

  //turn the PID off
  SolPID.SetMode(MANUAL);
  SolPID.SetSampleTime(20);

}
void greetings(){ // This function scrolls a message on the display at every startup. Note that the solenoid is tested during the scrolling
  int s = 300; // how long each scroll transition takes. Change to modify scroll speed
  disp(25,25,26,2); //"  M" // You can make your own message here using the characters in "byte string[X] = ..". You can add new characters there if you want to
  delay(s);
  disp(25,26,1,2); //" MI"
  delay(s);
  disp(26,1,13,2); //"MIA"
  delay(s);
  //2analogWrite(solPin, 0); // Stop solenoid test
  disp(1,13,20,2); //"IAT"
  delay(s);
  disp(13,20,13,2); //"ATA"
  delay(s);
  disp(20,13,25,2); //"TA "
  delay(s);
  disp(13,25,25,2); //"A  "
  delay(s);
  disp(25,25,25,2); //"   "
  delay(s/2);
}

void loop(){ ////////                                       *LOOP*
  Time = millis(); // Store current time
  mode(); // user interface and settings are almost all here

    if((Time - debounceTime) > debounceDelay){ // check if it's time to check buttonstates
    debounceTime = Time; // save time of the last debounce
    getButton(); // retrieve button states
  }

  pressureIn = analogRead(pressurePin); // Input from MAP sensor
  pressurekPa = map(pressureIn, 51, 972, 20, 250); // Convert signal to kPa
  if(pressureIn < 51)pressurekPa = 20; // The 
  if(pressureIn > 972)pressurekPa = 250;

  if(pressurekPa > (atm+7)){ // If there's boost the solenoid is made active
    boost = 1;
    vacuumTime = Time;
  }
  if((Time - vacuumTime)>1000 && boost == 1){
    buttonState3 = 0; // can't remember why this is here. It disables scramble boost when boost drops
    boost = 0; // If there's vacuum for 1sec solenoid is made inactive
    SolPID.SetMode(MANUAL);
    Output=0;
    Serial.println("VACUUM!!!");
  }

  if(overBoostSet[mem] == 1){ // If current mem has boost cut set on:
    if(pressurekPa >= overBoostLimit[mem] && buttonState3 == 0){
      if(obFlag < 10){
        obFlag++; // if obLimit reached put a flag on
      }
      scrambleTime = (Time - scrambleSec[mem]*1000);
    }
    if((pressurekPa > (overBoostLimit[mem] + scrambleBoost[mem])) && (buttonState3 == 1) && (obFlag < 10))obFlag++;
    if((pressurekPa < overBoostLimit[mem]) && (obFlag > 0) && (overBoost == 0))obFlag--;
    if(((pressurekPa + 30) < overBoostLimit[mem]) && obFlag > 0)obFlag--;
    if(obFlag == 10){
      overBoost = 1;
      analogWrite(buzzerPin, 155);
      analogWrite(solPin, 0);
    }
    else if (obFlag == 0){
      overBoost = 0;
      analogWrite(buzzerPin, 0);
    }
  }
  else overBoost = 0;

  pressure0 = boostTarget[mem]; // save initial boostTarget as pressure0 that is modified when scramble is on
  if(buttonState3 == 1){ // if external scramble button pressed
    pressure0 = (boostTarget[mem]+scrambleBoost[mem]);
    scrambleTime = Time;
    digitalWrite(ledPin, HIGH); // scramble led on
  }
  else if((Time - scrambleTime) < scrambleSec[mem]*1000 && Time > scrambleSec[mem]*1000){ // hold scramblle on for (scrambleSec[mem]) seconds
    pressure0 = (boostTarget[mem]+scrambleBoost[mem]);
  }
  else(ledPin, LOW); // scramble has ended so turn led off
  //PID
  Setpoint = pressure0;
  Input = pressurekPa;
  SolPID.SetTunings(Kp, Ki, Kd); // Send new values for PID
  SolPID.Compute();
  //
  if((Time - solTime) > 10){ // Refresh solenoid every 10ms (twice as fast as PID sampletime)
    if(pressurekPa > peak){ // store peak pressure with this interval too
      peak = pressurekPa;
      EEPROM.write(5, peak);
    }
    solTime = Time;
    solenoid(); // This is the function that controls the solenoid
  }
  if((Time-SerialTime)>500){
    SerialTime = Time;
    Serial.print("Kp: ");
    Serial.print(SolPID.GetKp());
    Serial.print("  Ki: ");
    Serial.print(SolPID.GetKi());
    Serial.print("  Kd: ");
    Serial.println(SolPID.GetKd());
    Serial.print("Pressure: ");
    Serial.print(pressurekPa);
    Serial.print("  PWM ");
    Serial.println(pwm);
  }
  delay(1); //// Refresh about 1000 times per second
} ////////                                                                                          *END OF LOOP*

void mode(){
  switch(mainMode){ 
  case 0: // main display, shows current boost in kPa
    if(memDisp == 1){
      if((Time - memTime) < (2*memDelay)){
        if((Time - memTime) < memDelay){ // For first second show mem (1-6)
          disp(25,25,mem,0);
          //Serial.println(mem);  //-----------------------------------------Serial stuff..
        }
        else{
          getabc(boostTarget[mem] - atm); // For second second show boosttarget for mem
          disp(a,b,c,1);
        }
      }
      else memDisp = 0;
    }
    else{ // when mem stuff done  
      if(buttonState2 == 1 && buttonState1 == 0){ // Press lower button(2) for 2 secs. Go to "main set up" menu (mainMode 1-3, open/closed loop, dc and gain)
        if((Time - buttonTime2) > 2000){
          mainMode = 1;
          buttonState2 = 2; // ignore button until released and pressed again
          modeTime = Time; // Timestamp for when mode was entered to count how lng to show mode title
        }
      }
      if(buttonState1 == 1 && buttonState2 == 0){ // Upper button for 2 secs, move to menu 2
        if((Time - buttonTime1) > 2000){
          mainMode = 6; // button2 pressed for 2 seconds, change to mainMode = 6, Set up 2
          buttonState1 = 2; // ignore button until released and pressed again
        }
      }
      if((pressurekPa-atm) < 0) pressurekPa = atm;
      if((Time-kPaTime)>refreshkPa){ // avoid boost scrolling too fast on display
        kPaTime = Time;
        getabc(pressurekPa-atm); // convert boost to individual digits. eg. 234 --> a=2, b=3, c=4
        disp(a,b,c,1); // (x,y,z,1) "1" means show decimal point
      }
    } // The following can be done while memDisp = 1
    if(buttonState2 == 1 && buttonState1 == 1){ // Press both buttons simultaneously to set display brightness
      dimmer++;
      if(dimmer > 2) dimmer = 0;
      EEPROM.write(7, dimmer);
      buttonState1 = 2; // ignore buttons until released and pressed again
      buttonState2 = 2;
    }
    if(buttonReleased1 == 1 && ((Time - buttonTime1) < 500)){ // Select memory setting 1-6
      mem++;
      if(mem>6) mem = 1;
      buttonReleased1 = 0;
    }
    if(buttonReleased2 == 1 && ((Time - buttonTime2) < 500)){
      mem--;
      if(mem<1) mem = 6;
      buttonReleased2 = 0;
    }
    if(mem != oldMem){ // if memory setting changed, store it and show setting number and boost target
      EEPROM.write(6, mem); // This will be written everytime mem is changed or device turned on. Consider changing address after circa 100,000 writes
      oldMem = mem;
      setSettings();
      memDisp = 1;
      memTime = Time;
    }
    break;  
  case 1: // Solenoid setup
    if(closedLoop[mem] == 0) disp(0,10,16,0); // "OPE" for open loop
    if(closedLoop[mem] == 1) disp(10,27,11,0); // (14,15,0,0,) "CLO" for closed loop/ currently shows "Pid"
    if(buttonReleased1 == 1){
      closedLoop[mem] = 0;
      buttonReleased1 = 0;
    }
    if(buttonReleased2 == 1){
      closedLoop[mem] = 1;
      buttonReleased2 = 0;
    }
    if(buttonState1 == 1 && buttonState2 == 1){
      if(closedLoop[mem] != oldClosedLoop){
        EEPROM.write((mem*10+3), closedLoop[mem]);
        oldClosedLoop = closedLoop[mem];
      }
      if(closedLoop[mem] == 0){
        mainMode = 2; // dc
      }
      else mainMode = 13; //PID
      buttonState1 = 2;
      buttonState2 = 2;
      modeTime = Time;
    }
    break;
  case 2: // duty cycle
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(25, 11, 12, 0); // " dc"
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && dc[mem]<255){
        dc[mem]++;
        buttonReleased1 = 0; 
      }
      if(buttonState1 == 1 && buttonState2 == 0 && dc[mem]<100 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        dc[mem]++;
      }
      if(buttonReleased2 == 1 && dc[mem]>0){
        dc[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && dc[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        dc[mem]--;
      }
      getabc(dc[mem]);
      disp(a,b,c,0);
      if(buttonState1 == 1 && buttonState2 == 1){
        if(dc[mem] != oldDc){
          EEPROM.write((mem*10+1), dc[mem]);
          oldDc = dc[mem];
        }
        mainMode = 0; // 3 to jump to gain, 12 to boost down
        buttonState1 = 2;
        buttonState2 = 2;
        //modeTime = Time;
      }
    }
    break;
  case 4:
    disp(5,16,20,0); // atmosphere setup "SEt"
    if(buttonState2 == 1 && buttonState1 == 1){ // set atmospeheric pressure
      atm = pressurekPa;
      EEPROM.write(4, atm);
      buttonState1 = 2;
      buttonState2 = 2;
      mainMode = 6;
    }
    if(buttonReleased1 == 1 || buttonReleased2 == 1){ // if only one button pressed briefly return to previous screen
      mainMode = 6;
      buttonReleased1 = 0;
      buttonReleased2 = 0;
    }
    break;
  case 5:
    if((Time - modeTime) < modeDuration){
      disp(20,21,9,0); // boost target "trg"
    }
    else{
      if(buttonReleased1 == 1 && boostTarget[mem]<250){
        boostTarget[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && boostTarget[mem]<250 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        boostTarget[mem]++;
      }
      if(buttonReleased2 == 1 && boostTarget[mem]>120){
        boostTarget[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && boostTarget[mem]>120 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        boostTarget[mem]--;
      }
      getabc(boostTarget[mem] - atm);
      disp(a,b,c,1);
      if(buttonState1 == 1 && buttonState2 == 1){
        if(boostTarget[mem] != oldBoostTarget){
          EEPROM.write((mem*10+5), boostTarget[mem]);
          oldBoostTarget = boostTarget[mem];
        }
        mainMode = 6;
        buttonState1 = 2;
        buttonState2 = 2;
      }
    }
    break;
  case 6:
    switch(subMode){
    case 0:
      disp(5,12,21,0); // "Scr" scramble
      if(buttonReleased1 == 1){
        subMode = 5;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 1;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 7;
        buttonState1 = 2;
        buttonState2 = 2;
        modeTime = Time;
      }
      break;
    case 1:
      disp(20,21,9,0); // "trg" target
      if(buttonReleased1 == 1){
        subMode = 0;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 2;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 5; // boost target setup
        buttonState1 = 2;
        buttonState2 = 2;
        modeTime = Time;
      }
      break;
    case 2:
      disp(13,20,26,0); // "Atm"
      if(buttonReleased1 == 1){
        subMode = 1;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 3;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 4; // atm setup
        buttonState1 = 2;
        buttonState2 = 2;
        modeTime = Time;
      }  
      break;
    case 3:
      disp(14,18,20,0); // "CUt" for over boost cut
      if(buttonReleased1 == 1){
        subMode = 2;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 4;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 9; // over boost setup
        buttonState1 = 2;
        buttonState2 = 2;
      }
      break;
    case 4:
      disp(10,16,13,0); // "PEA"
      if(buttonReleased1 == 1){
        subMode = 3;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 5;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 11; // peak
        buttonState1 = 2;
        buttonState2 = 2;
      } 
      break;
    case 5:
      disp(16,5,14,0); // "ESC"
      if(buttonReleased1 == 1){
        subMode = 4;
        buttonReleased1 = 0;
      }
      if(buttonReleased2 == 1){
        subMode = 0;
        buttonReleased2 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 1){
        mainMode = 0; // return to start
        buttonState1 = 2;
        buttonState2 = 2;
        subMode = 0;
      } 
      break;
    } // submode ends
    break; // end of menu 2
  case 7:
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(5, 16, 14, 0); // "SEC"
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && scrambleSec[mem]<100){
        scrambleSec[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && scrambleSec[mem]<100 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        scrambleSec[mem]++;
      }
      if(buttonReleased2 == 1 && scrambleSec[mem]>0){
        scrambleSec[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && scrambleSec[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        scrambleSec[mem]--;
      }
      getabc(scrambleSec[mem]);
      disp(a,b,c,0);
    }
    if(buttonState1 == 1 && buttonState2 == 1){
      if(scrambleSec[mem] != oldScrambleSec){
        EEPROM.write((mem*10+4), scrambleSec[mem]);
        oldScrambleSec = scrambleSec[mem];
      }
      mainMode = 8;
      buttonState1 = 2;
      buttonState2 = 2;
      modeTime = Time;
    }
    break;
  case 8: 
    if((Time - modeTime) < modeDuration){ // scrambleBoost
      disp(13, 11, 11, 0); // "Add"
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && scrambleBoost[mem]<30){
        scrambleBoost[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && scrambleBoost[mem]<30 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        scrambleBoost[mem]++;
      }
      if(buttonReleased2 == 1 && scrambleBoost[mem]>0){
        scrambleBoost[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && scrambleBoost[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        scrambleBoost[mem]--;
      }
      getabc(scrambleBoost[mem]);
      disp(a,b,c,1);
    }
    if(buttonState1 == 1 && buttonState2 == 1){
      if(scrambleBoost[mem] != oldScrambleBoost){
        EEPROM.write((mem*10+6), scrambleBoost[mem]);
        oldScrambleBoost = scrambleBoost[mem];
      }
      mainMode = 6;
      buttonState1 = 2;
      buttonState2 = 2;
    }
    break;
  case 9:
    if(overBoostSet[mem] == 0) disp(0,22,22,0); // "OFF"
    if(overBoostSet[mem] == 1) disp(25,0,17,0); // " ON"
    if(buttonReleased1 == 1){
      overBoostSet[mem] = 1;
      buttonReleased1 = 0;
    }
    if(buttonReleased2 == 1){
      overBoostSet[mem] = 0;
      buttonReleased2 = 0;
    }
    if(buttonState1 == 1 && buttonState2 == 1){
      if(overBoostSet[mem] != oldOverBoostSet){
        EEPROM.write((mem*10+8), overBoostSet[mem]);
        oldClosedLoop = closedLoop[mem];
      }
      if(overBoostSet[mem] == 0) mainMode = 6;
      else{
        mainMode = 10;
        modeTime = Time;
      }
      buttonState1 = 2;
      buttonState2 = 2;
    }
    break;
  case 10:
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(15, 1, 26, 0); // "LIm"
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && overBoostLimit[mem]<250){
        overBoostLimit[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && overBoostLimit[mem]<250 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        overBoostLimit[mem]++;
      }
      if(buttonReleased2 == 1 && overBoostLimit[mem]>0){
        overBoostLimit[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && overBoostLimit[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        overBoostLimit[mem]--;
      }
      getabc(overBoostLimit[mem]-atm);
      disp(a,b,c,1);
    }

    if(buttonState1 == 1 && buttonState2 == 1){
      if(overBoostLimit[mem] != oldOverBoostLimit){
        EEPROM.write((mem*10+7), overBoostLimit[mem]);
        oldOverBoostLimit = overBoostLimit[mem];
      }
      mainMode = 6;
      buttonState1 = 2;
      buttonState2 = 2;
    }
    break;
  case 11:
    getabc(peak-atm);
    disp(a,b,c,1); // show peak pressure
    if(buttonReleased1 == 1 || buttonReleased2 == 1){ // if only one button pressed briefly return to previous screen
      mainMode = 6;
      buttonReleased1 = 0;
      buttonReleased2 = 0;
    }
    if(buttonState1 == 1 && buttonState2 == 1){ // if both buttons pressed reset the peak pressure
      peak = 0;
      EEPROM.write(5, peak);
      mainMode = 6;
      buttonState1 = 2;
      buttonState2 = 2;
    }
    break;
    /*case 12:
     if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
     disp(25, 19, 11, 0); // " bd" boostdown
     }else{ // when title displayed move to mode itself
     if(buttonReleased2 == 1 && boostDown[mem]<80){
     boostDown[mem]++;
     buttonReleased2 = 0;
     }
     if(buttonState1 == 1 && buttonState2 == 0 && boostDown[mem]>0 && (Time - buttonTime1) > 1000){
     buttonTime1 += 100;
     boostDown[mem]--;
     }
     if(buttonReleased1 == 1 && boostDown[mem]>0){
     boostDown[mem]--;
     buttonReleased1 = 0;
     }
     if(buttonState2 == 1 && buttonState1 == 0 && boostDown[mem]<80 && (Time - buttonTime2) > 1000){
     buttonTime2 += 100;
     boostDown[mem]++;
     }
     getabc(boostDown[mem]);
     disp(24,b,c,1);
     }
     if(buttonState1 == 1 && buttonState2 == 1){ // go to main screen and store value to EEPROM
     if(boostDown[mem] != oldBoostDown){
     EEPROM.write((mem*10+9), boostDown[mem]);
     oldBoostDown = boostDown[mem];
     }
     mainMode = 0;
     buttonState1 = 2;
     buttonState2 = 2;
     }
  break;*/
  case 13: // Kp
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(25, 25, 10, 0); // "  P" Kp
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && KpB[mem]<254){
        KpB[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && KpB[mem]<254 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        KpB[mem]++;
      }
      if(buttonReleased2 == 1 && KpB[mem]>0){
        KpB[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && KpB[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        KpB[mem]--;
      }
      Kp = (KpB[mem]/100);
      getabc(KpB[mem]);
      disp(a,b,c,1);
      if(buttonState1 == 1 && buttonState2 == 1){ // go to main screen and store value to EEPROM
        if(KpB[mem] != oldKpB){
          EEPROM.write((mem*10+101), KpB[mem]);
          oldKpB = KpB[mem];
        }
        mainMode = 14;
        buttonState1 = 2;
        buttonState2 = 2;
        modeTime = Time;
      }
    }
    break;
  case 14: // Ki
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(25, 25, 27, 0); // "  i" Ki
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && KiB[mem]<254){
        KiB[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && KiB[mem]<254 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        KiB[mem]++;
      }
      if(buttonReleased2 == 1 && KiB[mem]>0){
        KiB[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && KiB[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        KiB[mem]--;
      }
      Ki = (KiB[mem]/100);
      getabc(KiB[mem]);
      disp(a,b,c,1);
      if(buttonState1 == 1 && buttonState2 == 1){ // go to main screen and store value to EEPROM
        if(KiB[mem] != oldKiB){
          EEPROM.write((mem*10+102), KiB[mem]);
          oldKiB = KiB[mem];
        }
        mainMode = 15;
        buttonState1 = 2;
        buttonState2 = 2;
        modeTime = Time;
      }
    }
    break;
  case 15: // Kd
    if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
      disp(25, 25, 11, 0); // "  d" Kd
    }
    else{ // when title displayed move to mode itself
      if(buttonReleased1 == 1 && KdB[mem]<254){
        KdB[mem]++;
        buttonReleased1 = 0;
      }
      if(buttonState1 == 1 && buttonState2 == 0 && KdB[mem]<254 && (Time - buttonTime1) > 1000){
        buttonTime1 += 100;
        KdB[mem]++;
      }
      if(buttonReleased2 == 1 && KdB[mem]>0){
        KdB[mem]--;
        buttonReleased2 = 0;
      }
      if(buttonState2 == 1 && buttonState1 == 0 && KdB[mem]>0 && (Time - buttonTime2) > 1000){
        buttonTime2 += 100;
        KdB[mem]--;
      }
      Kd = (KdB[mem]/100);
      getabc(KdB[mem]);
      disp(a,b,c,1);
      if(buttonState1 == 1 && buttonState2 == 1){ // go to main screen and store value to EEPROM
        if(KdB[mem] != oldKdB){
          EEPROM.write((mem*10+103), KdB[mem]);
          oldKdB = KdB[mem];
        }
        mainMode = 0;
        buttonState1 = 2;
        buttonState2 = 2;
        //modeTime = Time;
      }
    }
    break;
    /*case X: //Example, explains how to add more functions. You can add a new submode that changes mainMode to X to get here (substitute X with free number)
     if((Time - modeTime) < modeDuration){ // show mode title (dc, gain, etc.) for the lenght of "modeduration" millisec.
     disp(6, 13, 1, 0); // "GAI", choose numbers that represent wanted symbol from "byte string[...] = {..}". Last 0 means no dots, 1 would mean dot in first segment, 3 means dot in second segment.
     }else{ // when title has displayed for the time of "modeDuration" move to mode itself
     if(buttonReleased1 == 1){ // What to do when upper button is pressed. This could change variable or change the subMode like in case 6:
     // Do something, like increase a variable by 1
     f++;
     }
     if(buttonReleased2 == 1){ // What to do when lower button is pressed
     // Do something
     f--;
     }
     getabc(f); // Show the current state of f by first separating the digits to a, b and c
     disp(a,b,c,0); // send these digits to display
     if(buttonState1 == 1 && buttonState2 == 1){ // What to do when both buttons pressed simultaneously. For example save variable and return to main screen or other function
     if(f != oldF){ // check if varable has changed before saving
     EEPROM.write(f, 512); // save the variable (what to write, where to write)
     oldGain = gain[mem]; // update oldF to avoid writing to EEPROM if not necessary
     }
     // other thing to do when both buttons pressed
     mainMode = Y; // move to mainmode Y next
     buttonState1 = 2; // reset buttonstates, important
     buttonState2 = 2;
     modeTime = Time; // reset modeTime, so next modes title will be diplayed
     }
     }
     break;*/
  } // end of switch
} // end of mode()

void solenoid(){ // most important functional part. Here the true dc of solenoid is decided and axcecuted
  if(closedLoop[mem] == 0 && overBoost == 0 && boost == 1){ // open loop is used
    pwm = dc[mem];// convert dc to pwm. 0 = no signal to solenoid at all, 255 = solenoid closed all the way
    analogWrite(solPin, pwm); // dc is banged to solenoid
    SolPID.SetMode(MANUAL);
  }
  if(closedLoop[mem] == 1 && overBoost == 0 && boost == 1){ // closed loop is used, so adjustments to dc are being made with PID
    //if(((pressure0 - pressurekPa) > boostDown[mem]) && (pressure0 > pressurekPa)){
    //SolPID.SetMode(MANUAL);
    //pwm = 255; // target is not even near --> shut solenoid completely to gain boost
    //Output = 80;
    //} else{
    SolPID.SetMode(AUTOMATIC); //PID ON
    pwm = Output;
    //}
    analogWrite(solPin, pwm); // send new pwm to solenoid
  }// end of closedloop = 1

  if(overBoost == 1){
    analogWrite(solPin, 0); // if overboost detected, all above is ignored and solenoid opened
    Output = 0;
    SolPID.SetMode(MANUAL);
  }
  if(boost == 0){
    analogWrite(solPin, 0); // if there's vacuum no need, for solenoid to work, all above is ignored and solenoid opened
    Output = 0;
    SolPID.SetMode(MANUAL);
  }
}

void setSettings(){ // this function is excecuted everytime a new memory "place" (one of six memory) is selected. It retrieves the memory specific settings from EEPROM
  dc[mem] = EEPROM.read(10*mem+1);
  gain[mem] = EEPROM.read(10*mem+2);
  closedLoop[mem] = EEPROM.read(10*mem+3);
  scrambleSec[mem] = EEPROM.read(10*mem+4);
  boostTarget[mem] = EEPROM.read(10*mem+5);
  scrambleBoost[mem] = EEPROM.read(10*mem+6);
  overBoostLimit[mem] = EEPROM.read(10*mem+7);
  overBoostSet[mem] = EEPROM.read(10*mem+8);
  boostDown[mem] = EEPROM.read(10*mem+9);
  KpB[mem] = EEPROM.read(10*mem+101);
  KiB[mem] = EEPROM.read(10*mem+102);
  KdB[mem] = EEPROM.read(10*mem+103);

  /*if(dc[mem] > 100)dc[mem] = 10; // REMOVE after scrolling through all memories!
   if(gain[mem] > 100)gain[mem] = 0; // REMOVE after scrolling through all memories!
   if(closedLoop[mem] > 1)closedLoop[mem] = 0; // REMOVE after scrolling through all memories!
   if(scrambleSec[mem] > 100)scrambleBoost[mem] = 0; // REMOVE after scrolling through all memories!
   if(boostTarget[mem] > 250)boostTarget[mem] = 120; // REMOVE after scrolling through all memories!
   if(boostTarget[mem] < 120)boostTarget[mem] = 120; // REMOVE after scrolling through all memories!
   if(scrambleBoost[mem] > 200)scrambleBoost[mem] = 0; // REMOVE after scrolling through all memories!
   if(overBoostLimit[mem] > 250)overBoostLimit[mem] = 250; // REMOVE after scrolling through all memories!
   if(overBoostSet[mem] > 1)overBoostSet[mem] = 0; // REMOVE after scrolling through all memories!
   if(boostDown[mem] > 30)boostDown[mem] = 10; // REMOVE after scrolling through all memories!*/  //Ser
  if(KpB[mem]==255)KpB[mem] = 10;
  if(KiB[mem]==255)KiB[mem] = 1;
  if(KdB[mem]==255)KdB[mem] = 3;

  Kp = (KpB[mem]/100);
  Ki = (KiB[mem]/100);
  Kd = (KdB[mem]/100);

  oldDc = dc[mem];
  oldGain = gain[mem];
  oldClosedLoop = closedLoop[mem];
  oldScrambleSec = scrambleSec[mem];
  oldBoostTarget = boostTarget[mem];
  oldScrambleBoost = scrambleBoost[mem];
  oldOverBoostLimit = overBoostLimit[mem];
  oldOverBoostSet = overBoostSet[mem];
  oldBoostDown = boostDown[mem];
  oldKpB = KpB[mem];
  oldKiB = KiB[mem];
  oldKdB = KdB[mem];
}

void getabc(byte p){ // this function separates numbers from it's input. eg 125kpa --> a=1, b=2, c=5. a, b and c are later sent to disp() function to show on display
  if(p > 0){
    if(p < 100){ // 0 < p < 100
      if(p<10){ // 0 < p < 10
        a=0;
        b=0;
        c=p;
      }
      else{ // 10 <= p < 100
        a = 0;
        b = p/10;
        c = p - 10*b;
      }
    }
    else{ // p >= 100
      a = p/100;
      b = (p-100*a)/10;
      c = p-100*a-10*b;
    }
  }
  else{ // if p is zero or less. Ability to show negative numbers could be added here
    a=0;
    b=0;
    c=0;
  }
}

void getButton(){ // this function updates button states and keeps track of every buttons actions
  if(digitalRead(buttonPin1) == LOW && buttonState1 == 0){ // button 1 just pressed
    buttonState1 = 1; // button pressed
    buttonTime1 = Time; 
  } // timestamp when it was pressed
  if(digitalRead(buttonPin2) == LOW && buttonState2 == 0){ // button 2 just pressed
    buttonState2 = 1;
    buttonTime2 = Time; 
  }
  if(digitalRead(buttonPin1) == HIGH){ // button 1 is released etc.
    buttonReleased1 = 0; // reset buttonreleased
    if(buttonState1 == 1) buttonReleased1 = 1;
    buttonState1 = 0; 
  } // reset buttonstate
  if(digitalRead(buttonPin2) == HIGH){
    buttonReleased2 = 0;
    if(buttonState2 == 1) buttonReleased2 = 1;
    buttonState2 = 0; 
  }
  if(digitalRead(buttonPin3) == LOW){ // button 3 pressed
    buttonState3 = 1; // button pressed
  }
  else buttonState3 = 0;
}

void disp(byte x, byte y, byte z, byte dec){ // This function controls the display
  if((Time - dispTime) > dispDelay || dec == 2){ // update display only every 50ms (for dispdelay = 50, 20Hz, can be changed)
    if(dimmer == 0)analogWrite(brightnessPin, 230);
    if(dimmer == 1)analogWrite(brightnessPin, 170);
    if(dimmer == 2)analogWrite(brightnessPin, 0);
    dispTime = Time; // Keep track when display was updated last time
    if(dec==1) x=((string[x])+1); // if dec = 1 a decimal point is added. This is done by adding 1 to segments output value. eg string[10] = "P" = 248 || 248+1=249 = "P."
    else x=string[x]; // if no decimal is added x is just string[x]
    if(dec==3) y=((string[y])+1); // same thing except dec = 3 results the decimal point being in second segment instead of first
    else y=string[y];

    digitalWrite(latchPin, LOW); // prepare shift register to receive new bytes
    shiftOut(dataPin, clockPin, LSBFIRST, string[z]); // third
    shiftOut(dataPin, clockPin, LSBFIRST, y); // second
    shiftOut(dataPin, clockPin, LSBFIRST, x); // this will show on first segment
    digitalWrite(latchPin, HIGH); // tell shift register to update new pin states (update screen)
  }
}

