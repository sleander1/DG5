 /*  

Kenwood DG5 digital display emulator, V2.0.1 - LED version

Updates made January 2016 by Stephen Leander, KV6O
  - LED version, uses MAX 7219 based, 8 digit, 7-segment display
  - added correction factor (see variable "cf" in the "Frequency measurement variables" section
  - added smoothing/averaging to reduce display jitter
  - code cleaned up and better documented
  - this version uses FreqCount library rather than timer code by Nick Gammon
  - tested with latest Arduino IDE (1.8.2), you will need to get the FreqCount library under "Sketch/Include Library/Manage Libraries.."  
        Search on "FreqCount" and you should find Paul Stoffregen.  Install the latest version.
  - You will need to do the same for the LED Control library, search on "LedControl"

Stephen Leander, KV6O


Original release notes:
********************************************************************************************************
August 22, 2014

Outputs Kenwood Commander commands on the Serial/USB port for a logging program (DXLab's Commander)
No idea if it will work with other programs because I haven't tested anything else!

Got the idea from Todd Harrison's website, Toddfun.com, where he outlined and built an Adrunio DG-5 emulator, with the plans of using this to display
the frequency, just like the DG-5.

http://www.toddfun.com/2013/02/07/arduino-frequency-display-for-kenwood-ts-520s-hf-ham-radio-part-1/


Counter code based on Arduino timer code by Nick Gammon
http://www.gammon.com.au/forum/?id=11504


Emulates a TS-790 for DX Commander, based on input from Dave, AA6YQ:
  
        From Dave AA6YQ, DXLab's author:
        From: dxlab@yahoogroups.com [mailto:dxlab@yahoogroups.com] Sent: Tuesday, August 05, 2014 10:42 PM To: dxlab@yahoogroups.com Subject: [dxlab] Kenwood
        
          >>>If your emulator responds to the ID; command with 
        
        ID007;
        
        >>>Commander will think its controlling a TS-790. The only other commands to which your emulator will then have to respond are IF; and FB;
        
        >>>Note: Commander pays attention to the following bytes of the radio's response to an IF; command:
        
        3-13: VFO A frequency
        29: RX vs TX status
        30: Mode
        31: VFO selection
        33: split status
        
        >>>Operation won't be convenient unless your emulator can correctly report the radio's mode.
        
        
See http://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf for more on Kenwood command set.


8/22/14 - Version 1.0  Prototyped the circuit using 3 TI PLL's (74HC4046A's), a 74HC93 counter for prescaling the HFO, and 74HC153 for selecting the 
signal (VFO, BFO, or HFO) to be counted by the Arduino on pin 5.

8/29/14 Version 1.1 - Added LCD display using 4-bit parrallel display type, added "freq" to have single calculated frequency for serial port and LCD.

9/9/14  Version 1.2 - Added DEBUG0;/DEBUG1; command line functionallity for turning on and off serial debugging

10/11/14 - Version 1.4 (abandoned 1.3, was attempt to make gate time shorter) Added USB/LSB/CW logic using BFO (CAR) frequency.

10/14/14 - Modifications to accomidate the V1.5 board.  Added SelectPinA/B for the new Arduino pins used for the select lines on the 153 (done to free up D2 and D3
            for the later addition of an rotary encoder - these are external Inturrupt pins).  Also had to flip the VFO/HET logic selection, not sure why!
            
10/17/14 - Change the counter gate time to 200ms (up from 100ms) due to the measured frequency being a little high (200-300 Hz) Not an issue on prototype, 
            need to troubleshoot.  Added "splash screen at startup" with adjustable call sign display during operation.

***********************************************************************************************************

1/17/16 - Version 1.6 - removed LCD code and added LED code.

1/17/16 - Version 1.6.1 - Added correction factor option, USB/LSB/CW display on LED, counter gate time back to 100ms

1/18/16 - Version 1.6.2 - added averaging routine to help with display jitter, cleaned up code, added commenting like a good person should.

4/24/16 - Version 1.6.3 - corrected error in calculating correction factor (was being multiplied by 8 for HFO)
*/


#include <LedControl.h>
#include <FreqCount.h>

boolean DEBUG = false;                                       // Debug switch

/*
 LedControl pins for LED display using MAX72XX
 LedControl(dataPin,clockPin,csPin,numDevices)
Arduino pin 8 is connected to the DataIn  - Pin 13 on DG5 shield on LCD2 header
Arduino pin 9 is connected to LOAD (CS)   - Pin 12 on DG5 shield on LCD2 header
Arduino pin 10 is connected to the CLK    - Pin 11 on DG5 shield on LCD2 header
Arduino pin 5 reserved for frequency counter - do not use
*/
LedControl lc=LedControl(8,10,9,2);   

// Averaging variables for smoothing display jitter
const int numReadings = 5;               // number of readings to average over
unsigned long readings[numReadings];      // the readings from the frequency county
int readIndex = 0;                        // the index of the current reading
unsigned long total = 0;                  // the running total
unsigned long average = 0;                // the average

// timer varables
volatile unsigned long timerCounts;
volatile boolean counterReady;

//the booleans are for tracking USB/LSB/CW.  Also tracked by the "mode" char, this is only used to track if the mode has changed at this point, could remove the boolean logic but I am lazy
boolean USB = false;
boolean LSB = false;
boolean CW = false;
char mode = 'U';        // current mode computed from the BFO frequency 
char OldMode = 'U';     // used in test to see if mode has changed

// counting routine variables
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;


//Frequency measurement variables
unsigned long frq = 0;        // From counter
unsigned long vfo = 0;        // holds measured vfo frequency
unsigned long bfo = 0;        // holds measured bfo frequency
unsigned long hfo = 0;        // holds measured hfo frequency (which has been divided by 8 in hardware to get frequency into range that the Arduino can measure)
unsigned long freq = 0;       // calculated frequency
unsigned long FreqOld = 0;    // used in test to see if frequency has changed
signed long cf = 0;           // correction factor in Hz - doesn't correct for ceramic oscillators, like on the UNO board - too drifty! If the display is low, use a negative number to move it up, such as "-100" to bump the display up 100Hz

String inputString = "";         // a string to hold incoming command
boolean stringComplete = false;  // whether the string is complete


void setup () 
  {
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {       //initilize averaging array
    readings[thisReading] = 0;}

 //    FreqCount.begin(1000);
    
  pinMode(4, OUTPUT);      //setup pins used to select vfo/bfo/hfo using 74LS153
  pinMode(6, OUTPUT);

  //Startup LED Display
  //The MAX72XX is in power-saving mode on startup, we have to do a wakeup call
  
  lc.shutdown(0,false);
  // Set the brightness, 0-15 (2 seemed good to me, might need to be brighter thru a lens)
  lc.setIntensity(0,2);
  // and clear the display
  lc.clearDisplay(0);
    
 Serial.begin(9600);   
// reserve 15 bytes for input string for commands
  inputString.reserve(15);

//Display DG5 version on startup
lc.setDigit(0,5,2,true);
lc.setDigit(0,4,0,true);
lc.setDigit(0,3,1,false);

delay(2500);  // wait 2.5 seconds

lc.clearDisplay(0);  // clear LEDs

//FreqCount.begin(100);
   // end of setup
}


//LED display function
void DisplayFreq(long int Frequency) {
 int Freq[6];
for (int i = 5; i >= 0; i--) {
    Freq[i] = Frequency % 10;
    Frequency /= 10;}
 
lc.setDigit(0,7,Freq[0],false);
lc.setDigit(0,6,Freq[1],true);
lc.setDigit(0,5,Freq[2],false);
lc.setDigit(0,4,Freq[3],false);
lc.setDigit(0,3,Freq[4],true);
lc.setDigit(0,2,Freq[5],false);
}

//Serial input function 
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == ';') {                // Look for ";" - this is the Kenwood command terminator.
      stringComplete = true;
    } 
  }
} 


  

  
void loop () 
  {

  for (int x=0; x < 3; x++){                   // Loop thru the 3 signals to count the signals.
         if (x==0) {                            //Select VFO 
             digitalWrite(4, LOW);
             digitalWrite(6, LOW);
         }
          if (x==1) {                           //Select BFO
             digitalWrite(4, HIGH);
             digitalWrite(6, LOW);              
         }
         
         if (x==2) {                            //Select HFO
             digitalWrite(4, LOW);
             digitalWrite(6, HIGH);
         }         
         
delay (5);  //settle time

FreqCount.begin(100);

while (FreqCount.available() == 0); 
     frq = (FreqCount.read())*10;
//    Serial.println(frq);
FreqCount.end();

   

//Correction Factor, if needed.  This is added to each of the 3 frequencies read.

          if (x==0)vfo=frq+cf;                  // load frequency's into vfo, bfo, and hfo.
          if (x==1)bfo=frq+cf;
          if (x==2)hfo=(frq*8)+cf;              //hfo multiplied by 8 to get actual frequency (hardware divided by 8 with 74LS93 to get frequency into readable range (Arduino can't count above 8Mhz) 
          

   

//Compute the actual dial frequency, round and drop least signifigant digits 
freq = ((((hfo-(bfo+vfo))/10)+5)/10);



// use the BFO (CAR) frequency to determine USB/LSB/CW


// In the TS-520S Service manual on pg. 6, it describes the frequencies used by the BFO for the different modes.
// It states, “Frequencies are 3396.5 kHz for USB, 3393.5 kHz for LSB, and 3394.3 kHz (receive) and 3395.0 kHz (transmit) for CW.”
// You might have to tweak these values for your BFO - this is a function that the origional DG5 didn't do.
// If your Arduino's crystal is off, you might be able to fix both using the correction factor (cf) variable in the "Frequency measurement variables" section


if (bfo > 3396000){
  USB = true;
  LSB = false;
  CW = false;
  mode = 'U';
  }
  
else if (bfo < 3393500){
  USB = false;
  LSB = true;
  CW = false;
  mode='L';
  }
  
  else{
  USB = false;
  LSB = false;
  CW = true;
  mode='C';
  }

// Debugging stuff. 

//#ifdef DEBUG                                  // for debugging and monitoring the results on the serial port
if (DEBUG == true){                             // Type "DEBUG1;" (and enter) on the port to enable, "DEBUG0;" to turn off
Serial.print("VFO: ");                          // prints measured VFO/BFO/HFO values, along with the calculated
Serial.print (vfo);                             // frequency, and calculated USB/LSB/CW settings.
Serial.print(" BFO: ");
Serial.print (bfo);
Serial.print(" HFO: ");
Serial.print (hfo);
Serial.print (" FREQ: ");
Serial.print (freq);
Serial.print (" USB: ");
Serial.print (USB);
Serial.print (" LSB: ");
Serial.print (LSB);
Serial.print (" CW: ");
Serial.println (CW);
}
   
// write to LED display
// first, we add the new frequency into an array of X frequencies so we can average

 
  total = total - readings[readIndex];            // subtract the last calculated freq
  readings[readIndex] = freq;                     // load the new freq reading into the array
  total = total + readings[readIndex];            // add the freq to the total
  readIndex = readIndex + 1;                      // advance to the next position in the array
    if (readIndex >= numReadings) {               // if we're at the end of the array...
    readIndex = 0;                                // wrap around to the beginning
  }
  average = total / numReadings;                  // now, calculate the average:
 
 if (average != FreqOld) {DisplayFreq(average);}     // has it changed since the last time we updated the display?  If so, update, if not, skip
 
 FreqOld = average;                               // copy new average to FreqOld so we can test if it's changed the next time we run thru the loop


//  Display the mode on the last position of the LED (far right)
//  if the mode has changed.  Helps prevent unnecessary strobing of the display

if (mode != OldMode) {
        if (USB == true){
        lc.setRow(0,0,0x1c);                      // write a little "u"
      }
      else if (LSB == true){                      
        lc.setChar(0,0,'L',false);                // write a big  "L"
      }
      else if (CW == true){                       // write a little "c"
         lc.setChar(0,0,'C',false);
      }   
}

OldMode = mode;  //copy current mode to OldMode so we can test next time thru the loop

//#endif - end of frequency counting loop

}  

 //  Serial port handeling 
 //  Check serial port to see if we have a command
  if (stringComplete) {
    
    if (inputString == "ID;") {            //ID the radio as a "ID007" - Kenwood TS790. There is no TS-520S. ;-)
         Serial.print("ID007;");
    }
  
     else if (inputString =="DEBUG1;"){        // Turn on debugging when "DEBUG1;" is received.
       DEBUG=true;
     }
     
     else if (inputString =="DEBUG0;"){        // Turn off debugging when "DEBUG0;" is received.
       DEBUG=false;
     }
     
     else if (inputString == "IF;"){
       
       // Output the frequency and mode in the Kenwood "IF;" return
       // Reading strings from flash to save memory, could use a more elegant approach, but this works :-)
       
       
       if (freq > 90000){  // if above 10Mhz set the padding correct for the IF; command
      
       if (USB == true){            
       Serial.print (F("IF000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000020000000;"));  // The "2" is the mode, 
       }
       else if (LSB == true){
       Serial.print (F("IF000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000010000000;"));    // Mode set to 1 for "LSB".
       }
       else if (CW == true){
       Serial.print (F("IF000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000030000000;"));  // The "3" is the mode, CW
     }      
       }
       
       else {             // if below 10Mhz set the padding correct for the IF; command
       if (USB == true){
       Serial.print (F("IF0000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000020000000;"));  // The "2" is the mode, 
       }
       else if (LSB == true){
       Serial.print (F("IF0000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000010000000;"));    // Mode set to 1 for "LSB".
       }
       else if (CW == true){
       Serial.print (F("IF0000"));
       Serial.print (freq);
       Serial.print (F("00000000000000000030000000;"));  // The "3" is the mode, CW         
       }
       }

  }
  }
    // clear the string:
    inputString = "";
    stringComplete = false;

    // end of loop 
  } 
 
