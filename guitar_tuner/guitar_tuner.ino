//generalized wave freq detection with 38.5kHz sampling rate and interrupts
//by Amanda Ghassaei
//https://www.instructables.com/id/Arduino-Frequency-Detection/

#include <LiquidCrystal_I2C.h>

//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;    //keeps time and sends vales to store in timer[] occasionally
int timer[10];            //sstorage for timing of events
int slope[10];            //storage for slope of events
unsigned int totalTimer;  //used to calculate period
unsigned int period;      //storage for period of wave
byte index = 0;           //current storage index
float frequency;          //storage for frequency calculations
int maxSlope = 0;         //used to calculate max slope as trigger point
int newSlope;             //storage for incoming slope data

//variables for decided whether you have a match
byte noMatch = 0;   //counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 3;  //slope tolerance- adjust this if you need
int timerTol = 10;  //timer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 30;  //raise if you have a very noisy signal

LiquidCrystal_I2C lcd(0x27, 16, 2);

int ok;

void setup() {
  ok = 0;
  lcd.init();       // Initiate the LCD module
  lcd.backlight();  // Turn on the backlight

  Serial.begin(9600);

  pinMode(13, OUTPUT);  //led indicator pin
  pinMode(12, OUTPUT);  //output pin

  cli();  //disable interrupts

  //set up continuous sampling of analog pin 0 at 38.5kHz

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0);  //set reference voltage
  ADMUX |= (1 << ADLAR);  //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);  //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE);                 //enabble auto trigger
  ADCSRA |= (1 << ADIE);                  //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);                  //enable ADC
  ADCSRA |= (1 << ADSC);                  //start ADC measurements

  sei();  //enable interrupts
}

ISR(ADC_vect) {        //when new ADC value ready
  PORTB &= B11101111;  //set pin 12 low
  prevData = newData;  //store previous value
  newData = ADCH;      //get value from A0

  if (prevData < 127 && newData >= 127) {  //if increasing and crossing midpoint
    newSlope = newData - prevData;         //calculate slope

    if (abs(newSlope - maxSlope) < slopeTol) {  //if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;

      if (index == 0) {      //new max slope just reset
        PORTB |= B00010000;  //set pin 12 high
        noMatch = 0;
        index++;                                                                                    //increment index
      } else if (abs(timer[0] - timer[index]) < timerTol && abs(slope[0] - newSlope) < slopeTol) {  //if timer duration and slopes match
        //sum timer values
        totalTimer = 0;

        for (byte i = 0; i < index; i++) {
          totalTimer += timer[i];
        }

        period = totalTimer;  //set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;           //set index to 1
        PORTB |= B00010000;  //set pin 12 high
        noMatch = 0;
      } else {    //crossing midpoint but not match
        index++;  //increment index

        if (index > 9) {
          reset();
        }
      }
    } else if (newSlope > maxSlope) {  //if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;  //reset clock
      noMatch = 0;
      index = 0;  //reset index
    } else {      //slope not steep enough
      noMatch++;  //increment no match counter
      if (noMatch > 9) {
        reset();
      }
    }
  }

  if (newData == 0 || newData == 1023) {  //if clipping
    PORTB |= B00100000;                   //set pin 13 high - turn on clipping indicator led
    clipping = 1;                         //currently clipping
  }

  time++;  //increment timer at rate of 38.5kHz

  ampTimer++;  //increment amplitude timer
  if (abs(127 - ADCH) > maxAmp) {
    maxAmp = abs(127 - ADCH);
  }

  if (ampTimer == 1000) {
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}

void reset() {   //clear out some variables
  index = 0;     //reset index
  noMatch = 0;   //reset match couner
  maxSlope = 0;  //reset slope
}


void checkClipping() {   //manage clipping indicator LED
  if (clipping) {        //if currently clipping
    PORTB &= B11011111;  //turn off clipping indicator led
    clipping = 0;
  }
}


void display() {
  // LOW E
  if (abs(frequency - 82.4) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E<-A  D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("@: Low E in tune");
  }

  if (frequency > 83.4 && frequency < 95) {
    lcd.setCursor(0, 0);
    lcd.print("E<-A  D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                 ");
    lcd.setCursor(0, 1);
    lcd.print("<: Low E is #");
  }

  if (frequency < 81.4) {
    lcd.setCursor(0, 0);
    lcd.print("E<-A  D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                 ");
    lcd.setCursor(0, 1);
    lcd.print("<: Low E is b");
  }


  // A
  if (abs(frequency - 110) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E  A<-D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("@: A in tune");
  }

  if (frequency > 111 && frequency < 130) {
    lcd.setCursor(0, 0);
    lcd.print("E  A<-D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("<: A is #");
  }

  if (frequency < 109 && frequency > 95) {
    lcd.setCursor(0, 0);
    lcd.print("E  A<-D  G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    lcd.setCursor(0, 1);
    lcd.print("<: A is b");
  }

  // D
  if (abs(frequency - 146.8) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D<-G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("@: D in tune");
  }

  if (frequency > 147.8 && frequency < 164) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D<-G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("<: D is #");
  }

  if (frequency < 145.8 && frequency > 130) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D<-G  B  E");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    lcd.setCursor(0, 1);
    lcd.print("<: D is b");
  }

  // G
  if (abs(frequency - 196) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G<-B  E");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("@: G in tune");
  }

  if (frequency > 197 && frequency < 220) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G<-B  E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("<: G is #");
  }

  if (frequency < 195 && frequency > 165) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G<-B  E");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    lcd.setCursor(0, 1);
    lcd.print("<: G is b");
  }

  // B
  if (abs(frequency - 246.9) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B<-E");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("@: B in tune");
  }

  if (frequency > 247.9 && frequency < 280) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B<-E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("<: B is #");
  }

  if (frequency < 245.9 && frequency > 220) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B<-E");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    lcd.setCursor(0, 1);
    lcd.print("<: B is b");
  }

  // High E
  if (abs(frequency - 329.6) < 1) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B->E");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("@ High E in tune");
  }

  if (frequency > 330.9) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B->E");
    lcd.setCursor(0, 1);
    lcd.print("                   ");
    lcd.setCursor(0, 1);
    lcd.print("<: High E is #");
  }

  if (frequency < 328.6 && frequency > 280) {
    lcd.setCursor(0, 0);
    lcd.print("E  A  D  G  B->E");
    lcd.setCursor(0, 1);
    lcd.print("                  ");
    lcd.setCursor(0, 1);
    lcd.print("<: High E is b");
  }
  ok = 1;
}

void loop() {

  checkClipping();

  if (checkMaxAmp > ampThreshold) {
    frequency = 38462 / float(period);  //calculate frequency timer rate/period

    if (frequency > 40 && frequency < 370) {
      Serial.print(frequency);
      Serial.print(" curr hz  -  ");

      lcd.setCursor(0, 1);
      lcd.print("                 ");

      display();
    }
  }

  if (!ok) {
    lcd.setCursor(0, 0);
    lcd.print("Hi :D Pluck a");
    lcd.setCursor(0, 1);
    lcd.print("string!");
  }
}
