
// Super size PPA with 6 EMS

//********************** PHYSICAL PARAMETERS **************************
//const float tf=0.70;
//const int offset=1;
const int interbeam_dist_mm=30;
const float interbeam_const=interbeam_dist_mm*1000.0;
const int beam2em_dist_mm=30;
const int em_width=15;
const float ratio_IRPS2EM_interbeam=beam2em_dist_mm/interbeam_dist_mm;
const float ratio_em_width_interbeam=em_width/interbeam_dist_mm;
const float em_inter_clicks=2*em_width/interbeam_dist_mm;
//const float em_inter_clickstf=tf*2*em_width/interbeam_dist_mm;
const float beam_em_clicks=2*interbeam_dist_mm/beam2em_dist_mm;
//*********************************************************************

float em_inter_clickstf;
float tf=0.75;
int offset=1200;
bool tf_or_offset=0;

//********************** SCREEN ***************************************
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 8
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
const int normal=0;
//*********************************************************************

//********************** THROTTLE PARAMETERS **************************
//THROTTLE
struct controller_struct
{  int value;
   int switch_status;
};
float ThrottleValue = 0;  // variable to store the value coming from the sensor

//throttle variables
int throttle=0;
int last_throttle=0;
int irps_adjust=3;
int old_value=40;

//Set these values manually.  They are derived in parts in range 0-1023
const int low_no_switch=434;
//const int high_no_switch=856;
const int high_no_switch=750;
const int low_a=61;
const int high_a=201;
const int low_b=860;
const int high_b=959;
const int low_both=330;
const int high_both=414;
const int a=1;
const int b=2;
const int both=3;
const int none=0;
const int error=5;
const int min_range=1;
const int max_range=80;
//*********************************************************************

struct controller_struct returnController(int sensorValue) {
int y;
int switch_status;
struct controller_struct temp_controller;
if (sensorValue >= low_no_switch && sensorValue <= high_no_switch) {
// Serial.println ("nothing is pressed");
 y = map(sensorValue, low_no_switch, high_no_switch, min_range, max_range);
 switch_status=none;
}
else if (sensorValue >= low_a && sensorValue <= high_a) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_a, high_a, min_range,max_range);
 switch_status=a; 
}
 else if (sensorValue >= low_b && sensorValue <= high_b) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_b,high_b, min_range, max_range);
 switch_status=b;  
}
else if (sensorValue >= low_both && sensorValue <= high_both) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_both, high_both, min_range, max_range);
 switch_status=both; 
}
else {
 Serial.println ("Recalibrate throttle");
 y = 0;
 switch_status=error; 
}
temp_controller.value=y;
temp_controller.switch_status=switch_status;
return temp_controller;
}

//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH
#include <QueueArray.h>

int DUT=3;    //the number of the IRPS paired with IRPS 0
volatile unsigned long actual_pulse=0;
volatile int phase=0;
volatile float delta_time=99999;      // starting value equivalent to zero meters per second
volatile int EM_status=0;       // 0 = [EM off] IRPS not yet triggered,  1 = first IRPS gate,  2 = second IRPS gate,  3 = EM ON,
volatile int last_irps_number;
volatile int irps_number;

int xPos=0;
int loop_timer=0;     // facilitates occasional use of main loop by low frequency activities
int temp_calc;
float speed;

//*************************** PINOUT PARAMETERS ******************************************
//Pinouts - see 'arduino pin selection.jpg'
//        serial_pc     =0;   
//        serial_pc     =1;
const int ir_interrupt  =2;   
const int ir_add0       =3;
const int ir_add1       =4;
const int ir_add2       =5;
const int out_add0      =A2;
const int out_add1      =A1;
const int out_add2      =A0;
const int display_dc    =8;
const int display_cs    =10;
const int display_sda   =11;
const int out_flag      =A3;
const int display_sc    =13;
const int thr           =A4;
//const int da            =18;    //A4   I2C for BMP280
//const int cl            =19;    //A5   I2C for BMP280
//***************************************************************************************

// create a queue of characters.
QueueArray <char> printqueue;

struct ir_event {
    int irps_number;
    int em_number;
    unsigned long wait_duration;
    unsigned long pulse_duration;
    String print_string;
    int irps_status;
};

//Infra-Red Position Sensor (IRPS)
class IRPS
{
  // Class Member Variables
  // These are initialized at startup
  int irps_number_;   // the number of the IRPS
  int em_number_;   // the em related to the IRPS
  bool enabled_; // flag indicates whether beam break events are further processed
  int interbeam_dist;  // mm distance between beams
  int irps2em_dist;  // mm distance from closest beam to centre of em
  float tuning_factor_; // tuning factor which scales pulse duration
  bool send_to_print_stack_; // flag indicates whether beam break events are printed
  
  // These maintain the current state
  int irps_status;          // status of IRPS
  unsigned long s1_time;  // s1_time
  unsigned long s2_time;  // s2_time
 
  // Constructor - creates a IRPS 
  // and initializes the member variables and state
  public:
  IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, float tuning_factor, bool send_to_print_stack)
  {
    irps_number_=irps_number;
    em_number_=em_number;
    enabled_=enabled;
    interbeam_dist=interbeam_dist_mm;
    irps2em_dist=beam2em_dist_mm;
    tuning_factor_=tuning_factor;
    send_to_print_stack_=send_to_print_stack;
    irps_status=0;
    s1_time=0;
    s2_time=0; 
  }

public: void set_status(int new_status){
irps_status=new_status;
}

ir_event handleStatus()

{

  
//if triggering becomes out of synch, reset the status  
//    if (irps_number != last_irps_number) irps_status=0;

    
//    first infrared gate has been triggered
    if (irps_status==0){
      s1_time=micros();
      irps_status=1;
      EM_status=1;
      String report;          // hold data for printing     
//      replace report with "" to reduce serial data volumes
      ir_event temp = {irps_number_ , em_number_, 0, 0, "",1};
      return temp;
    }

//    second infrared gate has been triggered    
    if (irps_status==1){
      s2_time=micros();
      irps_status=2;
      EM_status=2;
      last_irps_number=irps_number;

    unsigned long wait_clicks_=0;
    unsigned long pulse_clicks_=0;
    delta_time = s2_time - s1_time;

    if (enabled_) {
    wait_clicks_= beam_em_clicks * delta_time;
//    pulse_clicks_= tf * delta_time * em_inter_clicks;
    pulse_clicks_= em_inter_clickstf * delta_time;
    }
    
      String report;          // hold data for printing      
       if (send_to_print_stack_==true) {
       float speed_=interbeam_const/(float) delta_time;
        report= String(irps_number_)+","+String(speed_,2) + "," + String(tf) + "," + String(offset);
//        report= String(irps_number_)+","+String(delta_time);        
       }
       
      irps_status=0;
      ir_event temp = {irps_number_ , em_number_, wait_clicks_, pulse_clicks_, report,2};      
      return temp;
    }
  }
};

class EM
{
  // Class Member Variables, initialized at startup
  int em_number_;   // the number of the EM
  bool enabled_; // flag indicates whether the EM can be turned on
  int safetime_;
  
  // These maintain the current state
  // Constructor - creates an EM
  // and initializes the member variables and state
  public:
  EM(int em_number,bool enabled,int safetime)
  {
    em_number_=em_number;
    enabled_=enabled;
    safetime_=safetime;
  }

void setup_timer(ir_event fire){
    actual_pulse = min(fire.pulse_duration,safetime_);

//    write output address prior to energising output line
      digitalWrite(out_add0,em_number_ & 1);
      digitalWrite(out_add1,em_number_ & 2);
      digitalWrite(out_add2,em_number_ & 4);   

          noInterrupts(); // disable all interrupts
          TCCR1A = 0;   //reset all bits
          TCCR1B = 0;   // reset all bits
          OCR1A = fire.wait_duration -offset; // input is already in clicks, in theory should subtract 1
          TCCR1B |= (1 << WGM12); // CTC mode
          TCCR1B |= (1 << CS11); // 8 prescaler
          TIFR1 |= (1 << OCF1A); // clear WORKS !!!
          
          TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
          TCNT1 = 0;    // set counter to 0
          digitalWrite(out_flag, EM_DISABLE);
          phase=0;
          interrupts(); // enable all interrupts
}

};    //end of EM class

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{

//for end of pulse, send disable output
if (phase==1) {
digitalWrite(out_flag,EM_DISABLE);
TIMSK1 &= ~(1 << OCIE1A); // disable CTC interrupt to give a 'one-shot' effect
EM_status=0;
}

//for beginning of pulse, send activate output
if (phase==0) {
digitalWrite(out_flag,EM_ACTIVE);
          noInterrupts(); // disable all interrupts
          TCCR1A = 0;   //reset all bits
          TCCR1B = 0;   // reset all bits
          OCR1A = actual_pulse; // input is already in clicks, in theory should subtract 1
          TCCR1B |= (1 << WGM12); // CTC mode
          TCCR1B |= (1 << CS11); // 8 prescaler
          TIFR1 |= (1 << OCF1A); // clear WORKS !!!
          
          TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
          TCNT1 = 0;    // set counter to 0
          phase =1;          
          interrupts(); // enable all interrupts  
          EM_status=3;
}
}

//Intialise Infra-Red Position Sensors and ElectroMagnets
//**************************** IRPS PHYSICAL CONFIG PARAMETERS ***************************
//IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, float tuning_factor, bool send_to_print_stack)
IRPS irps[8] = {
IRPS(0,0,false, interbeam_dist_mm,beam2em_dist_mm,0.0,false),
IRPS(1,1,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false), 
IRPS(2,0,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false), 
IRPS(3,2,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false),
IRPS(4,4,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false), 
IRPS(5,5,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false), 
IRPS(6,3,true, interbeam_dist_mm,beam2em_dist_mm,0.78,false), 
IRPS(7,6,false, interbeam_dist_mm,beam2em_dist_mm,0.78,false)
};
//***************************************************************************************

//***************************** EM CONFIG PARAMETERS ************************************
//max pulse duration in microseconds
//(int em_number,bool output_to_em,int safetime)
EM em[6] = {
EM(0,true, 25000),
EM(1,true, 25000),
EM(2,true, 25000),
EM(3,true, 25000),
EM(4,true, 25000),
EM(5,true, 25000)  

};
//***************************************************************************************

void setup() {

Serial.begin (250000);                      // start serial communication.

pinMode(ir_interrupt,INPUT); 
pinMode(ir_add0, INPUT); 
pinMode(ir_add1, INPUT); 
pinMode(ir_add2, INPUT); 
pinMode(out_add0, OUTPUT); 
pinMode(out_add1, OUTPUT); 
pinMode(out_add2, OUTPUT); 
pinMode(display_dc, OUTPUT); 
pinMode(display_cs, OUTPUT); 
pinMode(display_sda, OUTPUT); 
pinMode(out_flag, OUTPUT);
digitalWrite(out_flag,EM_DISABLE);        // initialise with turned off value 
pinMode(display_sc, INPUT); 

//SCREEN
tft.begin();
tft.setRotation(3);
display_main();
  
attachInterrupt(0, demuxINT, RISING);     // define interrupt based on rising edge of interrupt 0
                                          // in Arduino Uno, interrupt 0 is pin 2

//set pre calculated value for pulse time scaler
em_inter_clickstf=tf*2*em_width/interbeam_dist_mm;
}

//MAIN LOOP
void loop() {

if (EM_status !=0) return;      // Don't embark on complex logic when ball between IR sensors and EM

// Output to serial connection
if (!printqueue.isEmpty ()) {
 while (printqueue.peek()!= '\0') {
        Serial.print (printqueue.dequeue ());
 }
 Serial.println(printqueue.dequeue ());
 }

// read the value from the throttle and update the tf variable
if (loop_timer%10==0) {
ThrottleValue = analogRead(thr);
struct controller_struct temp_controller_loop=returnController(ThrottleValue);
//tf=ThrottleValue/1000;      // fast approximation to throttle value in range 0.45 to 0.9
//Red button toggles
if (temp_controller_loop.switch_status==a){
  tf_or_offset=!tf_or_offset;
}
if (tf_or_offset==0) {
//tft.print(temp_controller_loop.value);
tf= (float) temp_controller_loop.value/80.0;
em_inter_clickstf=tf*em_inter_clicks;
}
else if (tf_or_offset==1) {
offset= 600+temp_controller_loop.value * 20;
}

}

//CALCULATE AND DRAW SPEED
if (loop_timer%5==1) {
speed=interbeam_const/(float) delta_time;
temp_calc=240-speed*35;

//Manage screen scrolling
xPos=xPos+1;
if (xPos >319) {
  display_main(); 
  xPos=0;}
}

if (loop_timer%5==2) {

//int colorcode=4<<(irps_number-1);
int colorcode=65535;
tft.drawPixel( xPos,temp_calc, colorcode );
tft.drawPixel( xPos, temp_calc-1, colorcode );
//tft.drawPixel( xPos, temp_calc-2, ILI9341_WHITE );
}

if (loop_timer%5==3) {
tft.setCursor(0, 35);
tft.print(speed);
tft.print("   ");
tft.print(tf);
if (tf_or_offset==0) {
  tft.print("_   ");
  tft.print(offset);
  tft.print ("   ");
}
else {
  tft.print ("    ");
  tft.print(offset);
  tft.print("_");
}
}

//Manage loop counter to facilitate occasional use of main loop by low frequency tasks
if (++loop_timer==99) loop_timer=0;
}   //end of main loop

void demuxINT() {
//demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address

irps_number=(~PIND & B00111000) >> 3;
struct ir_event current_event=irps[irps_number].handleStatus();

if (current_event.irps_status==1) {
//    Return all other IRPS status to 0
  for (int i=0; i <= 3; i++){
  if (i != irps_number) irps[i].set_status(0);
   }
}

else if (current_event.pulse_duration >0 ) em[current_event.em_number].setup_timer(current_event);
  int string_length=current_event.print_string.length();
  if (string_length>2) {
   for (int i=0; i <= current_event.print_string.length(); i++){
      printqueue.enqueue(current_event.print_string.charAt(i));
   }
  }
}

void display_main(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  
  tft.setTextSize(1);
  tft.println("~ Super PPAv3 ~");
  tft.println("");
  tft.print("  Speed");
  tft.setTextSize(2);
}

