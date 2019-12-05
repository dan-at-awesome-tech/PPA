
//********************** PHYSICAL PARAMETERS **************************
const int interbeam_dist_mm=30;                                         // defined by the IRPS hardware geometry
const float interbeam_dist_um=interbeam_dist_mm*1000.0;                   // a convenient float version of interbeam_dist_mm to interbeam_dist_um (micrometers) to reduce calculation time
const int beam2em_dist_mm=32;                                           // defined by the length of the connecting rod between IRPS and EM
const int em_width=15;                                                  // defined by width of EM

//const float beam_em_clicks=2*interbeam_dist_mm/beam2em_dist_mm;         // A geometric constant used to reduce calculation time for start of pulse calculation
//const float em_inter_clicks=2*em_width/interbeam_dist_mm;               // A geometric constant used to reduce calculation time for duration of pulse calculation
//const float beam_em_ratio=beam2em_dist_mm/interbeam_dist_mm;             // A geometric constant used to reduce calculation time for start of pulse calculation
const float em_inter_ratio=7.5/interbeam_dist_mm;               // A geometric constant used to reduce calculation time for duration of pulse calculation
const int clicks_per_us=2;                                              // with an Arduino Uno, use 2 processor clicks per microsecond
//*********************************************************************

volatile float beam_em_ratio;                 // a geometric ratio of the distance of beam to EM divided by interbeam distance
volatile float em_inter_ratiotf;            // a variable to hold the pulse duration constant multiplied by the tuning factor
float tf=0.75;                      // initial value for the tuning factor, which can be controlled by throttle
float offset=1.0;                      // initial value for the offset, defined as the number of mm back from the EM that the pulse starts.  This parameter can be adjusted with the throttle
bool tf_or_offset=0;                // initial value for user interface focus (cursor)
bool mode=0;                         // mode=0 normal   mode=1 throttle display
int tempThrottle;       // temporary value to store throttle


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
   int button_status;
};
float ThrottleValue = 0;  // variable to store the value coming from the sensor

//throttle variables
int throttle=0;
int last_throttle=0;
int irps_adjust=3;
int cv=5;       //correction value
//Set these values manually.  They are derived in parts in range 0-1023
const int low_no_switch=458-cv;
const int high_no_switch=852+cv;
const int low_a=65-cv;
const int high_a=198+cv;
const int low_b=872-cv;
const int high_b=952+cv;
const int low_both=336-cv;
const int high_both=415+cv;
const int a=1;
const int b=2;
const int both=3;
const int none=0;
const int error=5;
const int min_range=1;
const int max_range=80;
//*********************************************************************

//Throttle logic
struct controller_struct returnController(int sensorValue) {
int y;
int button_status;
struct controller_struct temp_controller;
//Serial.println (sensorValue);
if (sensorValue >= low_no_switch && sensorValue <= high_no_switch) {
// Serial.println ("nothing is pressed");
 y = map(sensorValue, low_no_switch, high_no_switch, min_range, max_range);
 button_status=none;
}
else if (sensorValue >= low_a && sensorValue <= high_a) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_a, high_a, min_range,max_range);
 button_status=a; 
}
 else if (sensorValue >= low_b && sensorValue <= high_b) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_b,high_b, min_range, max_range);
 button_status=b;  
}
else if (sensorValue >= low_both && sensorValue <= high_both) {
// Serial.println ("S2 is pressed");
 y = map(sensorValue, low_both, high_both, min_range, max_range);
 button_status=both; 
}
else {
// Serial.println ("Recalibrate throttle");
 y = 0;
 button_status=error; 
}
temp_controller.value=y;
temp_controller.button_status=button_status;
return temp_controller;
}

//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH
#include <QueueArray.h>

//DUT is Device Under Test, referring to the IRPS positioned after the EM, which is used to calibrate the EM
int DUT=3;    //the number of the IRPS paired with IRPS 0
volatile unsigned long actual_pulse=0;
volatile int phase=0;                 // variable to hold whether the hardware timers are waiting for EM turn on (0) or EM turn off (1)
volatile float delta_time=9999999;      // starting time difference between first and second beam interruptions, equivalent to zero meters per second
volatile float delta_clicks=9999999;    // starting click difference between first and second beam interruptions, equivalent to zero meters per second
volatile int EM_status=0;       // 0 = [EM off] IRPS not yet triggered,  1 = first IRPS gate,  2 = second IRPS gate,  3 = EM ON,

volatile int irps_number;             // holds the current IRPS being triggered

//Screen and reporting variables
int xPos=0;           // the x-position of the rolling velocity graph, which is incremented as the graph covers the screen 
int loop_timer=0;     // facilitates occasional use of main loop by low frequency activities
int temp_calc;        // variable to hold velocity in metres per second for graphing purposes
float speed;          // holds speed for graphing and reporting purposes

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

// create a queue of characters for reporting over USB
QueueArray <char> printqueue;

// The main structure to hold an upcoming EM activation
struct ir_event {
    int irps_number;
    int em_number;
    unsigned long wait_duration;    // the number of clicks between the second beam interruption and the start of EM activation
    unsigned long pulse_duration;   // the number of clicks the EM is activated
    String print_string;            // the reporting string to be sent over USB
    int irps_status;                // indicates prior first beam break (0), after first beam break (1), and after second beam break (2)
};

//Infra-Red Position Sensor (IRPS)
class IRPS
{
  // Class Member Variables
  // These are initialized at startup
  int irps_number_;   // the number of the IRPS
  int em_number_;   // the em related to the IRPS
  bool enabled_; // flag indicates whether beam break events are further processed into an EM activation
  int interbeam_dist;  // mm distance between beams
  int irps2em_dist;  // mm distance from closest beam to centre of em
  float tuning_factor_; // tuning factor which scales pulse duration
  bool send_to_print_stack_; // flag indicates whether beam break events are printed to stack for subsequent sending over USB
  
  // Store the current state
  int irps_status;          // status of IRPS
  unsigned long s1_time;  // s1_time is when first beam is interrupted
  unsigned long s2_time;  // s2_time is when second beam is interrupted
 
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

// The main function that is called when a beam interrupt occurs
ir_event handleStatus()
  {

//    first infrared beam has been broken
    if (irps_status==0){
      s1_time=micros();       //record beam break time
      irps_status=1;          //update status to show first beam break
      EM_status=1;            //update the volatile variable so main loop doesn't embark on complex work during critical period
      String report;          // hold data for printing     
//    replace report with "" to reduce serial data volumes
      ir_event temp = {irps_number_ , em_number_, 0, 0, "",1};   // return IRPS and EM, no need to return wait, pulse or report when first beam has been broken
      return temp;
    }

//    second infrared beam has been broken
    if (irps_status==1){
      s2_time=micros();     //record beam break time
      irps_status=2;        //update status to show first beam break
      EM_status=2;          //update the volatile variable so main loop doesn't embark on complex work during critical period
 
    unsigned long wait_clicks_=0;               //reset calculation variable
    unsigned long pulse_clicks_=0;              //reset calculation variable
    delta_time = s2_time - s1_time;             //time between first and second beam breaks
    delta_clicks = delta_time * clicks_per_us;   //convert microseconds to clicks

//  if the electromagnet is enabled for pulsing, calculate required timing
    if (enabled_) {
    wait_clicks_= beam_em_ratio * delta_clicks;                 //wait_clicks is number of clicks before starting EM pulse
    pulse_clicks_= em_inter_ratiotf * delta_clicks;            //pulse_clicks is number of clicks of EM pulse duration
    }
    
      String report;          // hold data for printing      
       if (send_to_print_stack_==true) {
       float speed_=interbeam_dist_um/(float) delta_time;       //calculate speed as float for reporting
        report= String(irps_number_)+","+String(wait_clicks_) + "," + String(pulse_clicks_);   

//        report= String(irps_number_)+","+String(speed_,2) + "," + String(tf) + "," + String(offset);   
//        report= String(irps_number_)+","+String(delta_time);        
       }
       
      irps_status=0;                                          //now reset status for next rotation
      ir_event temp = {irps_number_ , em_number_, wait_clicks_, pulse_clicks_, report,2};      
      return temp;
    }
  }
};

class EM
{
  // Class Member Variables, initialized at startup
  int em_number_;   // the number of the EM
  bool enabled_;    // flag indicates whether the EM can be turned on
  int safetime_;    // the maximum allowed pulse duration
  
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
    actual_pulse = min(fire.pulse_duration,safetime_);      // restrict pulse duration to maximum safe value


//    write EM output address prior to energising output line
      digitalWrite(out_add0,em_number_ & 1);
      digitalWrite(out_add1,em_number_ & 2);
      digitalWrite(out_add2,em_number_ & 4);   

//    Set up interrupt to wait for right time to start the pulse

          noInterrupts();                         // disable all interrupts while interrupt parameters are changing
          TCCR1A = 0;                             //reset all bits
          TCCR1B = 0;                             // reset all bits
          OCR1A = fire.wait_duration;             // input is already in clicks, in theory should subtract 1
          TCCR1B |= (1 << WGM12);                 // CTC mode
          TCCR1B |= (1 << CS11);                  // 8 prescaler.  On a 16MHz Arduino, a prescaler of 8 gives a 0.5usecond click
          TIFR1 |= (1 << OCF1A);                  // clear
          
          TIMSK1 |= (1 << OCIE1A);                // enable timer compare interrupt
          TCNT1 = 0;                              // set counter to 0
          digitalWrite(out_flag, EM_DISABLE);     // ensure EM is OFF during waiting period
          phase=0;                                // recognise that EM is in waiting period
          interrupts();                           // enable all interrupts
}

};    //end of EM class



// **********   Hardware timing interrupt event  ***************

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{

//for end of pulse, send disable output quickly
if (phase==1) {
digitalWrite(out_flag,EM_DISABLE);              // disable EM
TIMSK1 &= ~(1 << OCIE1A);                       // disable future interrupts ie. CTC interrupt to give a 'one-shot' effect
EM_status=0;                                    // clear status to show EM is off
}

//for start of pulse, send activate output
if (phase==0) {
digitalWrite(out_flag,EM_ACTIVE);
          noInterrupts();                       // disable all interrupts
          TCCR1A = 0;                           //reset all bits
          TCCR1B = 0;                           // reset all bits
          OCR1A = actual_pulse;                 // input is already in clicks, in theory should subtract 1
          TCCR1B |= (1 << WGM12);               // CTC mode
          TCCR1B |= (1 << CS11);                // 8 prescaler
          TIFR1 |= (1 << OCF1A);                // clear
          
          TIMSK1 |= (1 << OCIE1A);              // enable timer compare interrupt
          TCNT1 = 0;                            // set counter to 0
          phase =1;          
          interrupts();                         // enable all interrupts  
          EM_status=3;                          // Set EM is now energised
}
}



//Intialise Infra-Red Position Sensors and ElectroMagnets
//**************************** IRPS PHYSICAL CONFIG PARAMETERS ***************************
//IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, float tuning_factor, bool send_to_print_stack)
IRPS irps[4] = {
IRPS(0,0,false, interbeam_dist_mm,beam2em_dist_mm,0.0,false),
IRPS(1,0,true, interbeam_dist_mm,beam2em_dist_mm,tf,true), 
IRPS(2,2,true, interbeam_dist_mm,beam2em_dist_mm,tf,false), 
IRPS(3,1,true, interbeam_dist_mm,beam2em_dist_mm,tf,false) 
};
//***************************************************************************************



//***************************** EM CONFIG PARAMETERS ************************************
//max pulse duration in microseconds
//(int em_number,bool output_to_em,int safetime)
EM em[3] = {
EM(0,true, 25000),
EM(1,true, 25000),
EM(2,true, 25000)  
};
//***************************************************************************************

void setup() {
  
//initial calculation of ratio for start of pulse duration
beam_em_ratio=(float)(beam2em_dist_mm-offset)/interbeam_dist_mm;             // A geometric ratio used to reduce calculation time for start of pulse calculation

//initial calculation of pulse time scaler
em_inter_ratiotf=tf*em_inter_ratio;                                   // A geometric ratio of EM half point width to interbeam distance multiplied by tuning factor

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

}

//MAIN LOOP
void loop() {

//if (EM_status !=0) return;      // Don't embark on complex logic when ball between IR sensors and EM

// Output to serial connection
if (!printqueue.isEmpty ()) {
 while (printqueue.peek()!= '\0') {
        Serial.print (printqueue.dequeue ());
 }
 Serial.println(printqueue.dequeue ());
 }

// read the value from the throttle and update the tf variable
if (loop_timer%9==0) {
ThrottleValue = analogRead(thr);
struct controller_struct temp_controller_loop=returnController(ThrottleValue);
tempThrottle=temp_controller_loop.value;

//Red button toggles between tf and offset parameters
if (temp_controller_loop.button_status==a){
  tf_or_offset=!tf_or_offset;
}

// update tuning factor
if (tf_or_offset==0) {
//tft.print(temp_controller_loop.value);
if (temp_controller_loop.button_status==none){
tf= (float) temp_controller_loop.value/50.0;
em_inter_ratiotf=(float)tf*((em_width/2.0)+offset)/interbeam_dist_mm;                                   // A geometric ratio of EM half point width to interbeam distance multiplied by tuning factor
}
}

// update offset
else if (tf_or_offset==1) {
//set offset in range 0-4mm from throttle value 1-80   
if (temp_controller_loop.button_status==none){
offset= (float)temp_controller_loop.value /12.0;
beam_em_ratio=((float)beam2em_dist_mm-offset)/interbeam_dist_mm;             // A geometric ratio used to reduce calculation time for start of pulse calculation
}
}

//Both button calls custom function
if (temp_controller_loop.button_status==both){
  // insert custom function triggered by the Green and Red button

  // toggle throttle display mode
  mode=!mode;
}

}

if (mode==1){
    tft.fillScreen(ILI9341_BLACK);  
    tft.setCursor(10,110);
    tft.print("Throttle: ");
    tft.print(ThrottleValue);
    tft.print(" ");
    tft.print(tempThrottle);
    tft.print(" ");
  }

// Process normal screen mode
if (mode==0) {

//CALCULATE AND DRAW SPEED
if (loop_timer%5==1) {
speed=interbeam_dist_um/(float) delta_time;          // micrometers divided by microseconds is equivalent to metres per second
temp_calc=240-speed*35;                           // scale speed according to graph area on scale for plotting purposes
  
//Manage screen scrolling
xPos=xPos+1;
if (xPos >319) {
  display_main(); 
  xPos=0;}
}

//occasional screen update
if (loop_timer%5==2) {

//int colorcode=4<<(irps_number-1);
int colorcode=65535;
tft.drawPixel( xPos,temp_calc, colorcode );
tft.drawPixel( xPos, temp_calc-1, colorcode );
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
}
//}

//Manage loop counter to facilitate occasional use of main loop by low frequency tasks
if (++loop_timer==99) loop_timer=0;
}   //end of main loop

void demuxINT() {

//demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address
irps_number=(~PIND & B00111000) >> 3;

//call function to process IR interrupt
struct ir_event current_event=irps[irps_number].handleStatus();

//if it was first beam, then reset all other EM status
if (current_event.irps_status==1) {
//    Return all other IRPS status to 0
  for (int i=0; i <= 3; i++){
  if (i != irps_number) irps[i].set_status(0);
   }
}

// if a pulse is required, then send parameters to EM function
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
  tft.println("~ PPA! ~ particle_simple_2_param_v5");
  tft.println("");
  tft.print("  Speed");
  tft.setTextSize(2);
}
