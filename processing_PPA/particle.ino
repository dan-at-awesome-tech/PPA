
//4May17 - investigate why can't exit debug mode

//Hey, all values for board 12b and new electromagnets

//Switch to do
//read sensor 0.  Calculate speed and arrival finish pulse times.
//Set up alternative output pulse interrupts to Valve output

//SCREEN
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 8
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
const int normal=0;
const int debug=9;
int graphics_mode=0;

//PRESSURE SENSOR
int PressureValue=0;

//THROTTLE
struct controller_struct
{
   int value;
   int switch_status;
};
int ThrottleValue = 0;  // variable to store the value coming from the sensor
//Set these values manually
const int low_no_switch=434;
const int high_no_switch=856;
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

//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH
#include <QueueArray.h>
const int em_width=15;
int DUT=3;    //the number of the IRPS paired with IRPS 0
volatile unsigned long actual_pulse=0;
volatile int phase=0;
volatile int fire0=0;
volatile int fire1=0;
volatile int fire2=0;
volatile float latest_speed=0;
volatile int EM_status=0;       // 0 = [EM off] IRPS not yet triggered,  1 = first IRPS gate,  2 = second IRPS gate,  3 = EM ON,
int xPos=0;
int loop_timer=0;

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
const int pressure_sensor =A5;    
const int pump          =6;  
const int valve         =7;  
//const int da            =18;    //A4   I2C for BMP280
//const int cl            =19;    //A5   I2C for BMP280

//throttle variables
int throttle=0;
int last_throttle=0;
int irps_adjust=3;
int old_value=40;

//switch variables
int switch_status=0;
int last_switch_status=0;
unsigned long temp_millis=0;
int vacuum_status=0;
int pump_on=0;

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

float read_tf(){
  return tuning_factor_;
}

void update_tf(float tf){
tuning_factor_=tf;
};


ir_event handleStatus()
  {

//    first infrared gate has been triggered
    if (irps_status==0){
      s1_time=micros();
      irps_status=1;
      EM_status=1;
      String report;          // hold data for printing     
      // create String:  irps_number,status,time,speed,pulse        
//  24/4/2017    if (send_to_print_stack_==true) report= String(irps_number_)+" 1 "+String(s1_time);
//  24/4/2017    if (send_to_print_stack_==true) report= String(irps_number_)+" 1 ";
// do not stack any data for first IRPS interrupt            
//      replace report with "" to reduce serial data volumes
      ir_event temp = {irps_number_ , em_number_, 0, 0, "",1};
      return temp;
    }

//    first infrared gate has been triggered    
    if (irps_status==1){
      s2_time=micros();
      irps_status=2;
      EM_status=2;
//create random tuning factor for data gathering purposes
//if (irps_number_==DUT) tuning_factor_=(float)random(50,115)/100.0;
      
      float speed_=calcSpeed(s1_time,s2_time);
      latest_speed=speed_;
      unsigned long wait_length_=enabled_ * calcWaitDuration(speed_);
      unsigned long pulse_length_=enabled_ * calcPulseDuration(speed_);
      String report;          // hold data for printing      
      // create String:  irps_number,status,time,speed,pulse       
//      if (send_to_print_stack_==true) report= String(irps_number_)+" 2 "+String(s2_time)+" "+String(speed_,3) + " "+String(wait_length_)+ " "+String(pulse_length_);
// 24/4/2017      if (send_to_print_stack_==true) report= String(irps_number_)+",2,"+String(speed_,3) + ","+String(wait_length_)+ ","+String(pulse_length_) + "," + String(tuning_factor_) + "," + String(s2_time);
       if (send_to_print_stack_==true) report= String(irps_number_)+","+String(speed_,2) + "," + String(tuning_factor_);

  if (irps_number_==3) fire1=1;
  if (irps_number_==1) fire0=1;
  if (irps_number_==2) fire2=1;  

      
      irps_status=0;
      ir_event temp = {irps_number_ , em_number_, wait_length_, pulse_length_, report,2};      
      return temp;
    }
  }

 float calcSpeed(unsigned long s1_time_, unsigned long s2_time_){
//  factor of 0.001 converts mm/microseconds to m/s
  unsigned long delta_ul = s2_time_ - s1_time_;
  float speed_=1000.0 * (float) interbeam_dist/(float) delta_ul;
  return speed_;
  };
  
 unsigned long calcWaitDuration(float speed_){
//  output must be in microseconds
//  factor of 1000 converts mm/(m/s) to microseconds
//  unsigned long time_= 1000 * irps2em_dist/speed_;
//experiment to see if additional on time as ball approaches face increases velocity
  unsigned long time_= 1000 * irps2em_dist/speed_-100;  
  return time_;
  };
 
 unsigned long calcPulseDuration(float speed_){
//  output must be in microseconds
//  factor of 1000 converts mm/(m/s) to microseconds
  word part_a = 25 * tuning_factor_;
  word part_b = 40 * em_width/speed_;
//  unsigned long time_= part_a * part_b;
//experiment to see if additional on time as ball approaches face increases velocity
  unsigned long time_= part_a * part_b + 100;
  return time_;
  };

};

class EM
{
  // Class Member Variables
  // These are initialized at startup
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

      if (enabled_) {

//    write output address prior to energising output line
      digitalWrite(out_add0,em_number_ & 1);
      digitalWrite(out_add1,em_number_ & 2);
      digitalWrite(out_add2,em_number_ & 4);   

          noInterrupts(); // disable all interrupts
          TCCR1A = 0;   //reset all bits
          TCCR1B = 0;   // reset all bits
//          OCR1A = (int) 1999; // compare match register is 1ms duration.
          OCR1A = fire.wait_duration*2 -1; // convert useconds to timer clicks by * 2
          TCCR1B |= (1 << WGM12); // CTC mode
          TCCR1B |= (1 << CS11); // 8 prescaler
          TIFR1 |= (1 << OCF1A); // clear WORKS !!!
          
          TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
          TCNT1 = 0;    // set counter to 0
          digitalWrite(out_flag, EM_DISABLE);
          phase=0;
          interrupts(); // enable all interrupts
      }

}

};

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
//          OCR1A = (int) 1999; // compare match register is 1ms duration.
          OCR1A = actual_pulse*2 -1; // convert useconds to timer clicks by * 2
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

// ******************************************************
//Intialise Infra-Red Position Sensors and ElectroMagnets

//IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, float tuning_factor, bool send_to_print_stack)
IRPS irps[4] = {
IRPS(0,0,false, 30,30,0.0,true),
IRPS(1,0,true, 30,30,0.78,false),      // optimised 20170122
IRPS(2,2,true, 30,30,0.78,true),       // optimised 20170122
IRPS(3,1,true, 30,30,0.78,false)       // optimised 20170122
};

//max pulse duration in microseconds
//(int em_number,bool output_to_em,int safetime)
EM em[3] = {
EM(0,true, 25000),
EM(1,true, 25000),
EM(2,true, 25000)  
};

void setup() {
  
Serial.begin (115200);                      // start serial communication.

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
pinMode(thr, INPUT);    //A0 
//pinMode(swt, INPUT);    //A1
pinMode(pump, OUTPUT);    //A2
pinMode(valve, OUTPUT);    //A3
//pinMode(da, INPUT);    //A4   I2C for BMP280
//pinMode(cl, INPUT);    //A5   I2C for BMP280
pinMode(pressure_sensor, INPUT);    //A5

//SCREEN
tft.begin();
  tft.setRotation(3);
  display_main();
  
attachInterrupt(0, demuxINT, RISING);     // define interrupt

//Serial.println("*** INITIALISE ***");
}

//MAIN LOOP
void loop() {

// Output to serial connection
if (!printqueue.isEmpty ()) {
 while (printqueue.peek()!= '\0') {
        Serial.print (printqueue.dequeue ());
 }
Serial.println(printqueue.dequeue ());
 }

if (EM_status !=0) return;

// read the value from the throttle:
ThrottleValue = analogRead(thr);
struct controller_struct temp_controller_loop=returnController(ThrottleValue);

if (loop_timer % 10==0) {
  
//TUNING FACTOR FUNCTION
if (temp_controller_loop.switch_status==a) {
//update current EM tuning factor

//remove reverse printing of current value
tft.setCursor(230, 150+(DUT*25));
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print(DUT);
  tft.print(" ");  
  tft.print(irps[DUT].read_tf());
  tft.print(" ");
//change value and display
DUT=DUT+1;
if (DUT==4) DUT=0;
tft.setCursor(230, 150+(DUT*25));
  tft.setTextColor(ILI9341_GREEN,ILI9341_WHITE);
  tft.print(DUT);
  tft.print(" ");  
  tft.print(irps[DUT].read_tf());
  tft.print(" ");
}

//if buttons not activated and throttle value changes
if (temp_controller_loop.switch_status==none && graphics_mode==normal) {
if (temp_controller_loop.value > old_value) irps[DUT].update_tf(min(1.05,irps[DUT].read_tf()+0.01));
if (temp_controller_loop.value < old_value) irps[DUT].update_tf(max(0.60,irps[DUT].read_tf()-0.01));
old_value=temp_controller_loop.value;
tft.setCursor(230, 150+(DUT*25));
  tft.setTextColor(ILI9341_GREEN,ILI9341_WHITE);
  tft.print(DUT);
  tft.print(" ");  
  tft.print(irps[DUT].read_tf());
  tft.print(" ");
}
}

if (loop_timer % 10==2) {
// button b function
if (temp_controller_loop.switch_status==b) {
    digitalWrite(valve,HIGH); }
else digitalWrite(valve,LOW);

//turn on pump
//    digitalWrite(pump,HIGH);
//}
//else digitalWrite(pump,LOW);

//disable an EM
//em[0] = {EM(0,false,25000)};
//}
//else em[0] = {EM(0,true,25000)};
}

//DEBUG MODE FUNCTION
if (temp_controller_loop.switch_status==both) {
if (graphics_mode==debug) {
  graphics_mode=normal;
  tft.fillScreen(ILI9341_BLACK);  
}
else if (graphics_mode==normal) {
  graphics_mode=debug;
  tft.fillScreen(ILI9341_BLACK);  
}
}

if (graphics_mode==debug) {
//draw green rectangles
for (int i=0; i <= 3; i++){
 tft.fillRect(i*55,0,50,50,ILI9341_GREEN);
}

int IRPS_interrupt=digitalRead(ir_interrupt);
if (IRPS_interrupt==HIGH) {
int irps_number=(4*!digitalRead(ir_add2)) + (2*!digitalRead(ir_add1)) + !digitalRead(ir_add0);  
//draw red rectangles
tft.fillRect(irps_number*55,0,50,50,ILI9341_RED);
}
}

if (loop_timer%20==4) {
//READ PRESSURE
PressureValue = analogRead(pressure_sensor)/8.05;    // convert to KPa approximately, where 101kPa gives reading of about 815
}

if (loop_timer%10==6 && graphics_mode==normal) {
//PERIODIC SCAN FOR SCREEN UPDATE
tft.setCursor(0, 35);

//DRAW THROTTLE VALUE
tft.drawPixel( xPos, 240-temp_controller_loop.value*3, ILI9341_RED );
tft.drawPixel( xPos, 240-temp_controller_loop.value*3-1, ILI9341_RED );
tft.drawPixel( xPos, 240-temp_controller_loop.value*3-2, ILI9341_RED );
  tft.setTextColor(ILI9341_RED,ILI9341_BLACK);
  tft.print(temp_controller_loop.value);
  tft.print("   ");

//DRAW SPEED
tft.drawPixel( xPos, 240-latest_speed*40, ILI9341_WHITE );
tft.drawPixel( xPos, 240-latest_speed*40-1, ILI9341_WHITE );
tft.drawPixel( xPos, 240-latest_speed*40-2, ILI9341_WHITE );
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tft.print(latest_speed);
  tft.print("   ");

//DRAW PRESSURE VALUE
tft.drawPixel( xPos, 240-PressureValue*1.5, ILI9341_GREEN );
tft.drawPixel( xPos, 240-PressureValue*1.5-1, ILI9341_GREEN );
tft.drawPixel( xPos, 240-PressureValue*1.5-2, ILI9341_GREEN );
  tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print(PressureValue);
  tft.println("      ");      // cover over any previous displays that extended out this far

//tft.println(latest_speed);
//latest_speed=0;
xPos=xPos+1;

if (xPos >319) {
  display_main(); 
  xPos=0;
}
}

loop_timer=loop_timer+1;
if (loop_timer==99) loop_timer=0;

}   //end of loop

void demuxINT() {
//demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address

int irps_number=(4*!digitalRead(ir_add2)) + (2*!digitalRead(ir_add1)) + !digitalRead(ir_add0);
struct ir_event current_event=irps[irps_number].handleStatus();

if (current_event.irps_status==1) {
//    Return all other IRPS status to 0
  for (int i=0; i <= 3; i++){
  if (i != irps_number) irps[i].set_status(0);
   }
}

//try else 20170122
else 

if (current_event.pulse_duration >0 ) em[current_event.em_number].setup_timer(current_event);
  int string_length=current_event.print_string.length();
  if (string_length>2) {
   for (int i=0; i <= current_event.print_string.length(); i++){
      printqueue.enqueue(current_event.print_string.charAt(i));
   }
  }
}

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
 Serial.println ("error case - invalid input");
 y = 0;
 switch_status=error; 
}
temp_controller.value=y;
temp_controller.switch_status=switch_status;
return temp_controller;
}

void display_main(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("      ~ Personal Particle Accelerator! ~");
  tft.println("");
  tft.setTextColor(ILI9341_RED);
  tft.print("Throttle");
  tft.setTextColor(ILI9341_WHITE);
  tft.print("   Speed");
  tft.setTextColor(ILI9341_GREEN);
  tft.println("      Pressure");
  tft.setTextSize(2);

tft.setCursor(230, 150+(1*25));
if (DUT==1) tft.setTextColor(ILI9341_GREEN,ILI9341_WHITE);
else tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print("1 ");
  tft.print(irps[DUT].read_tf());
  tft.print(" ");

tft.setCursor(230, 150+(2*25));
if (DUT==2) tft.setTextColor(ILI9341_GREEN,ILI9341_WHITE);
else tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print("2 ");
  tft.print(irps[DUT].read_tf());
  tft.print(" ");  

tft.setCursor(230, 150+(3*25));
if (DUT==3) tft.setTextColor(ILI9341_GREEN,ILI9341_WHITE);
else tft.setTextColor(ILI9341_GREEN,ILI9341_BLACK);
  tft.print("3 ");  
  tft.print(irps[DUT].read_tf());
  tft.print(" ");  
}

