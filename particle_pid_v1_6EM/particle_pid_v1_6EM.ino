
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling

//********************** PHYSICAL PARAMETERS **************************
const float beam_width_mm = 15.0f;                                       // defined by the IRPS hardware geometry
const float beam_width_um = beam_width_mm * 1000.0f;                     // a convenient float version of beam_width_mm to beam_width_mm (micrometers) to reduce calculation time
const float beam_center_mm = beam_width_mm / 2.0f;                       // halfway through an IRPS sensor
const float beam_center_um = beam_center_mm * 1000.0f;                   // a convenient float version of beam_center_mm to beam_center_mm (micrometers) to reduce calculation time
const float interbeam_dist_mm = 30.0f;                                   // defined by the IRPS hardware geometry
const float interbeam_dist_um = interbeam_dist_mm * 1000.0f;             // a convenient float version of interbeam_dist_mm to interbeam_dist_um (micrometers) to reduce calculation time
const float beam2em_dist_mm = 24.0f;                                     // defined by the length of the connecting rod between IRPS and EM
const float beam2em_dist_um = beam2em_dist_mm * 1000.0f;                 // a convenient float version of beam2em_dist_mm to beam2em_dist_um (micrometers) to reduce calculation time
const float em_width_mm = 15.0f;                                         // defined by width of EM
const float em_width_um = em_width_mm * 1000.0f;                         // a convenient float version of em_width_mm to em_width_um (micrometers) to reduce calculation time
const float em_center_mm = em_width_mm / 2.0f;                           // halfway through the EM
const float em_center_um = em_center_mm * 1000.0f;                       // a convenient float version of em_center_mm to em_center_um (micrometers) to reduce calculation time

const float max_pulse_dist_mm = 
  beam_center_mm + beam2em_dist_mm + em_center_mm;                       // all pulses will be relative to this max distance
const float max_pulse_dist_um = 
  beam_center_um + beam2em_dist_um + em_center_um;                       // all pulses will be relative to this max distance

const int em_max_safe_clicks = 60000;

const int clicks_per_us = 2;                                             // with an Arduino Uno, use 2 processor clicks per microsecond
//*********************************************************************


//********************** SCREEN ***************************************
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 8
#define TFT_CS 10
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
//*********************************************************************

//********************** PID ******************************************
#include "PID_v2.h"
// Specify the links and initial tuning parameters
//double Kp = 4, Ki = 0.2, Kd = 0.1;
double Kp = (max_pulse_dist_um * 0.80f), Ki = 0.20f, Kd = 0.10f;
PID_v2 ppaPID(Kp, Ki, Kd, PID::Direct);
//*********************************************************************

//********************** THROTTLE PARAMETERS **************************
//throttle variables
int cv = 5;     //correction value
//Set these values manually.  They are derived in parts in range 0-1023
const int low_no_switch = 458 - cv;
const int high_no_switch = 852 + cv;
//*********************************************************************

double updateThrottle(const int pin) {
  float throttle = analogRead(pin);
  if (throttle < low_no_switch) {
    return 2.0f;
  } else if (throttle > high_no_switch) {
    return 4.0f;
  } else {
    // map from throttle position to 2.00m/s - 4.00m/s
    return map(throttle, low_no_switch, high_no_switch, 200, 400) / 100.0;
  }
}

//IR sensors and EM's
//NOTE: in board 11 onwards, OUT_FLAG is pulsed LOW to activate electromagnets.
//set via polarity definitions below
#define EM_ACTIVE LOW
#define EM_DISABLE HIGH
#include <QueueArray.h>

volatile int irps_number = 0;             // holds the current IRPS being triggered

//Screen and reporting variables
int xPos = 0;         // the x-position of the rolling velocity graph, which is incremented as the graph covers the screen
int last_irps_number = 0;

//*************************** PINOUT PARAMETERS ******************************************
//Pinouts - see 'arduino pin selection.jpg'
//        serial_pc     =0;
//        serial_pc     =1;
const int ir_interrupt  = 2;
const int ir_add0       = 3;
const int ir_add1       = 4;
const int ir_add2       = 5;
const int out_flag      = A3;
const int out_add0      = A2;
const int out_add1      = A1;
const int out_add2      = A0;
const int display_dc    = TFT_DC;
const int display_cs    = TFT_CS;
const int display_sda   = 11;
const int display_sc    = 13;
const int thr           = A4;
//const int scope0_toggle = A5;
//const int da            =18;    //A4   I2C for BMP280
//const int cl            =19;    //A5   I2C for BMP280
//***************************************************************************************

// create a queue of characters for reporting over USB
QueueArray <char> printqueue;

enum class IRPSStatus {
  Ready = 0,
  Entered = 1,
  Exited = 2 
};

// The main structure to hold an upcoming EM activation
struct ir_event {
  int irps_number;
  int em_number;
  unsigned long wait_duration;    // the number of clicks between the second beam interruption and the start of EM activation
  unsigned long pulse_duration;   // the number of clicks the EM is activated
  String print_string;            // the reporting string to be sent over USB
  IRPSStatus irps_status;         // indicates prior first beam break (0), after first beam break (1), and after second beam break (2)
};

//Infra-Red Position Sensor (IRPS)
class IRPS
{
    // Class Member Variables
    // These are initialized at startup
    int irps_number_;          // the number of the IRPS
    int em_number_;            // the em related to the IRPS
    bool enabled_;             // flag indicates whether beam break events are further processed into an EM activation
    int interbeam_dist_mm_;    // mm distance between beams
    int max_dist_mm_;          // mm distance from closest beam to centre of beam to em distance
    int pulse_dist_mm_;        // mm distance from closest beam to centre of em
    bool send_to_print_stack_; // flag indicates whether beam break events are printed to stack for subsequent sending over USB

    // Store the current state
    IRPSStatus irps_status_;   // status of IRPS
    unsigned long s1_us;       // s1_us is when first beam is interrupted
    unsigned long s2_us;       // s2_us is when second beam is interrupted

    // Constructor - creates a IRPS
    // and initializes the member variables and state
  public:
    IRPS(int irps_number, int em_number, bool enabled, int interbeam_dist_mm, int max_dist_mm, bool send_to_print_stack)
    {
      irps_number_ = irps_number;
      em_number_ = em_number;
      enabled_ = enabled;
      interbeam_dist_mm_ = interbeam_dist_mm;
      max_dist_mm_ = max_dist_mm;
      send_to_print_stack_ = send_to_print_stack;
      pulse_dist_mm_ = 0;
      reset();
    }

  public:

    void reset() {
      irps_status_ = IRPSStatus::Ready;
      s1_us = 0;
      s2_us = 0;
    }

    void set_pulse_dist_mm(int pulse_mm) {
      pulse_dist_mm_ = pulse_mm;
    }
    
    IRPSStatus get_status() {
      return irps_status_;
    }

    int get_irps_number() {
      return irps_number_;
    }

    int get_em_number() {
      return em_number_;
    }
    
    // microseconds between first and second beam breaks
    float get_delta_time_us() {
      if (irps_status_ == IRPSStatus::Exited && s1_us != 0 && s2_us != 0) {
        return s2_us - s1_us;
      }
      return -1.0f; // invalid
    }

    // The main function that is called when a beam interrupt occurs
    ir_event handleStatus(int irps_number)
    {

      //    first infrared beam has been broken in a valid state
      if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Ready && s1_us == 0 && s2_us == 0) {
        s1_us = micros();                   // record beam break time
        irps_status_ = IRPSStatus::Entered; // update status to show first beam break
        String report;                      // hold data for printing
        if (send_to_print_stack_ == true) {
//          report = "<";
        }
        ir_event temp = {irps_number_ , em_number_, 0, 0, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when first beam has been broken
        return temp;
      }

      //    second infrared beam has been broken in a valid state
      else if (irps_number == irps_number_ && irps_status_ == IRPSStatus::Entered && s1_us != 0 && s2_us == 0) {
        s2_us = micros();   // record beam break time
        irps_status_ = IRPSStatus::Exited;   // update status to show second beam break

        const float delta_time_us_ = get_delta_time_us();             //time between first and second beam breaks
        const float delta_clicks_ = delta_time_us_ * clicks_per_us;   //convert microseconds to clicks
        const float clicks_mm_ = delta_clicks_ / interbeam_dist_mm_;  //convert clicks to clicks per mm

        // reset when irps measurment invalid
        if (delta_time_us_ < 0.0f) {
          reset();
          String report;          // hold data for printing
          if (send_to_print_stack_ == true) {
            report = "r" + String(irps_number);
          }
          ir_event temp = {irps_number_ , em_number_, 0, 0, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when resetting
          return temp;
        }

        unsigned long wait_clicks_ = 0;             //reset calculation variable
        unsigned long pulse_clicks_ = 0;            //reset calculation variable

        //  if the electromagnet is enabled for pulsing, calculate required timing
        if (enabled_) {
          pulse_clicks_ = clicks_mm_ * pulse_dist_mm_;                  // pulse_clicks is number of clicks of EM pulse duration
          wait_clicks_ = clicks_mm_ * (max_dist_mm_ - pulse_dist_mm_);  // wait_clicks is number of clicks before starting EM pulse
        }

        String report;          // hold data for printing
        if (send_to_print_stack_ == true) {
          //     the following calculation could be moved out of interrupt time
//          float speed_ = interbeam_dist_um / (float) delta_time_;   //calculate speed as float for reporting
//          report = String(irps_number_) + "," + String(speed_, 2);
        }

        ir_event temp = {irps_number_ , em_number_, wait_clicks_, pulse_clicks_, report, irps_status_};
        return temp;
      }

      // reset due to invalid state
      String report;          // hold data for printing
      if (send_to_print_stack_ == true) {
        report = "b" + String(irps_number) + "," + String(int(irps_status_));
      }
      reset();
      ir_event temp = {irps_number_ , em_number_, 0, 0, report, irps_status_};  // return IRPS and EM, no need to return wait, pulse or report when
      return temp;
    }
   
};

//Intialise Infra-Red Position Sensors and ElectroMagnets
//**************************** IRPS PHYSICAL CONFIG PARAMETERS ***************************
//IRPS(int irps_number,int em_number,bool enabled,int interbeam_dist_mm, int beam2em_dist_mm, bool send_to_print_stack)
//note: array below allows for Super size, but will also work for Standard PPA
IRPS irps[] = {
  IRPS(0, 7, false, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(1, 0, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(2, 2, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(3, 1, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(4, 4, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(5, 5, true, interbeam_dist_mm, max_pulse_dist_mm, false),
  IRPS(6, 3, true, interbeam_dist_mm, max_pulse_dist_mm, false)
};
//~,1,0,2,4,5,3
//***************************************************************************************
const int irpsCount = sizeof(irps) / sizeof(IRPS);

static class IRPS& active_irps() {
  return irps[irps_number];
}

enum class EMStatus {
  Off = 0,
  Pending = 1,
  On = 2 
};

class EM
{
    // Class Member Variables, initialized at startup
    int em_number_;   // the number of the EM
    bool enabled_;    // flag indicates whether the EM can be turned on
    EMStatus status_;       // EM - pending, on, off
    unsigned long actual_pulse_clicks_;    // the requested pulse duration
    unsigned long safetime_;    // the maximum allowed pulse duration

    // These maintain the current state
    // Constructor - creates an EM
    // and initializes the member variables and state
  public:
    EM(int em_number, bool enabled, unsigned long safetime)
    {
      em_number_ = em_number;
      enabled_ = enabled;
      safetime_ = safetime;
    }

    void reset() {
      digitalWrite(out_flag, EM_DISABLE);   // disable EM
      noInterrupts();                       // disable all interrupts
      TIMSK1 &= ~(1 << OCIE1A);             // disable future interrupts ie. CTC interrupt to give a 'one-shot' effect
      status_ = EMStatus::Off;              // recognise that EM is Off
      interrupts();                         // enable all interrupts
    }

    void set_pulse_clicks(unsigned long pulse_clicks) {
      actual_pulse_clicks_ = min(pulse_clicks, safetime_); // restrict pulse duration to maximum safe value
    }
    unsigned long get_pulse_clicks() {
      return actual_pulse_clicks_;
    }

    EMStatus get_status() {
      return status_;
    }
    void set_status(EMStatus newStatus) {
      status_ = newStatus;
    }

    void setup_timer(ir_event fire) {
      digitalWrite(out_flag, EM_DISABLE);   // ensure EM is OFF during waiting period

      set_pulse_clicks(fire.pulse_duration);

//      digitalWrite(scope0_toggle, LOW);     // display wait period start on scope

      //    write EM output address prior to energising output line
      digitalWrite(out_add0, em_number_ & 1);
      digitalWrite(out_add1, em_number_ & 2);
      digitalWrite(out_add2, em_number_ & 4);

      //    Set up interrupt to wait for right time to start the pulse

      noInterrupts();                         // disable all interrupts while interrupt parameters are changing
      TCCR1A = 0;                             // reset all bits
      TCCR1B = 0;                             // reset all bits
      OCR1A = fire.wait_duration;             // input is already in clicks, in theory should subtract 1
      TCCR1B |= (1 << WGM12);                 // CTC mode
      TCCR1B |= (1 << CS11);                  // 8 prescaler.  On a 16MHz Arduino, a prescaler of 8 gives a 0.5usecond click
      TIFR1 |= (1 << OCF1A);                  // clear

      TIMSK1 |= (1 << OCIE1A);                // enable timer compare interrupt
      TCNT1 = 0;                              // set counter to 0
      status_ = EMStatus::Pending;            // Set EM is in waiting period 
      interrupts();                           // enable all interrupts
    }

};    //end of EM class

//***************************** EM CONFIG PARAMETERS ************************************
//max pulse duration in microseconds
//(int em_number,bool output_to_em,int safetime)
//note: array below allows for Super size, but will also work for Standard PPA
EM em[] = {
  EM(0, true, em_max_safe_clicks),
  EM(1, true, em_max_safe_clicks),
  EM(2, true, em_max_safe_clicks),
  EM(3, true, em_max_safe_clicks),
  EM(4, true, em_max_safe_clicks),
  EM(5, true, em_max_safe_clicks),
};
//***************************************************************************************
const int emCount = sizeof(em) / sizeof(EM);

static class EM& active_em() {
  return em[active_irps().get_em_number()];
}

// **********   Hardware timing interrupt event  ***************

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{
  auto& em_ = active_em();

  //for end of pulse, send disable output quickly
  if (em_.get_status() == EMStatus::On) {
    digitalWrite(out_flag, EM_DISABLE);   // disable EM
    noInterrupts();                       // disable all interrupts
    TIMSK1 &= ~(1 << OCIE1A);             // disable future interrupts ie. CTC interrupt to give a 'one-shot' effect
    em_.set_status(EMStatus::Off);        // clear status to show EM is off
    interrupts();                         // enable all interrupts
  }

  //for start of pulse, send activate output
  else if (em_.get_status() == EMStatus::Pending) {
//    digitalWrite(scope0_toggle, HIGH);    // display wait period start on scope

    const auto actual_pulse = active_em().get_pulse_clicks();

    digitalWrite(out_flag, EM_ACTIVE);    // enable the EM

    noInterrupts();                       // disable all interrupts
    TCCR1A = 0;                           // reset all bits
    TCCR1B = 0;                           // reset all bits
    OCR1A = actual_pulse;                 // input is already in clicks, in theory should subtract 1
    TCCR1B |= (1 << WGM12);               // CTC mode
    TCCR1B |= (1 << CS11);                // 8 prescaler.  On a 16MHz Arduino, a prescaler of 8 gives a 0.5usecond click
    TIFR1 |= (1 << OCF1A);                // clear

    TIMSK1 |= (1 << OCIE1A);              // enable timer compare interrupt
    TCNT1 = 0;                            // set counter to 0
    em_.set_status(EMStatus::On);         // Set EM is now energised
    interrupts();                         // enable all interrupts
  }
}

void setup() {

  Serial.begin (115200);                      // start serial communication.

  // high baud rate required to send data in limited time
  pinMode(ir_interrupt, INPUT);
  pinMode(ir_add0, INPUT);
  pinMode(ir_add1, INPUT);
  pinMode(ir_add2, INPUT);
  pinMode(out_flag, OUTPUT);
  pinMode(out_add0, OUTPUT);
  pinMode(out_add1, OUTPUT);
  pinMode(out_add2, OUTPUT);
  pinMode(display_dc, OUTPUT);
  pinMode(display_cs, OUTPUT);
  pinMode(display_sda, OUTPUT);
  pinMode(display_sc, INPUT);
//  pinMode(scope0_toggle, OUTPUT);

//  digitalWrite(scope0_toggle, LOW);       // initialise with turned off value
  digitalWrite(out_flag, EM_DISABLE);       // initialise with turned off value

  ppaPID.SetOutputLimits(1, max_pulse_dist_um); // PID is used to modulate how long the pulse is in micrometers
  ppaPID.Start(0.0f, 0.0f, updateThrottle(thr));       // 

  //SCREEN
  tft.begin();
  tft.setRotation(3);
  tft.setTextWrap(false);
  tft.setAddrWindow(0, 0, tft.width(), tft.height());
  display_main();

  printqueue.setPrinter(Serial);
  printqueue.enqueue('\0');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('=');
  printqueue.enqueue('\0');

  irps_number = 0;
  last_irps_number = 0;

  // reset all irps
  for (int i = 0; i < irpsCount; i++) {
    irps[i].reset();
    irps[i].set_pulse_dist_mm(max_pulse_dist_mm);
  }

  // reset all em
  for (int i = 0; i < emCount; i++) {
    em[i].reset();
  }

  attachInterrupt(digitalPinToInterrupt(2), demuxINT, RISING);     // define interrupt based on rising edge of pin 2

}

//MAIN LOOP
void loop() {
  wdt_enable(WDTO_500MS);

  auto& irps_ = active_irps();
  auto& em_ = active_em();

  // Output to serial connection
  if (irps_.get_irps_number() == 0 && irps_.get_status() != IRPSStatus::Entered && em_.get_status() == EMStatus::Off) {
    while (!printqueue.isEmpty ()) {
      char next = printqueue.dequeue();
      if (next != '\0') {
        Serial.print(next);
      } else {
        Serial.println("");
      }
    }
  }

  if (irps_.get_status() != IRPSStatus::Exited || em_.get_status() != EMStatus::Off) {
    return;
  }

  // update once after each EM pulse
  if (last_irps_number != irps_.get_irps_number()) {
    last_irps_number = irps_.get_irps_number();

    float delta_time_us = irps_.get_delta_time_us();

    const double target_speed = updateThrottle(thr);

    ppaPID.Setpoint(target_speed);

    if (delta_time_us > 0) {
      const double speed = interbeam_dist_um / delta_time_us;  // micrometers divided by microseconds is equivalent to metres per second

      const double output_mm = ppaPID.Run(speed) / 1000.0f;

      irps_.set_pulse_dist_mm(output_mm);

      const auto yPos = map(speed * 100, 0, 600, tft.height(), 49);  // scale speed according to graph area on scale for plotting purposes
  
      //Manage screen scrolling
      xPos = xPos + 1;
      if (xPos > tft.width()) {
        tft.fillRect(0, 49, tft.width(), tft.height() - 49, ILI9341_BLACK);
        xPos = 0;
      }
  
      tft.drawPixel( xPos, yPos, ILI9341_WHITE );
      tft.drawPixel( xPos, yPos - 1, ILI9341_WHITE );

      tft.setCursor(0, 35);
      tft.print(speed, 2);
      tft.print(" - ");
      tft.print(target_speed, 2);
    }
  }
}   //end of main loop

void demuxINT() {
 
  //demultiplexes hardware ir_interrupt and calls relevant Speed Sensor (IRPS) based on address
  irps_number = (~PIND & B00111000) >> 3;

  if (irps_number < 0 || irps_number >= irpsCount) {
    irps_number = 0;
    printqueue.enqueue('$');
    printqueue.enqueue('\0');
    return;
  }

  // notify all of interrupt
  for (int i = 0; i < irpsCount; i++) {
    //call function to process IR interrupt
    struct ir_event current_event = irps[i].handleStatus(irps_number);

    // if a pulse is required, then send parameters to EM function
    if (current_event.irps_status == IRPSStatus::Exited && current_event.pulse_duration > 0 ) {
      em[current_event.em_number].setup_timer(current_event);
    }

    // enqueue log output
    int string_length = current_event.print_string.length();
    if (string_length > 2) {
      for (int i = 0; i <= string_length && !printqueue.isFull(); i++) {
        printqueue.enqueue(current_event.print_string.charAt(i));
      }
    }
  }
}

void display_main() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.println("~ PPA! ~ PID");
  tft.println("");
  tft.print("Actual - Target (Speeds in m/sec)");
  tft.setTextSize(2);
}
