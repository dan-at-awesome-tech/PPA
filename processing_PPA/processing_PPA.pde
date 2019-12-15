
// Processing for PPA
// This program provides real-time data analysis for the Personal Particle Accelerator
// Note: you must plug the USB cable into the PPA Arduino before running this program, otherwise
// it will lock up as can't make serial connection

import processing.serial.*;

// Expected input data from USB in form
//  format   IRPS,speed,tf,offset
//  example  0,4.12,1.00,3.67

//Serial message constants
final int irps_key=0;
final int speed_key=1;
final int tf_key=2;
final int offset_key=3;

//Serial data variables
// DUT = Device Under Test: the IRPS which is the incoming side of the EM of interest
int DUT=1;
// CDEV = Calibration Device: the IRPS used to gather data from the outgoing side of the EM of interest
int CDEV = 0;


Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port
String serialString=null;
int sequence=0;    // record count of valid serial input
int last_x_sequence=0;
String[] list={"0","0","0","0","0","0"};
// 4 elements holds IRPS, speed, tf, offset
float[] coords = new float[4];
float[] last_coords = new float[4];

//menu for graph and menu selection
PImage img_para;  // Declare a variable of type PImage
PImage img_scatter;  // Declare a variable of type PImage
PImage img_awesome;
String menu_option="";
int scale_x=1;
int scale_y=1;
int mouseselection=0;  

//general graph
float left_margin=100;
float footer=100;
float header=180;
float right_margin=100;
float xmin;
float xmax;
float screenxstart;
float screenxend; 
float ymin;
float ymax;
float screenystart;
float screenyend;
float xincrement;
float yincrement;
String xaxis;
String yaxis;

//Scatter IO graph
float io_graphxmin=0.0;
float io_graphxmax=6;
float io_graphymin=0;
float io_graphymax=6;
float io_xincrement=1.0;
float io_yincrement=1.0;
String io_xaxis="Input speed (m/s)";
String io_yaxis="Output speed (m/s)";

//TF graph
float graphxmin=0.6;
float graphxmax=1.2;
float graphymin=0;
float graphymax=6;
float tf_xincrement=1;
float tf_yincrement=0.1;
String tf_xaxis="Tuning Factor (no units)";
String tf_yaxis="Offset (mm)";

void setup() {
  background(255,255,255);
  size(1000,1000);
  // menu images
  img_awesome=loadImage("Awesome.png");
  img_para = loadImage("optimisation screenshot2.jpg");
  img_scatter = loadImage("Scatter.jpg");  
  // Draw the image to the screen at coordinate
  image(img_awesome,200,0,600,200);
  image(img_para,500,200,300,200);
  image(img_scatter,100,200,300,200);  
textSize(19);
fill(255,255,255);
text("Optimisation",535,370);
fill(200,0,200);
rect(200,520,580,100);
textSize(40);
fill(0,0,0);
text("Supporter Crew Credits!",250,570);

  //initialise values from serial  
  coords[0] = -1;  // Assign value to first element in the array
  coords[1] = -1; // Assign default value to second element in the array
  coords[2] = -1;
  coords[3] = -1;
  last_coords[0] = -1;  // Assign value to first element in the array
  last_coords[1] = -1; // Assign default value to second element in the array
  last_coords[2] = -1;
  last_coords[3] = -1;

// Start serial connection.  This section could be commented out if you want to work on the code without USB  
String portName = Serial.list()[0];
//change the 0 to a 1 or 2 etc. to match your port
myPort = new Serial(this, portName, 500000); 
myPort.bufferUntil('\n');     
          
}

void draw() {    

//println(mouseX,mouseY);

  if (mouseselection==0) return;
  
  if (mouseselection==1) {

  // Input / Output graph menu selection
  if ((mouseX>100) && (mouseX<400) && (mouseY>200) && (mouseY<400)) {
  menu_option="io";
  setmaps(io_graphxmin,io_graphxmax,left_margin,width-right_margin,io_graphymin,io_graphymax,footer,height-header,io_xaxis, io_yaxis,io_xincrement,io_yincrement);
  menuresponse();
  stroke(255, 0,0,255);
  linegraph(xmin, ymin, xmax, ymax);
  }

  // Tuning Factor graph menu selection
  else if ((mouseX>500) && (mouseX<800) && (mouseY>200) && (mouseY<400)) {
    menu_option="TF";
    setmaps(graphxmin,graphxmax,left_margin,width-right_margin,graphymin,graphymax,footer,height-header,tf_xaxis,tf_yaxis,tf_xincrement, tf_yincrement);
  menuresponse();    
  }
  
  //Supporter Crew menu selection
  else if ((mouseX>200) && (mouseX<(200+580)) && (mouseY>520) && (mouseY<(520+100))) {
    menu_option="SC";
    background(128,128,128);
    textSize(40);
    fill(0,0,0);
    text("Supporter Crew Credits!",200,200);
    textSize(30);
    text("Imno",200,250);
    text("Cory Snavely",200,300);
    text("Kean",200,350);
    text("Richard Krege",200,400);
    text("Joerg Fricke",200,450);
    text("Colin Easton",200,500);
    text("Brian Brunswick",200,550);
    return;
  // screen does not return, but mouse selection options still operating    
  }

  else mouseselection=0;
  }
    
   if (menu_option=="TF") {
    TF_draw();
    return;
   }
    
    if (menu_option=="io") {
    io_draw();
    return;
   } 
}

void TF_draw(){
  
  if (coords[1] >= 0) {
  println(coords[0],coords[1],coords[2]);  
  
  // Color ranges from red (slow) to yellow (medium) to green (fast) to represent speed
  color slowcolor=color(255,0,0);
  color midcolor=color(255,255,0);
  color fastcolor=color(0,255,0);

  // Set the datapoint color to represent speed
  // Various speed representations as color are possible, the above non-linear representation
  // attempts to provide clarity on the top end of the speed range, to assist with achieving
  // high performance
  float speedmax=5;
  float speedval=coords[1]/speedmax;
  float colorcutover=0.5;
  float colorcutover2=0.75;
  color c3;
  // if slow speed, the use a single slow color
  if (speedval<=colorcutover) c3=slowcolor;
  // else if above slow speed, spread out the speed across two colour bands
  else if (speedval<=colorcutover2) c3=lerpColor(slowcolor, midcolor, 2*(speedval-colorcutover));
  // with a non-linear representation at the top end of speed
  else c3=lerpColor(midcolor, fastcolor, (16*(speedval-colorcutover2)*(speedval-colorcutover2)));
  fill(c3);
  stroke(c3);
  println(last_coords[0],last_coords[1],coords[2]);

  // Send the Tuning Factor (Horizontal) and Offset (Vertical) to the graph
  plotTFgraph(coords[2],coords[3]);
  last_coords[0]=coords[0];
  last_coords[1]=coords[1];  
  last_coords[2]=coords[2];
  last_coords[3]=coords[3];
  
  coords[1]=-1;    
  }
  
}

void io_draw(){
 
    if (coords[1] >= 0) {
  println(coords[0],coords[1],coords[2]);  

  //Plot solid red point
  fill(255, 0,0,255);
  stroke(255, 0,0,255);
  plotIOgraph(coords[0],coords[1]);

  //Overplot last point in white
  //fill(0, 255,255,255);
  //stroke(0, 255,255,255);

  // Use a color code to show TF value
  color c1=color(204,102,0);
  color c2=color(0,102,153);

  color c3 = lerpColor(c1, c2, coords[2]/1.6);
  //  color c3 = lerpColor(c1, c2, 0.3);
  
  fill(c3);
  stroke(c3);  
  
  plotIOgraph(last_coords[0],last_coords[1]);

  last_coords[0]=coords[0];
  last_coords[1]=coords[1];  
  last_coords[2]=coords[2];
  coords[1]=-1;  
  
  }
}

// Retrieve PPA data from USB serial connection
// Expected input data in form
//  IRPS,speed,tf,offset
//  0,4.12,1.00,3.67
void serialEvent (Serial myPort) {
  try {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

 // if data exists, process it 
  if (inString != null) {

 // if string is long enough (not mal-formed)    
    if (inString.length()>9) { 
//      println(inString);
      serialString=inString.trim();
      list = split(serialString, ',');
      sequence=sequence+1;
      int IRPS=int(list[irps_key]);
      if (list.length>1) {
        float speed=float(list[speed_key]);
        float TF_temp=float(list[tf_key]);
        float offset=float(list[offset_key]);
        if (IRPS==DUT)  processDUT(speed,TF_temp,offset);            //if IRPS is incoming side, then assign data
        else if ((IRPS==CDEV)&&(sequence==last_x_sequence+1)) process0(speed);           //else if IRPS is test IRPS (0) then process0 speed to make a paired dataset
       }
     }
   } 
  }
  catch(RuntimeException e) {
    e.printStackTrace(); }

}

// retain speed, tuning factor and offset of the first IRPS (DUT)
void processDUT(float temp1,float TF,float offset) {
  coords[0] = temp1;  // Assign value to first element in the array
  coords[1] = -1; // Assign default value to second element in the array
  coords[2] = TF;
  coords[3] = offset;
  last_x_sequence=sequence; 
//  println("Process DUT");
}

// retain only the speed of the Calibration IRPS (CDEV)
void process0(float temp2) {
  if (coords[0] >=0) {
    coords[1] = temp2; // Assign value to second element in the array
//println("Process CDEV");
 } 
 
}

// Draw a circular datapoint on the Tuning Factor graph
void plotTFgraph(float TF, float offset){
    if (TF >=xmin && TF <=xmax && offset>=ymin && offset<=ymax) {
    ellipsegraph(TF,offset,0.01,0.1); }
    //println("complete");
}

void plotIOgraph(float inspeed, float outspeed){
    if (inspeed >=xmin && inspeed <=xmax && outspeed>=ymin && outspeed<=ymax) {
    ellipsegraph(inspeed,outspeed,0.05,0.05);}
    //else out of bounds
}

// Create graph paper
void establishgraph(){
    draw_border_graph();
  vertlinesgraph(xmin,yincrement,xmax);
  horizlinesgraph(ymin,xincrement,ymax);
   textSize(14);
   unitsxgraph(xaxis);
   unitsygraph(yaxis);
}

// Setup graph vertical lines
void vertlinesgraph(float start,float increment,float end ){
 fill(255,255,255);
 stroke(255,255,255);
 textSize(10);

  for (float i = start; i < (end+increment/10); i = i+increment) {
      linegraph(i,ymin,i,ymax);   
      textgraph(nf(i,1,2), i, ymax-1.03*(ymax-ymin));                   //horizontal label
         }
}

// Setup graph horizontal lines
void horizlinesgraph(float start,float increment,float end ){
    for (float i = start; i < end; i = i+increment) {
      linegraph(xmin,i,xmax,i); 
      textgraph(nf(i,1,2), xmax-1.05*(xmax-xmin),i);                   //vert label
        } 
}

// Setup graph labels
void textgraph(String a, float b, float c){
   text(a,graphmapx(b),graphmapy(c));
}

// Setup graph Y unit labels
void unitsygraph(String axislabel){
  translate(width/2,height/2);  // Translate to the center
  rotate(-HALF_PI);                // Rotate by theta
  textAlign(RIGHT);            
                text(axislabel, 0,-height/2+height/25);
      rotate(HALF_PI); 
  textAlign(BASELINE);  
  translate(-width/2,-height/2);    // reset
}

// Setup graph x unit labels
void unitsxgraph(String axislabel){
    textgraph(axislabel, (xmax+xmin)/2,ymin-((ymax-ymin)/15)); 
}

// Draw border around the graph
void draw_border_graph(){
    noFill();
    rectMode(CORNERS);
    //rect(graphmapx(graphxmin),graphmapy(graphymin),graphmapx(graphxmax),graphmapy(graphymax));
    rectgraph(xmin,ymin,xmax,ymax);
} 

// Translate a rectangle in physics units to the allocated graph area
void rectgraph(float a,float b, float c, float d){
    rect(graphmapx(a),graphmapy(b),graphmapx(c),graphmapy(d));
}

// Translate a line in physics units to the allocated graph area
void linegraph(float a, float b, float c, float d){
   line(graphmapx(a),graphmapy(b),graphmapx(c),graphmapy(d));
}

// Translate an ellipse (eg. circle) in physics units to the allocated graph area
void ellipsegraph(float a, float b, float c, float d){
  ellipseMode(CENTER);
    //ellipse(graphmapx(a),graphmapy(b),graphmapx(c),graphmapy(d));
    //ellipse(graphmapx(a),graphmapy(b),map(c,0,xmax,0,c*(screenxend-screenxstart)/(xmax-xmin)),map(d,ymin,ymax,0,d*(screenyend-screenystart)/(ymax-ymin)));
    ellipse(graphmapx(a),graphmapy(b),c*(screenxend-screenxstart)/(xmax-xmin),d*(screenyend-screenystart)/(ymax-ymin));    
}

int screenmapy(float zzz){
return int(map(zzz,0,height,height,0));
}

int graphmapy(float zzz){
return int(screenmapy(map(zzz,ymin,ymax,screenystart,screenyend)));  
}  

int graphmapx(float zzz){
return int(map(zzz,xmin,xmax,screenxstart,screenxend));  
}  

void setmaps(float xstart, float xend, float val3, float val4, float val5, float val6, float val7, float val8, String val9, String val10, float val11, float val12) {
xmin=xstart;
xmax=xend;
screenxstart=val3;
screenxend=val4;
ymin=val5;
ymax=val6;
screenystart=val7;
screenyend=val8;
xaxis=val9;
yaxis=val10;
xincrement=val11;
yincrement=val12;
}

void mousePressed() {
  if (mouseButton == LEFT) {
     if (mouseselection==0) mouseselection=1;
  }
}

void menuresponse(){
  println(menu_option);  
  mouseselection=2;
  background(0,0,0);
  fill(0,0,0);
  stroke(255,255,255);
  image(img_awesome,200,0,600,200);  
  establishgraph();
}
