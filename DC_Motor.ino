char data; // initialize data variable recieved from Jetson
int LED1=3; // initialize LED1 as digital pin 3
int LED2 = 5; // initialize LED2 as digital pin 5
// these pins are where the voltage is output to the motors
// to light up LEDs instead of running a motor, simply replace the motors with LEDs

void setup() {

Serial.begin(9600); // initialize serial data connection 
pinMode(LED1, OUTPUT); // setup LED1 as voltage output 
pinMode(LED2, OUTPUT); // setup LED2 as voltage output
}

void loop() {

// sets condition to read serial data from Jetson
 if (Serial.available() ) { 
  data= Serial.read(); // Assigns data sent from Jetson to 'data' variable
  Serial.println(data);
 }

// motor control structure
// this structure can be extrapolated to lighting up LEDs and other applications
 
 if(data == 'A') {
  analogWrite(LED1, 100); // analogWrite function sets voltage output level at a specific pin
  analogWrite(LED2,0);    // voltage output can go from 0 to about 1000      
 }
 if(data == 'B'){
  analogWrite(LED1, 200);
  analogWrite(LED2,0);
}
 if(data == 'C'){
  analogWrite(LED1,0);
  analogWrite(LED2,100);
}
 if(data == 'D'){
  analogWrite(LED1,0);
  analogWrite(LED2,200);
}
 if(data == 'E'){
  analogWrite(LED1,0);
  analogWrite(LED2,0);
 }
}
