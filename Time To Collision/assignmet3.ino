/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground through 220 ohm resistor

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

int arraysize = 9 ;
float DistanceArry[9];
float UniqueDistanceArry[9] ;

float ProbabilityArry[9] ;
//float distributionProbabilityArry[9] ;
float Measure_Distance = 100 ;




int i = 0 ;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}



float Distance_last = 0 ;
float Distance_prev = 0 ;

float time_in_ms = 0 ;



void loop() {
    // read the analog in value:

      
    
  sensorValue = analogRead(analogInPin);
  float  outputValue = map(sensorValue, 0, 1023, 0, 5000)/1000.0; 
  float Distance = 29.988 * pow(outputValue , -1.173) ; // change the analog out value: analogWrite(analogOutPin, outputValue);
      
  
  Distance = Distance*10 ;

  Distance_prev = Distance_last ;
  Distance_last = Distance ;

    Serial.print("Distance_prev is : ");
    Serial.print(Distance_prev , 2 );
    Serial.print("Distance_last is : ");
    Serial.println(Distance_last , 2 );
  if(i!=0){
     float passed_dis = (Distance_prev - Distance_last)/1000  ;

  float current_speed = passed_dis / (time_in_ms/1000) ;
  
  float time_to_collision = Distance_last / (current_speed * 1000) ;



  Serial.print("Time to Collsion is : ");
  Serial.print(time_to_collision , 2 );
  Serial.print("passed_dis is : ");
  Serial.print(passed_dis , 2 );
  Serial.print("current speed : ");
  Serial.println(current_speed , 2 );


    if(time_to_collision < 1  && passed_dis > 0.04 && current_speed > 0.02){
       Serial.println("warning --->>>>>>  time to colision after "  );
         Serial.println(time_to_collision , 2 );
        
        analogWrite(11, 255); // turn the LED on (HIGH is the voltage level)
      
         delay(7000);
        analogWrite(11, 0); // turn the LED on (HIGH is the voltage level)
       
    }

  

  
  

  Serial.print(" I = ");
  Serial.print(i);
  Serial.print(" Distance = ");
  Serial.println(Distance);


 
   
  }
  time_in_ms =  500 ;
  delay(time_in_ms);

 
  i++ ;

 

}
