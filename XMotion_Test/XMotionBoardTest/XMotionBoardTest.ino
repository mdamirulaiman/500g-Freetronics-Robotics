#include <xmotion.h>

//Opponent Sensor
int RSens = A5; //Right Opponent Sensor Pin
int RFSens = 4; //Right Diagonal Opppnent Sensor Pin
int MSens = 2; //Middle Oppoent Sensor Pin
int LFSens = 1; //Left Diagonal Opponent Sensor Pin
int LSens = 0; //Left Opponent Sensor Pin
//Edge Sensor Connections
int LEdge = A4; //Left Line Sensor Pin
int REdge = A1; //Right Line Sensor Pin
int Start = 10; //Start Button Pin
// Led Connections
int Led1 = 8;
int Led2 = 9;
// Dipswitches Connections
int DS1    =     5  ;  //DS1
int DS2    =     6  ; //DS2
int DS3    =     7  ;//DS3
int SPD    =     A3  ;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(Start, INPUT);
  pinMode(DS1, INPUT);
  pinMode(DS2, INPUT);
  pinMode(DS3, INPUT);
  pinMode(SPD, INPUT);
  pinMode(LEdge, INPUT);
  pinMode(REdge, INPUT);
  pinMode(LSens, INPUT);
  pinMode(RSens, INPUT);
  pinMode(LFSens, INPUT);
  pinMode(MSens, INPUT);
  pinMode(RFSens, INPUT);
  digitalWrite(Start, HIGH);
  digitalWrite(DS1, HIGH);
  digitalWrite(DS2, HIGH);
  digitalWrite(DS3, HIGH);
  }
  
  void BlinkModeLed(int timex){
  digitalWrite(Led1, HIGH);
  digitalWrite(Led2, HIGH);
  delay(timex);
  digitalWrite(Led1, LOW);
  digitalWrite(Led2, LOW);
  delay(timex);
 }


// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int Module = digitalRead(Start);
  int Dip1 = digitalRead(DS1);
  int Dip2 = digitalRead(DS2);
  int Dip3 = digitalRead(DS3);
  int Trim = analogRead(SPD);
  int Left = analogRead(LEdge);
  int Right = analogRead(REdge);
  float BattV = xmotion.VoltageIn() * 1.0583;
 
  
  // print out the state of the button:
  Serial.print("Start Module : "); Serial.print(Module); Serial.print("  ");
  Serial.print("Dipswitch : "); Serial.print(Dip1); Serial.print(" | "); Serial.print(Dip2);  Serial.print(" | "); Serial.print(Dip3); Serial.print("  "); 
  Serial.print("Trimpots : "); Serial.print(Trim); Serial.print("  ");
  Serial.print("Voltage : "); Serial.print(BattV); Serial.print("  ");
  Serial.print("Line Sensor : "); Serial.print(Left);  Serial.print(" | "); Serial.print(Right); Serial.print("  ");
  Serial.print("Opponent Sensors : "); Serial.print(digitalRead(LSens)); Serial.print(digitalRead(LFSens)); Serial.print(digitalRead(MSens)); Serial.print(digitalRead(RFSens)); Serial.println(digitalRead(RSens)); 
  BlinkModeLed(20);
  delay(100);      // delay in between reads for stability
}
