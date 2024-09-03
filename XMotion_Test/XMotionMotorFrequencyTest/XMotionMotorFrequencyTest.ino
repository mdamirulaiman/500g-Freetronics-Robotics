
// PWM output pin
#define pwmL      3
#define pwmR      11
int SPD = A3;


// Variable to represent PWM value
int MotorSpeed = 0; 
int MotorSpeedraw = 0;

void setup(){

  pinMode(SPD,INPUT);
  Serial.begin(9600);
  // Set the PWM Frequency  
  setPwmFrequency(pwmL, 64);
  setPwmFrequency(pwmR, 64);
  
  // Define Pins
  pinMode(pwmL,OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop()
{

  // Set Motor A forward
 
  digitalWrite(12, HIGH );
  digitalWrite(13, HIGH );
  
  MotorSpeedraw = analogRead(SPD); 
  
  // Convert to range of 0-255
  
  MotorSpeed = map(MotorSpeedraw, 0, 1023, 0, 255);
   
   // Send PWM to output pin
   analogWrite(pwmL,MotorSpeed);
   analogWrite(pwmR,MotorSpeed);
   Serial.print("RAW Value : ");
   Serial.print(MotorSpeedraw);
   Serial.print("\t");
   Serial.print("PWM Value : ");
   Serial.println(MotorSpeed);
  
  
  }


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR0B = TCCR0B & 0b11111000 | mode;
  }
}
