#include <xmotion.h>


//Opponent Sensor
int RSens = A5; //Right Opponent Sensor Pin
int RFSens = 4; //Right Diagonal Opppnent Sensor Pin
int MSens = 2; //Middle Oppoent Sensor Pin
int LFSens = 1; //Left Diagonal Opponent Sensor Pin
int LSens = 0; //Left Opponent Sensor Pin
//Edge Sensor Connections
int LEdge = A2; //Left Line Sensor Pin
int REdge = A1; //Right Line Sensor Pin
int Start = 10; //Start Button Pin
// Led Connections
int Led1 = 8;
int Led2 = 9;
// Dipswitches Connections
int DS1    =     5  ;  //DS1
int DS2    =     6  ; //DS2
int DS3    =     7  ;//DS3



int SPD = A3;


// Direction.
#define LEFT    0
#define RIGHT   1


// Global variables.
uint8_t searchDir = LEFT;

// Speed values
int IdleSpeed,IdleSpeed_raw; // Idle Speed while no sensor giving data.
int MaxSpeed = 255; // Max Speed when sensor detect opponent
int TurnSpeed = 200; // Left and Right Forward Turning Speed
int EdgeTurn = 200; // Turning Time variable when minisumo sees white line in milliseconds


/*******************************************************************************
 * Start Routine
 * This function should be called once only when the game start.
 *******************************************************************************/
 void startRoutine() {
  // Start delay.
delay(200);
uint32_t startTimestamp = millis();
if(digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0){ //forward full speed
	xmotion.MotorControl(255,255);
	delay(200);
}else if(digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0){ // left turn
	xmotion.MotorControl(-255,255);
	delay(250);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 300 ){
			break;
		}
	}
}else if(digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0{ // forward full speed
	xmotion.MotorControl(255,255);
	delay(200);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 250){
			break;
		}
	}
}else if(digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1){ // right turn
	xmotion.MotorControl(255,-255);
	delay(250);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 300 ){
			break;
		}
	}
}else if(digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0){ // left round turn
	xmotion.MotorControl(-255,255);
	delay(200);
	xmotion.MotorControl(255,150);
	delay(650);
	xmotion.MotorControl(255,-255);
	delay(200);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 350 ){
			break;
		}
	}
}else if(digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1){ // right round turn
	xmotion.MotorControl(255,-255);
	delay(200);
	xmotion.MotorControl(150,255);
	delay(650);
	xmotion.MotorControl(-255,255);
	delay(200);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 350 ){
			break;
		}
	}
}else if(digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1){ // backward round turn
	xmotion.MotorControl(255,150);
	delay(600);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 400 ){
			break;
		}
	}
}else if(digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1){ // backward turn
	xmotion.MotorControl(255,-255);
	delay(400);
	while(digitalRead(MSens) == 0){
		if(millis() - startTimestamp > 400 ){
			break;
		}
	}
}
}


/*******************************************************************************
 * Back Off
 * This function should be called when the ring edge is detected.
 *******************************************************************************/
void backoff(uint8_t dir) {
  
  // Reverse.
  xmotion.MotorControl(-255,-255);
  delay(200);

  // Stop the motors.
  xmotion.MotorControl(0,0);
  delay(20);
  
  // Rotate..
  if (dir == LEFT) {
    xmotion.MotorControl(-TurnSpeed,TurnSpeed);
  } else {
    xmotion.MotorControl(TurnSpeed,-TurnSpeed);
  }
  delay(EdgeTurn);

  // Start looking for opponent.
  // Timeout after a short period.
  uint32_t uTurnTimestamp = millis();
  while (millis() - uTurnTimestamp < 300) {
    // Opponent is detected if either one of the opponent sensor is triggered.
    if ( digitalRead(MSens) || digitalRead(LFSens) || digitalRead(RFSens) || digitalRead(LSens) || digitalRead(RSens) ) 
	{ 
      // Stop the motors.
      xmotion.MotorControl(0,0);
      delay(30);

      // Return to the main loop and run the attach program.
      return;
    }
  }

  // If opponent is not found, move forward and continue searching in the main loop..
  xmotion.MotorControl(IdleSpeed,IdleSpeed);
  delay(150);
}


/*******************************************************************************
 * Search
 *******************************************************************************/
void search() {
  // Move in circular motion.
  if (searchDir == LEFT) {
    xmotion.MotorControl(IdleSpeed,IdleSpeed);
  } else {
    xmotion.MotorControl(IdleSpeed,IdleSpeed);
  }
}


/*******************************************************************************
 * Attack
 * Track and attack the opponent in full speed.
 * Do nothing if opponent is not found.
 *******************************************************************************/
void attack() {
  uint32_t attackTimestamp = millis();

  // Opponent in front center.
  // Go straight in full speed.
  if (digitalRead(MSens)) {
    xmotion.MotorControl(MaxSpeed,MaxSpeed);
  }

  // Opponent in front left.
  // Turn left.
  else if (digitalRead(LFSens)) {
    xmotion.MotorControl(-IdleSpeed,MaxSpeed);
  }

  // Opponent in front right.
  // Turn right.
  else if (digitalRead(RFSens)) {
    xmotion.MotorControl(MaxSpeed,-IdleSpeed);
  }

  // Opponent in left side.
  // Rotate left until opponent is in front.
  else if (digitalRead(LSens)) {
    xmotion.MotorControl(-MaxSpeed,MaxSpeed);
    while (!digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 400) {
        break;
      }
    }
  }

  // Opponent in right side.
  // Rotate right until opponent is in front.
  else if (digitalRead(RSens)) {
    xmotion.MotorControl(MaxSpeed,-MaxSpeed);
    while (!digitalRead(MSens)) {
      // Quit if opponent is not found after timeout.
      if (millis() - attackTimestamp > 400) {
        break;
      }
    }
  }
  
}

void read_line(){
 int left_raw = analogRead(LEdge);
       Serial.print("Left : ");
       Serial.print(left_raw);
       Serial.print(" | ");
 int right_raw = analogRead(REdge);
       Serial.print("Right : ");
       Serial.println(right_raw);
}

/*******************************************************************************
 * Setup
 * This function runs once after reset.
 *******************************************************************************/
void setup() {
  pinMode(Start, INPUT);
  pinMode(LEdge, INPUT);
  pinMode(REdge, INPUT);
  pinMode(LSens, INPUT);
  pinMode(RSens, INPUT);
  pinMode(LFSens, INPUT);
  pinMode(MSens, INPUT);
  pinMode(RFSens, INPUT);
  pinMode(Led1, OUTPUT);
  pinMode(Led2, OUTPUT);
  pinMode(SPD,INPUT);
  pinMode(DS1,INPUT_PULLUP);
  pinMode(DS2,INPUT_PULLUP);
  pinMode(DS3,INPUT_PULLUP);
  Serial.begin(9600);
 
  
  // Stop the motors.
  xmotion.MotorControl(0,0);
  xmotion.StopMotors(10);

  // Wait until button is pressed.
  while (!digitalRead(Start)) {// While waiting, show the status of the edge sensor for easy calibration.
    read_line();
	if(analogRead(LEdge) < 500){
       digitalWrite(Led1, HIGH);		   
    }else{
      digitalWrite(Led1, LOW);}
    if(analogRead(REdge) < 500) {
       digitalWrite(Led2, HIGH);
    }else{
      digitalWrite(Led2, LOW);}
  delay(20);
  }

  // Wait until button is released.
  if (digitalRead(Start)){       //button is pressed and the value is 1
  // Turn on the LEDs.
  xmotion.ToggleLeds(100);
  // Start routine..
  startRoutine();}


}
/*******************************************************************************
 * Main program loop.
 *******************************************************************************/
void loop() {
  IdleSpeed_raw = analogRead(SPD);
  IdleSpeed = map(IdleSpeed_raw,0,1023,0,255);
  // Edge is detected on the left.
  if (analogRead(LEdge) < 400 && analogRead(REdge) > 400) {
    // Back off and make a U-Turn to the right.
    backoff(RIGHT);

    // Toggle the search direction.
    searchDir ^= 1;
  }
  
  // Edge is detected on the right.
  else if (analogRead(REdge) < 400 && analogRead(LEdge) > 400) {
    // Back off and make a U-Turn to the right.
    backoff(LEFT);
    
    // Toggle the search direction.
    searchDir ^= 1;
  }

  // Edge is not detected.
  else {
    // Keep searching if opponent is not detected.
    if ( !digitalRead(MSens) && !digitalRead(LFSens) && !digitalRead(RFSens) && !digitalRead(LSens) && !digitalRead(RSens) )
       {
      search();
    }
    
    // Attack if opponent is in view.
    else {
      attack();
    }
  }

  // Stop the robot if the button is pressed.
  if (digitalRead(Start) == 0) {  //change to(!digitalRead(Start)) for ir using remote
    // Stop the motors.
    xmotion.StopMotors(10);

    // Loop forever here.
    while (1);
  }

}
