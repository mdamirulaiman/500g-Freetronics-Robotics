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
int DS1 = 5  ;  // DS1
int DS2 = 6  ;  // DS2
int DS3 = 7  ;  // DS3
int SPD = A3 ;  // Trimpot

// Direction.
#define LEFT    0
#define RIGHT   1

// Global variables.
uint8_t searchDir = LEFT;

// Speed values
int IdleSpeed = 0; // Idle Speed while no sensor giving data.
int MaxSpeed = 80; // Max Speed when sensor detect opponent
int TurnSpeed = 60; // Left and Right Forward Turning Speed
int EdgeTurn = 150; // Turning Time variable when minisumo sees white line in milliseconds

/*******************************************************************************
	Start Routine
	This function should be called once only when the game start.
*******************************************************************************/
void startRoutine() {
	// Calculate MaxSpeed
	MaxSpeed = 1.6 * map(analogRead(SPD), 0, 1023, 0, 100);
	
	digitalWrite(Led2, HIGH);
	delay(1000);
	xmotion.CounterLeds(250, 4);
	digitalWrite(Led1, HIGH);
	while (!digitalRead(Start)) {// While waiting, show the status of the edge sensor for easy calibration.
		read_line();
		if (analogRead(LEdge) < 400 || analogRead(REdge) < 400) {
			digitalWrite(Led2, HIGH);
			} else {
			digitalWrite(Led2, LOW);
		}
	}
	
	// Wait until button is released.
	if (digitalRead(Start)) {      //button is pressed and the value is 1
		// Start routine..
		
		// Start delay.
		xmotion.CounterLeds(1000, 3); // 3 seconds blink
		xmotion.CounterLeds(500, 2); // 2 seconds blink
		xmotion.CounterLeds(250, 4); // 1 second blink
		uint32_t startTimestamp = millis();
		
		if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) { //forward full speed
			xmotion.Forward(IdleSpeed, 50);
			
			} else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 0) { // left turn
			xmotion.Left0(65, 100);
			xmotion.Right0(0, 10);
			while (!digitalRead(MSens)) {
				if (millis() - startTimestamp > 300 ) {
					break;
				}
			}
			} else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) { // forward full speed
			xmotion.Forward(MaxSpeed, 100);
			while (!digitalRead(MSens)) {
				if (millis() - startTimestamp > 200) {
					break;
				}
			}
			} else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1) { // right turn
			xmotion.Right0(68, 105);
			xmotion.Left0(0, 10);
			while (!digitalRead(MSens)) {
				if (millis() - startTimestamp > 180 ) {
					break;
				}
			}
			} else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 0) { // left round turn
			xmotion.Left0(68, 105);
			xmotion.Right0(0, 10);
			xmotion.ArcTurn(57, 15, 1400);
			xmotion.Left0(0, 25);
			xmotion.Right0(60, 150);
			while (!digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
				if (millis() - startTimestamp > 1650 ) {
					break;
				}
			}
			} else if (digitalRead(DS1) == 0 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1) { // right round turn
			xmotion.Right0(68, 105);
			xmotion.Left0(0, 10);
			xmotion.ArcTurn(18, 57, 1500);
			xmotion.Right0(0, 10);
			xmotion.Left0(60, 150);
			while (!digitalRead(MSens) || !digitalRead(LFSens) || !digitalRead(LSens)) {
				if (millis() - startTimestamp > 1650 ) {
					break;
				}
			}
			} else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 0 && digitalRead(DS3) == 1) { // reverse and wait
			xmotion.Backward(30, 200);
			//xmotion.StopMotors(10);
			while (!digitalRead(LSens) || !digitalRead(LFSens) || !digitalRead(MSens) || !digitalRead(RFSens) || !digitalRead(RSens)) {
				if (millis() - startTimestamp > 2000 ) {
					xmotion.StopMotors(10);
					break;
				}
			}
			} else if (digitalRead(DS1) == 1 && digitalRead(DS2) == 1 && digitalRead(DS3) == 1) { // backward turn
			xmotion.Right0(43, 250);
			while (!digitalRead(MSens)) {
				if (millis() - startTimestamp > 500) {
					break;
				}
			}
		}
	}
}


/*******************************************************************************
	Back Off
	This function should be called when the ring edge is detected.
*******************************************************************************/
void backoff(uint8_t dir) {
	
	// Reverse.
	xmotion.Backward(65, 130);
	
	// Stop the motors.
	xmotion.Forward(0, 10);
	
	// Rotate..
	if (dir == LEFT) {
		xmotion.MotorControl(-TurnSpeed, TurnSpeed);
		} else {
		xmotion.MotorControl(TurnSpeed, -TurnSpeed);
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
			xmotion.Forward(1, 10);
			xmotion.Backward(1, 10);
			
			// Return to the main loop and run the attach program.
			return;
		}
	}
	// If opponent is not found, move forward and continue searching in the main loop..
}


/*******************************************************************************
	Search
*******************************************************************************/
void search() {
	// Move in circular motion.
	if (searchDir == LEFT) {
		xmotion.Forward(map(IdleSpeed, 0, 100, 0, 65), 10);
		} else {
		xmotion.Forward(map(IdleSpeed, 0, 100, 0, 65), 10);
	}
}


/*******************************************************************************
	Attack
	Track and attack the opponent in full speed.
	Do nothing if opponent is not found.
*******************************************************************************/
void attack() {
	uint32_t attackTimestamp = millis();
	
	// Opponent in front center.
	// Go straight in full speed.
	if (digitalRead(MSens)) {
		xmotion.Forward(MaxSpeed, 100);
	}
	
	// Opponent in front left.
	// Turn left.
	else if (digitalRead(LFSens)) {
		xmotion.ArcTurn(0, IdleSpeed, 100);
	}
	
	// Opponent in front right.
	// Turn right.
	else if (digitalRead(RFSens)) {
		xmotion.ArcTurn(IdleSpeed, 0, 100);
	}
	
	// Opponent in left side.
	// Rotate left until opponent is in front.
	else if (digitalRead(LSens)) {
		xmotion.Left0(IdleSpeed, 160);
		while (!digitalRead(MSens)) {
			// Quit if opponent is not found after timeout.
			if (millis() - attackTimestamp > 200) {
				break;
			}
		}
	}
	
	// Opponent in right side.
	// Rotate right until opponent is in front.
	else if (digitalRead(RSens)) {
		xmotion.Right0(IdleSpeed, 160);
		while (!digitalRead(MSens)) {
			// Quit if opponent is not found after timeout.
			if (millis() - attackTimestamp > 200) {
				break;
			}
		}
	}
}

void read_line() {
	int left_raw = analogRead(LEdge);
	int right_raw = analogRead(REdge);
	Serial.print("Left : ");
	Serial.print(left_raw);
	Serial.print(" | ");
	Serial.print("Right : ");
	Serial.println(right_raw);
}

/*******************************************************************************
	Setup
	This function runs once after reset.
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
	pinMode(SPD, INPUT);
	pinMode(DS1, INPUT_PULLUP);
	pinMode(DS2, INPUT_PULLUP);
	pinMode(DS3, INPUT_PULLUP);
	Serial.begin(9600);
	
	
	// Stop the motors.
	xmotion.MotorControl(0, 0);
	xmotion.StopMotors(10);
	
	startRoutine();
}
/*******************************************************************************
	Main program loop.
*******************************************************************************/
void loop() {
	IdleSpeed = map(analogRead(SPD), 0, 1023, 0, 100);
	// Edge is detected on the left.
	if ((analogRead(LEdge) < 400 && analogRead(REdge) > 400) || (analogRead(LEdge) < 400 && analogRead(REdge) < 400)) {
		delay(10);
		if ((analogRead(LEdge) < 400 && analogRead(REdge) > 400) || (analogRead(LEdge) < 400 && analogRead(REdge) < 400)) {
			// Back off and make a U-Turn to the right.
			backoff(RIGHT);
			
			// Toggle the search direction.
			searchDir ^= 1;
		}
	}
	
	// Edge is detected on the right.
	else if (analogRead(REdge) < 400 && analogRead(LEdge) > 400) {
		delay(10);
		if (analogRead(REdge) < 400 && analogRead(LEdge) > 400) {
			// Back off and make a U-Turn to the right.
			backoff(LEFT);
			
			// Toggle the search direction.
			searchDir ^= 1;
		}
	}
	
	// Edge is not detected.
	else {
		// Keep searching if opponent is not detected.
		if ( !digitalRead(MSens) && !digitalRead(LFSens) && !digitalRead(RFSens) && !digitalRead(LSens) && !digitalRead(RSens) && analogRead(LEdge) > 400 && analogRead(REdge) > 400 )
		{
			search();
		}
		
		// Attack if opponent is in view.
		else {
			attack();
		}
	}
	
	// Stop the robot if the button is pressed.
	if (digitalRead(Start)) {  //change to(!digitalRead(Start)) for ir using remote
		// Stop the motors.
		xmotion.StopMotors(10);
		// Go Back to startRoutine()
		startRoutine();
	}
	
}
