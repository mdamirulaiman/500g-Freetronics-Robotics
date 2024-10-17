// Pins connected to RC receiver
#define RC_SPEED 13
#define RC_STEERING 12

// Pin Motor Assignment
#define LPWM_1 18 // Left motor forward
#define LPWM_2 16 // Left motor backward
#define RPWM_1 19 // Right motor forward
#define RPWM_2 17 // Right motor backward

#define OB_LED 2

#define NEUTRAL_SPEED 1500
#define NEUTRAL_STEERING 1500
#define RANGE_SPEED 1000
#define RANGE_STEERING 1000

void setup()
{
    Serial.begin(115200);
    pinMode(OB_LED, OUTPUT);
    pinMode(RC_SPEED, INPUT);
    pinMode(RC_STEERING, INPUT);

    pinMode(LPWM_1, OUTPUT);
    pinMode(LPWM_2, OUTPUT);
    pinMode(RPWM_1, OUTPUT);
    pinMode(RPWM_2, OUTPUT);
    stopmotor();
}

void loop()
{
    int speedPulse = pulseIn(RC_SPEED, HIGH, 40000);
    if (speedPulse == 0)
    {
        digitalWrite(OB_LED, LOW);
        stopmotor();
        return;
    }

    // Convert the pulse width to percentage (-1.0 to 1.0).
    float speedPercent = (float)(speedPulse - NEUTRAL_SPEED) / (float)(RANGE_SPEED / 2);
    speedPercent = constrain(speedPercent, -1.0f, 1.0f);

    // Add in dead band.
    if ((speedPercent > -0.1f) && (speedPercent < 0.1f))
    {
        speedPercent = 0.0f;
    }

    int steeringPulse = pulseIn(RC_STEERING, HIGH, 40000);
    if (steeringPulse == 0)
    {
        digitalWrite(OB_LED, LOW);
        stopmotor();
        return;
    }

    // Convert the pulse width to percentage (-1.0 to 1.0).
    float steeringPercent = (float)(steeringPulse - NEUTRAL_STEERING) / (float)(RANGE_STEERING / 2);
    steeringPercent = constrain(steeringPercent, -1.0f, 1.0f);

    // Add in dead band.
    if ((steeringPercent > -0.1f) && (steeringPercent < 0.1f))
    {
        steeringPercent = 0.0f;
    }

    float leftSpeedPercent = speedPercent + steeringPercent;
    int leftSpeed = (int)(leftSpeedPercent * 255.0f);
    leftSpeed = constrain(leftSpeed, -255, 255);

    float rightSpeedPercent = speedPercent - steeringPercent;
    int rightSpeed = (int)(rightSpeedPercent * 255.0f);
    rightSpeed = constrain(rightSpeed, -255, 255);

    Set_Motor(leftSpeed, rightSpeed);

    if ((leftSpeed != 0) || (rightSpeed != 0))
    {
        digitalWrite(OB_LED, HIGH);
    }
    else
    {
        digitalWrite(OB_LED, LOW);
    }

    Serial.print("speedPulse:");
    Serial.print(speedPulse);
    Serial.print(" | ");
    Serial.print("steeringPulse:");
    Serial.print(steeringPulse);
    Serial.print(" | ");
    Serial.print("leftSpeed:");
    Serial.print(leftSpeed);
    Serial.print(" | ");
    Serial.print("rightSpeed:");
    Serial.println(rightSpeed);
}

void stopmotor()
{
    analogWrite(LPWM_1, 255);
    analogWrite(LPWM_2, 255);
    analogWrite(RPWM_1, 255);
    analogWrite(RPWM_2, 255);
}

void Set_Motor(float Lval, float Rval)
{
    if (Lval > 0)
    { // Forward
        analogWrite(LPWM_1, Lval);
        analogWrite(LPWM_2, 0);
    }
    else if (Lval < 0)
    { // Reverse
        Lval = abs(Lval);
        analogWrite(LPWM_2, Lval);
        analogWrite(LPWM_1, 0);
    }

    if (Rval > 0)
    { // Forward
        analogWrite(RPWM_1, Rval);
        analogWrite(RPWM_2, 0);
    }
    else if (Rval < 0)
    { // Reverse
        Rval = abs(Rval);
        analogWrite(RPWM_2, Rval);
        analogWrite(RPWM_1, 0);
    }
}
