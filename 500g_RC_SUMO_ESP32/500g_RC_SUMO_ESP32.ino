// Pins connected to RC receiver
#define RC_SPEED 13
#define RC_STEERING 12

// Pin Motor Assignment
#define LPWM_1 18 // Left motor forward
#define LPWM_2 16 // Left motor backward
#define RPWM_1 19 // Right motor forward
#define RPWM_2 17 // Right motor backward

#define OB_LED 2

#define NEUTRAL_SPEED 1470
#define NEUTRAL_STEERING 1470
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
        return;
    }

    // Convert the pulse width to percentage (-1.0 to 1.0).
    float steeringPercent = (float)(steeringPulse - NEUTRAL_STEERING) / (float)(RANGE_STEERING / 2);
    steeringPercent = constrain(steeringPercent, -1.0f, 1.0f);

    // Add in dead band.
    if ((steeringPercent > -0.1f) && (steeringPercent < 0.1f))
    {
        steeringPercent = 0.0f;
        stopmotor();
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
}

void stopmotor()
{
    digitalWrite(LPWM_1, HIGH);
    digitalWrite(LPWM_2, HIGH);
    digitalWrite(RPWM_1, HIGH);
    digitalWrite(RPWM_2, HIGH);
}

void Set_Motor(float Lval, float Rval)
{
    if (Lval >= 0)
    {
        analogWrite(LPWM_1, Lval);
        digitalWrite(LPWM_2, LOW);
    }
    else
    {
        Lval = abs(Lval);
        analogWrite(LPWM_2, Lval);
        digitalWrite(LPWM_1, LOW);
    }
    if (Rval >= 0)
    {
        analogWrite(RPWM_1, Rval);
        digitalWrite(RPWM_2, LOW);
    }
    else
    {
        Rval = abs(Rval);
        analogWrite(RPWM_2, Rval);
        digitalWrite(RPWM_1, LOW);
    }
}
