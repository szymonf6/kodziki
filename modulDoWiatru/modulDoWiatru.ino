/*
  Stepper Motor Test
  stepper-test01.ino
  Uses MA860H or similar Stepper Driver Unit
  Has speed control & reverse switch

  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Define pins
int HALL_SENSOR_A = 27;
int HALL_SENSOR_B = 25;
int driverPUL = 18; // PUL+ pin
int driverDIR = 17; // DIR+ pin
int ENA_pin = 21;
// int spd = 35;     // Potentiometer

// Variables
int pd = 600;   // Pulse Delay period
boolean setdir; // Set Direction
int A;
int B;

void setup()
{
    Serial.begin(9600);
    pinMode(driverPUL, OUTPUT);
    pinMode(driverDIR, OUTPUT);
    pinMode(ENA_pin, OUTPUT);
    pinMode(HALL_SENSOR_A, INPUT);
    pinMode(HALL_SENSOR_B, INPUT);
}

void loop()
{

    // pd = map((analogRead(spd)),0,4096,2000,50);
    A = analogRead(HALL_SENSOR_A);
    B = analogRead(HALL_SENSOR_B);
    B = B + 280; // skalowanie czujnika
    Serial.print("A =");
    Serial.print(A);
    Serial.print(" B = ");
    Serial.print(B);
    Serial.print("\r\n");
    // Serial.println(B);

    digitalWrite(ENA_pin, HIGH);
    if (A > B + 200)
    {
        digitalWrite(ENA_pin, LOW); // załączanie LOWEM

        setdir = LOW; // lewo
        digitalWrite(driverDIR, setdir);
        digitalWrite(driverPUL, HIGH);
        delayMicroseconds(pd);
        digitalWrite(driverPUL, LOW);
        delayMicroseconds(pd);
    }

    else if (B > A + 200)
    {
        digitalWrite(ENA_pin, LOW);

        setdir = HIGH; // lewo
        digitalWrite(driverDIR, setdir);
        digitalWrite(driverPUL, HIGH);
        delayMicroseconds(pd);
        digitalWrite(driverPUL, LOW);
        delayMicroseconds(pd);
    }

    else
    {
        digitalWrite(ENA_pin, HIGH);
    }
}

void stepper()
{

    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, LOW);
}