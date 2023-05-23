//zmienne do sprzegiel
#define REL1 19
#define REL2 18
#define REL3 5
#define REL4 17
//zmienne do silnika
#define RPM_HALL 35
#define RPM_MOTOR 32

//zmienna pomocnicza
uint8_t currByte = 0, i = 0;

//tablica do trzymania bitów
char crcTab[300], lineBuffer[7];

//zmienna do zliczania obrotow na minute
float hall_rotation = 100.0;

//zmienne do PWM
const float ratedSpeed = 3000.0;
const int resolution = 8;
int dutyCycle = 0;

void CalulateTable_CRC8()  //fill table
{
  uint8_t generator = 0x83;  //ATM-8 CRC (8bit)
  for (int divident = 0; divident < 256; divident++) {
    uint8_t currByte = divident;
    //calculate the CRC-8 value for current byte
    for (uint8_t bitSec = 0; bitSec < 8; bitSec++) {
      if ((currByte & 0x80) != 0) {
        currByte <<= 1;
        currByte ^= generator;
      } else {
        currByte <<= 1;
      }
    }
    //store CRC value in lookup table
    crcTab[divident] = currByte;
  }
  return;
}

char computeCRC(char message[], uint8_t nBytes) {
  uint8_t crc = 0x0;

  for (int b = 0; b < nBytes; ++b) {
    uint8_t data = message[b] ^ crc;
    crc = crcTab[data];
  }

  return crc;
}

void sendWorking()
{
  if(digitalRead(REL1) == HIGH)
    Serial.write("Generator 1 is working");
  else if(digitalRead(REL2) == HIGH)
    Serial.write("Generator 2 is working");
  else if(digitalRead(REL3) == HIGH)
    Serial.write("Generator 3 is working");
  else
    Serial.write("No generator is working"); 
}

void readFromHallAndSet()
{
  //reading value from hall
  float hall_count = 1.0;
  bool on_state = false;
  float start = micros();

  while(true)
  {
    if(digitalRead(RPM_HALL) == 0)
    {
      if(on_state == false)
      {
        on_state == true;
        hall_count += 1.0;
      }
    } else
    {
      on_state == false;
    }
    if(hall_count >= hall_rotation)
      break;
  }

  float end_time = micros();
  float time_passed = ((end_time - start)/1000000.0);
  Serial.print("Time passed: ");
  Serial.print(time_passed);

  float rpm_val = (hall_count/time_passed)*60.0;
  Serial.print("RPM: ");
  Serial.print(rpm_val);
  
  ///setting speed to motor
  //float PulseWidth = rpm_val/ratedSpeed;
  dutyCycle = map(rpm_val, 0, 3000, 0, 255);
  //dutyCycle = 255*rpm_val/(ratedSpeed);
  analogWrite(RPM_MOTOR, dutyCycle);
}

void setup() {
  //uruchomienie obslugi serial
  Serial.begin(115200);

  //konfiguracja wyjsc
  pinMode(REL1, OUTPUT);
  pinMode(REL2, OUTPUT);
  pinMode(REL3, OUTPUT);
  pinMode(REL4, OUTPUT);

  //do silnika
  pinMode(RPM_HALL, INPUT);
  pinMode(RPM_MOTOR, OUTPUT);

  //wywolanie tego wypelnianai tabeli
  void CalulateTable_CRC8();
}

//sprawdzanie czy tabilca ma okresolne cyfry

void loop()
{
  //sprawdzam czy serial sie wlaczyl
  if (Serial.available() > 0) {
    currByte = Serial.read();
    //sprawdzam ostatni znak
    if (currByte == '\r' || currByte == '\n')
    {
      if(i > 0)
      {
        if(lineBuffer[i] != computeCRC(lineBuffer, i))
        {
          //i cisne z sygnalami
          if((lineBuffer[0] == '6') && (lineBuffer[1] == '9') && (lineBuffer[2] == '0') && (lineBuffer[3] == '1') && (lineBuffer[4] = '1') && (lineBuffer[5] == '1'))
          {
            if(lineBuffer[6] == '0')
              {
                digitalWrite(REL1, LOW);
                digitalWrite(REL2, LOW);
                digitalWrite(REL3, LOW);

                char lineBuffer[7] = {'6','9','0','2','1','1','0'}; 
                Serial.write(lineBuffer);
                Serial.print("No generator is working");
              } else if(lineBuffer[6] == '1')
              {
                digitalWrite(REL1, HIGH);
                digitalWrite(REL2, LOW);
                digitalWrite(REL3, LOW);

                char lineBuffer[7] ={'6','9','0','2','1','1','1'}; 
                Serial.write(lineBuffer);
                Serial.print("First generator is working");
              } else if(lineBuffer[6] == '2')
              {
                digitalWrite(REL1, LOW);
                digitalWrite(REL2, HIGH);
                digitalWrite(REL3, LOW);

                char lineBuffer[7] ={'6','9','0','2','1','1','2'}; 
                Serial.write(lineBuffer);
                Serial.print("Second generator is working");
              } else if(lineBuffer[6] == '3')
              {
                digitalWrite(REL1, HIGH);
                digitalWrite(REL2, HIGH);
                digitalWrite(REL3, LOW);

                char lineBuffer[7] ={'6','9','0','2','1','1','3'}; 
                Serial.write(lineBuffer);
                Serial.print("First and second generators are working");
              } else if(lineBuffer[6] == '4')
              {
                digitalWrite(REL1, LOW);
                digitalWrite(REL2, LOW);
                digitalWrite(REL3, HIGH);

                char lineBuffer[7] ={'6','9','0','2','1','1','4'}; 
                Serial.write(lineBuffer);
                Serial.print("Third generator is working");
              } else if(lineBuffer[6] == '5')
              {
                digitalWrite(REL1, HIGH);
                digitalWrite(REL2, LOW);
                digitalWrite(REL3, HIGH);

                char lineBuffer[7] ={'6','9','0','2','1','1','5'}; 
                Serial.write(lineBuffer);
                Serial.print("First and third generators are working");
              } else if(lineBuffer[6] == '6')
              {
                digitalWrite(REL1, LOW);
                digitalWrite(REL2, HIGH);
                digitalWrite(REL3, HIGH);

                char lineBuffer[7] ={'6','9','0','2','1','1','6'}; 
                Serial.write(lineBuffer);
                Serial.print("Second and third generators are working");
              } else if(lineBuffer[6] == '7')
              {
                digitalWrite(REL1, HIGH);
                digitalWrite(REL2, HIGH);
                digitalWrite(REL3, HIGH);

                char lineBuffer[7] ={'6','9','0','2','1','1','7'}; 
                Serial.write(lineBuffer);
                Serial.print("All generators are working");
              }
              else if(lineBuffer[6] == '8')
              {
                //zasprzeglam silnik, ustawiem mu predkosc i go odsperzegam
                digitalWrite(REL4, HIGH);
                readFromHallAndSet();
                digitalWrite(REL4, LOW);

                char lineBuffer[7] ={'6','9','0','2','1','1','8'}; 
                Serial.write(lineBuffer);
              }
          }
          if(lineBuffer[0] == '6' && lineBuffer[1] == '9' && lineBuffer[2] == '2' && lineBuffer[3] == '1' && lineBuffer[4] == '6' &&lineBuffer[5] == '7')
          {
            sendWorking();
          }
        }
      }
    }
    else
    {
      if(i > 6)
      {
        i = 0;
      }
      lineBuffer[i] = currByte;
      i++;
    }
  }
  else
  {
    printf("Serial sie nie wlaczyl");
  }
}