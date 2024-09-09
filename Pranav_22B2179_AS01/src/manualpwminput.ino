// defining pinout constant values
#define C1 2
#define C2 3
#define IN3 8
#define IN4 12
#define ENB 9 

// period of manually generated PWM in milliseconds
#define N 200

void setup() {
  // setting L298 pins to OUTPUT mode
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // setting encoder pins to INPUT mode
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  // starting serial communication at 57600 bauds
  Serial.begin(57600);
}

void loop() {
  // seeking direction input from user through ArduinoIDE serial monitor 
  Serial.println("input 1,-1, and 0 for clockwise, anticlockwise rotation, and stopping the motor, respectively:");
  while (Serial.available() == 0) {}
  int direction = Serial.parseInt();
  
  // seeking PWM input from user through ArduinoIDE serial monitor 
  Serial.println("input duty cycle:");
  while (Serial.available() == 0) {}
  float dutycycle = Serial.parseFloat();

  // switch case to set motor direction to user input
  switch(direction) {
    case -1:
      Serial.println("clockwise");
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
      break;
    case +1:
      Serial.println("anti-clockwise");
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
      break;
    case 0:
      Serial.println("stop");
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,LOW);
      break;
    default: // stop
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,LOW);
      break;
  }

  // assume duty cycle input upto 2 decimal places
  // generating PWM manually of frequency 1000/N
  Serial.println("frequency = " + String(1000/N) + " Hz");
  for (int i=1; i <= 1000/N; i++){
    digitalWrite(ENB, HIGH);
    delay(N*dutycycle);
    digitalWrite(ENB, LOW);
    delay(N*(1-dutycycle));
  }
}