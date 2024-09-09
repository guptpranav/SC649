#define C1 2
#define C2 3
#define IN3 8
#define IN4 12
#define ENB 9

// C1C2 directional pattern for encoder top view
//  CLOCK := 00 10 11 01 00
// ACLOCK := 00 01 11 10 00

int aprev = 0; // previous C1 encoder state
int bprev = 0; // previous C2 encoder state
int acurr = 0; // latest C1 encoder state
int bcurr = 0; // latest C2 encoder state

float tickCounter     = 0.0;  // counter for encoder ticks
float ticksPerSecond  = 0.0;  // encoder ticks per second
float RPM             = 0.0;  // rotations per minute 
const float shaftGearboxRatio = 25.0; // encoder to motor shaft gear ratio, measured by averaging
const float ticksPerEncoderRotation = 25.0; // ticks per encoder rotation, measured by averaging
const float ticksPerMotorRotation = shaftGearboxRatio*ticksPerEncoderRotation; // ticks per motor rotation

unsigned long currentTime    = 0; // current running time of script
unsigned long lastUpdateTime = 0; // last time the tick count was updated

void setup() {
  // setting L298 pins to OUTPUT mode
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // setting encoder pins to INPUT mode
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  lastUpdateTime = millis();

  Serial.begin(9600);
}

void loop() {
  // currentTime = millis();

  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // analogWrite(ENB, 25);

  // updateEncoderTicks();
  // if(updateRPM()) {
  //   Serial.println(RPM);
  // }

  // setting clockwise rotation
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  int N = 10; // number of reading to average the RPM over
  float avgRPM = 0.0; // average RPM for a PWM

  for (int pwm=0; pwm<256; pwm+=5) {
    analogWrite (ENB, pwm);
    Serial.print("pwm = ");
    Serial.print(pwm);

    for (int n=0; n<N; n++) {
      while(!updateRPM()) {
        currentTime = millis();
        updateEncoderTicks();
      }
      avgRPM += RPM/N;
    }
    Serial.print(", encoder shaft RPM = ");
    Serial.print(avgRPM*shaftGearboxRatio);
    Serial.print(", motor shaft RPM = ");
    Serial.println(avgRPM);    
    avgRPM = 0;
  }
}

// updates motor tick count directionally
void updateEncoderTicks() {
  // measuring current encoder states
  acurr = digitalRead(C1);
  bcurr = digitalRead(C2);

  // updating directed tick count
  if ((acurr!=aprev) || (bcurr!=bprev)) {
    if ((acurr == 1-bprev) && (bcurr == aprev)) {
      tickCounter -= 1; // clockwise state transition
    }
    else {
      tickCounter += 1; // anticlockwise state transition
    }
  }

  // updating previous encoder states
  aprev = acurr;
  bprev = bcurr;
}

// measuring average RPM value over a second of rotation and returns true when done
bool updateRPM() {
  if (currentTime - lastUpdateTime >= 1000) {
    ticksPerSecond = tickCounter;
    tickCounter    = 0;
    RPM            = 6.0*ticksPerSecond/ticksPerMotorRotation; // (360/TPMR) * TPS * (1/60);
    lastUpdateTime = currentTime;
    return true;
  }
  else {
    return false;
  }
}
