////////////////////////////////////////////////////////////////////////////////////////////////////////////// SENSOR VARIABLES 
#define POT_PIN A0
#define RANGE_PIN A1
#define LIGHT_PIN A5
#define FORCE_PIN A3
#define BUTTON_PIN 13
int buttonValue=0;
int min_range = 8;
int max_range =20;

////////////////////////////////////////////////////////////////////////////////////////////////////////////// Servo VARIABLES 
#include <Servo.h>
Servo servo;  // Create servo object

////////////////////////////////////////////////////////////////////////////////////////////////////////////// DC MOTOR VARIABLES 
//Pins 
const int PIN_ENA = 7; // PWM E1-2
const int PIN_IN1 = 6; // H-bridge I1
const int PIN_IN2 = 5; // H-bridge I2
const int PIN_ENC_A = 3; // Encoder chA
const int PIN_ENC_B = 4; // Encoder chB

//Encoder config 
const long COUNTS_PER_REV = 360;
const long POS_TOL_COUNTS = 3;
const float VEL_TOL_CPS = 5.0;

//PID Control 
const float KPOS = 60;
float Kp = 1, Ki = 1, Kd = 0.0;

const int PWM_MIN_MOVE = 0;
const int PWM_MAX = 255;
const float ITERM_CLAMP = 200.0f;

//State 
volatile long encCount = 0;
long zeroOffset = 0;
bool activeMove = false;
long startCount = 0, targetCount = 0;
float userMaxVel_dps = 0, userMaxVel_cps = 0;
long lastCount = 0;
unsigned long lastMicros = 0;
float vel_cps = 0.0f;
float iTerm = 0.0f, lastVelErr = 0.0f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////// STEPPER MOTOR VARIABLES 
#define direction 9
#define step 10
#define EN 12

unsigned long lastStepTime = 0;
bool stepState = LOW;
char user_input;
int x;
int y;
int state;
float duration, distance;
bool cycleReset = false;

////////////////////////////////////////////////////////////////////////////////////////////////////////////// HELPER FUNCTION VARIABLES 
int mode = 0;
int sensorSel = 0;
int motorSel = 0;
int motorMode = 0;
int pwm = 0;
int other = 0;

bool cmdReady = false;
int sensorSelected = 0;
int motorSelected = 0;
int sensorValue = 0;
static int toggleState = 0;  




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////// SENSOR SETUP 
  pinMode(RANGE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////// Servo SETUP 
  servo.attach(11);  

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////// DC Motor SETUP 
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncB, CHANGE);
  motorStopCoast();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////// Stepper motor SETUP 
  pinMode(step, OUTPUT);
  pinMode(direction, OUTPUT);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  readCommand();
  if (cmdReady) {
    //reset 
    cmdReady = false;
    cycleReset = true; //for stepper motor
    motorStopCoast();
    


    Serial.print("mode="); Serial.print(mode);            // Modes:     (1)sensors (2)Gui
    Serial.print(" sensor="); Serial.print(sensorSel);    // Sensor:    (0)no use  (1)Pot     (2)Range    (3)Light    (4)Force
    Serial.print(" motor="); Serial.print(motorSel);      // Motor:     (1)Servo   (2)DC      (3)Stepper
    Serial.print(" motormode="); Serial.print(motorMode); // MotorMode: (0)not use (1)custom1 (22)custom2  (for user to use as a switch ccw,cw )
    Serial.print(" pwm="); Serial.print(pwm);             // pwm:   000-255
    Serial.print(" other="); Serial.println(other);       // other: 000-999  custom value for users

   
  }

  //send data to ROS2
  Serial.print(analogRead(POT_PIN));
  Serial.print(",");
  Serial.print(sensor_read_range(RANGE_PIN));
  Serial.print(",");
  Serial.print(analogRead(LIGHT_PIN));
  Serial.print(",");
  Serial.print(analogRead(FORCE_PIN));
  Serial.print(",");
  Serial.println(toggleState);


  //MAIN LOOP

  // Sensor Mode
  if(mode==1){

    if(sensorSel==1){
      sensorValue = signal_to_motor(POT_PIN);
    }
    else if(sensorSel==2){
      sensorValue = signal_to_motor(RANGE_PIN);
    }
    else if(sensorSel==3){
      sensorValue = signal_to_motor(LIGHT_PIN);
    }
    else if(sensorSel==4){
      sensorValue = signal_to_motor(FORCE_PIN);
    }
    else if(sensorSel==5){
      sensorValue = debounceToggleButton(BUTTON_PIN);
    }


    if(motorSel==1){
      servo_move(sensorValue);
    }
    else if(motorSel==2){
      controlUpdate();
      int speed = map(sensorValue,0,255,0,480);
      startMovement(3600,speed);
    }
    else if(motorSel==3){
      SpeedControl(sensorValue);
    }
  }


  //GUI mode
  else if(mode==2){
    if(motorSel==1){ //Servo
      int gui_pwm = map(other,0,180,0,255);
      constrain(gui_pwm,0,255);
      servo_move(gui_pwm);
    }
    else if(motorSel==2){ //DC
      controlUpdate();
      if (motorMode == 1){
        if (pwm == 001){
          startMovement(other/2, 500); //(angle,speed)
          pwm=000;
        }
        else if (pwm == 002){
          startMovement(3600, other);} //(angle,speed)
        } 
      else if (motorMode == 2){
          if (pwm == 001){
            startMovement(-other/2, 500); //(angle,speed)
            pwm=000;
          }
          else if (pwm == 002){
            startMovement(-3600, other); //(angle,speed)
          }
      }
    }
    else if(motorSel==3){// Stepper
      PositionControl(motorMode,other);
    }

  }
            
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////// SENSOR FUNCTIONS 
int debounceToggleButton(int pin) {
  static int lastStableState = LOW;
  static int lastReading = LOW;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 30;

  int reading = digitalRead(pin);
  // Reset debounce timer on change
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  // Accept stable state
  if ((millis() - lastDebounceTime) > debounceDelay) {

    // Detect rising edge (button press)
    if (reading == HIGH && lastStableState == LOW) {
      if (toggleState == 0) {
        toggleState = 255;
      } else {
        toggleState = 0;
      }
    }
    lastStableState = reading;
  }

  lastReading = reading;
  return toggleState;
}


long sensor_read_range(int sensorpin){
  long sensor_value = analogRead(RANGE_PIN);
  long mm = sensor_value * 5 + 150; 
  long inches = mm/25.4; 
  
  if(inches > max_range){
    inches = max_range;
  }
  if(inches < min_range){
    inches = min_range; 
  }
  return inches;
}

int sensor_read(int sensorpin){
  int rawsensorvalue = analogRead(sensorpin);
  return rawsensorvalue;
}

int signal_filter(int x) {
  static int y = 0;          // remembers last value
  y = y + (x - y) / 4;       // alpha = 0.25 (fast)
  return y;
}

int signal_to_motor(int sensorpin){
  int value_to_motors;
  if(sensorpin == RANGE_PIN){
    long inches = sensor_read_range(sensorpin);
    value_to_motors = map(inches,min_range,max_range,0,255);
  }
  else{
    int rawsensorvalue = sensor_read(sensorpin);
    value_to_motors = map(rawsensorvalue, 0, 1023,0,255);
  }

  value_to_motors = signal_filter(value_to_motors);
  return value_to_motors ;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////// Servo FUNCTIONS 
void servo_move(int signal){
  //invert for direction
  signal = 255-signal;
  static float smoothSignal = 0;   // remembers last value
  float alpha = 0.2;               // 0.1 = very smooth, 0.5 = fast

  // Exponential smoothing
  smoothSignal = alpha * signal + (1 - alpha) * smoothSignal;

  int angle = map((int)smoothSignal, 0, 255, 0, 180);
  servo.write(angle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////// DC MOTOR FUNCTIONS 
// Main DC Motor
void startMovement(float deg, float dps) {
  long c = getCount();
  startCount = c;
  float countsF = (deg / 360.0f) * (float)COUNTS_PER_REV;
  targetCount = startCount + (long)lround(countsF);
  userMaxVel_dps = dps;
  userMaxVel_cps = (dps / 360.0f) * (float)COUNTS_PER_REV;

  iTerm = 0.0f;
  lastVelErr = 0.0f;
  activeMove = true;
}

// interrupt
void isrEncA() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  encCount += (a == b) ? 1 : -1;
}
void isrEncB() {
  bool a = digitalRead(PIN_ENC_A);
  bool b = digitalRead(PIN_ENC_B);
  encCount += (a != b) ? 1 : -1;
}

// Motor drive helpers
void motorStopCoast() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

void motorDriveSignedPWM(int pwmSigned) {
  int pwm = constrain(abs(pwmSigned), 0, PWM_MAX);
  if (pwm == 0) {
    motorStopCoast();
    return;
  }

  if (pwmSigned > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  analogWrite(PIN_ENA, pwm);
}

long getCount() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c - zeroOffset;
}

//  Main control update 
void controlUpdate() {
  unsigned long now = micros();
  if (lastMicros == 0) {
    lastMicros = now;
    lastCount = getCount();
    return;
  }

  float dt = (now - lastMicros) / 1e6f;
  if (dt < 0.01f) return;
  lastMicros = now;

  long c = getCount();
  long dc = c - lastCount;
  lastCount = c;

  vel_cps = dc / dt;

  if (!activeMove) {
    motorStopCoast();
    iTerm = 0;
    lastVelErr = 0;
    return;
  }

  long posErr = targetCount - c;
  long absErr = labs(posErr);

  if (absErr <= POS_TOL_COUNTS && fabs(vel_cps) <= VEL_TOL_CPS) {
    activeMove = false;
    motorStopCoast();
    return;
  }

  float dir = (posErr >= 0) ? 1.0f : -1.0f;
  float velFromPos = KPOS * (float)absErr;
  float targetVel_cps = dir * min(userMaxVel_cps, velFromPos);

  float velErr = targetVel_cps - vel_cps;
  iTerm += (velErr * dt) * Ki;
  iTerm = constrain(iTerm, -ITERM_CLAMP, ITERM_CLAMP);

  float dTerm = (velErr - lastVelErr) / dt;
  lastVelErr = velErr;

  float u = Kp * velErr + iTerm + Kd * dTerm;

  int pwmSigned = (int)u;
  if (fabs(targetVel_cps) > 1.0f) {
    if (pwmSigned > 0) pwmSigned = max(pwmSigned, PWM_MIN_MOVE);
    if (pwmSigned < 0) pwmSigned = min(pwmSigned, -PWM_MIN_MOVE);
  }

  pwmSigned = constrain(pwmSigned, -PWM_MAX, PWM_MAX);
  motorDriveSignedPWM(pwmSigned);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////// STEP MOTOR FUNCTIONS 

void SpeedControl(int sensorValue)
{

  int stepInterval= map(sensorValue, 0, 255, 1, 5); 
  for (x=0; x<10; x++)
  {
    digitalWrite(step, HIGH);
    delay(stepInterval); 
    digitalWrite(step, LOW);
    delay(stepInterval);
  }

}

// Position Control for stepper motor
// CW direction = motorMode == 1
// CCW direction = motorMode ==2
void PositionControl(int motorMode, int angle)
{
  digitalWrite(EN, LOW);
  int steps = round((1/1.8) * angle);

  if (motorMode == 1){
    digitalWrite(direction, LOW);
  }
  else if (motorMode == 2){
    digitalWrite(direction, HIGH);
  }


  static int x=0;
  if(cycleReset){
    x=0;
    cycleReset=false;
  }

  for(x; x<steps; x++)  //Loop the stepping enough times for motion to be visible
  {
    digitalWrite(step,HIGH); //Trigger one step
    delay(1);
    digitalWrite(step,LOW); //Pull step…
    delay(1);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////// HELPER FUNCTIONS 
void readCommand() {
  static char data[11];   // 10 digits + '\0'
  static byte idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;
    if (c == '\n') {
      if (idx == 10) {   // exactly 10 digits
        mode      = data[0] - '0';
        sensorSel = data[1] - '0';
        motorSel  = data[2] - '0';
        motorMode  = data[3] - '0';

        pwm = (data[4] - '0') * 100 +
               (data[5] - '0') * 10  +
               (data[6] - '0');

        other = (data[7] - '0') * 100 +
               (data[8] - '0') * 10  +
               (data[9] - '0');

        if (pwm <= 255) {
          cmdReady = true;   
        }
      }
      idx = 0;              // reset for next packet
      return;
    }

    if (idx < 10) {
      data[idx++] = c;
    } else {
      idx = 0;              // overflow = reset
    }
  }
}
