#include <ZumoShield.h>

ZumoMotors motors;

// ── Pin assignments ──────────────────────────────────────────
#define FRONT A3
// #define FRONT_RIGHT_SENSOR A0
#define LEFT_SENSOR        A0
// #define FRONT_LEFT_SENSOR  A2
#define RIGHT_SENSOR       A1

#define DELAY_TURN 248   // defaul 235
#define RIGHT_SUB 16

// Speed - 275, delay turn - 190, wall_threshold - 250;      

// ── Tuning parameters ────────────────────────────────────────
const int   BASE_SPEED       = 325;    // forward cruise speed (0-400) // defaul - 250
const int   TURN_SPEED       = 212;    // spin speed during 180° turn  // default - 200
const int   TURN_SPEED_90    = 213;    // spin speed during 90° turn   // default - 200
const int   WALL_THRESHOLD   = 220;    // front sensor triggers 180° turn above this           // defaul - 250 125
const int   SIDE_WALL_MIN    = 20;     // side sensor minimum to activate centering
const int   SIDE_TURN_THRESH = 350;    // side sensor triggers 90° right turn above this  !!!!!!!!!!!!!!  lower number => harder to detect walls => straighter movement 100
const int   TURN_90_MS       = 325;    // ms to spin ~90° (tune this)
const int   COOLDOWN_MS      = 420;   // ms to ignore side sensors after a 90° turn
const float KP               = 0.09f; // turns angles - 0.15f default  // 0.10f is fine
const float ALPHA            = 0.15f; //
const float DEADBAND         = 20.0f;

float THRESHOLD_NEEDED_FOR_TURN = 575;   // default - 500

// Sensors
// float frontLeft;
// float frontRight;
float front;
float leftDist;
float rightDist;

// ── Smoothed sensor state ────────────────────────────────────
float smoothLeft  = -1;
float smoothRight = -1;

// ── Cooldown timer ───────────────────────────────────────────
unsigned long cooldownUntil = 0;  // millis() timestamp when cooldown expires

#define abs(x) ((x)>0?(x):-(x))




bool IF_THRESHOLD_NEEDED_FOR_TURN(){
  return abs(rightDist - leftDist) > THRESHOLD_NEEDED_FOR_TURN;
}

bool frontWall() {
  // return frontLeft > WALL_THRESHOLD || frontRight > WALL_THRESHOLD;
  return front > WALL_THRESHOLD;
}

bool leftWall() {
  return leftDist > SIDE_TURN_THRESH;
}

bool rightWall() {
  return rightDist > SIDE_TURN_THRESH;
}

void moveForward(){
  // ── 5. Centering (also skipped during cooldown) ───────────
  int correction = 0;

  bool wallLeft  = smoothLeft  > SIDE_WALL_MIN;
  bool wallRight = smoothRight > SIDE_WALL_MIN;
  float error;
  if (wallLeft || wallRight)
  {
    error = (smoothLeft-smoothRight);

    if (error > DEADBAND)
      correction = (int)(KP * (error - DEADBAND));
    else if (error < -DEADBAND)
      correction = (int)(KP * (error + DEADBAND));

    // correction = KP*error;
    
  }

  int leftSpeed  = constrain(BASE_SPEED + correction, 0, 400);
  int rightSpeed = constrain(BASE_SPEED - RIGHT_SUB - correction, 0, 400);

  motors.setLeftSpeed(leftSpeed);
  motors.setRightSpeed(rightSpeed);
  Serial.print("\n error:");
  Serial.print(error);
  Serial.print("\n leftSpeed: ");
  Serial.print(leftSpeed);
  Serial.print("\n rightSpeed: ");
  Serial.print(rightSpeed);
}

void turnRight(int delay_turn){
  // if(!IF_THRESHOLD_NEEDED_FOR_TURN()){
  //   return;
  // }

  delay(delay_turn);
  Serial.print("\nT Right: ");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  // 90° right turn: left wheel forward, right wheel backward
  motors.setLeftSpeed( TURN_SPEED_90);
  motors.setRightSpeed(-TURN_SPEED_90);
  delay(TURN_90_MS);  // tune TURN_90_MS until it turns ~90°

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  motors.setSpeeds(BASE_SPEED, BASE_SPEED - RIGHT_SUB);
  delay(delay_turn); ////////////////////
  
  // Ignore side sensors for COOLDOWN_MS so it doesn't immediately re-trigger
  cooldownUntil = millis() + COOLDOWN_MS;
  
  
}

void turnLeft(int delay_turn){

  // if(!IF_THRESHOLD_NEEDED_FOR_TURN()){
  //   return;
  // }

  delay(delay_turn);
  Serial.print("\n T Left: ");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  // 90° left
  motors.setLeftSpeed( -TURN_SPEED_90);
  motors.setRightSpeed(TURN_SPEED_90);
  delay(TURN_90_MS);  // tune TURN_90_MS until it turns ~90°

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  motors.setSpeeds(BASE_SPEED, BASE_SPEED - RIGHT_SUB);
  delay(delay_turn); //////////////////

  // Ignore side sensors for COOLDOWN_MS so it doesn't immediately re-trigger
  cooldownUntil = millis() + COOLDOWN_MS;
  
}

void turn180(){
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  motors.setLeftSpeed( TURN_SPEED);
  motors.setRightSpeed(-TURN_SPEED);
  delay(700);

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);

  // Start cooldown so side sensors are ignored right after the turn
  cooldownUntil = millis() + COOLDOWN_MS;
}

void moveForwardNoError() {
  motors.setSpeeds(BASE_SPEED, BASE_SPEED - RIGHT_SUB);
  delay(DELAY_TURN);                           //
}



void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  rightDist  = analogRead(RIGHT_SENSOR);

  while(!rightWall()){
    motors.setSpeeds(BASE_SPEED, BASE_SPEED - RIGHT_SUB);
    rightDist  = analogRead(RIGHT_SENSOR);
  }
  turnLeft(DELAY_TURN);
  

  //delay(750); // to be modified 500
}



void loop()
{
  // ── 1. Read all sensors ───────────────────────────────────
  // frontLeft  = analogRead(FRONT_LEFT_SENSOR);
  // frontRight = analogRead(FRONT_RIGHT_SENSOR);
  front  = analogRead(FRONT);
  leftDist   = analogRead(LEFT_SENSOR);
  rightDist  = analogRead(RIGHT_SENSOR);

  // Serial.print("FL:"); Serial.print(frontLeft);
  // Serial.print(" FR:"); Serial.print(frontRight);
  Serial.print("F:"); Serial.print(front);
  Serial.print(" L:");  Serial.print(leftDist);
  Serial.print(" R:");  Serial.println(rightDist);

  // ── Smooth the side sensors ────────────────────────────
  if (smoothLeft  < 0) smoothLeft  = leftDist;
  if (smoothRight < 0) smoothRight = rightDist;
  smoothLeft  = ALPHA * leftDist  + (1.0f - ALPHA) * smoothLeft;
  smoothRight = ALPHA * rightDist + (1.0f - ALPHA) * smoothRight;


  if (millis() >= cooldownUntil) {
    //int randomNumber = random(1, 4);   //
    Serial.print("\n");
    if(!frontWall()){
      if (rightWall() && leftWall()) {
        Serial.print("1: "); 
        moveForward();
      }
      else if (!rightWall() && leftWall()) {
        Serial.print("2:"); 
        moveForwardNoError();

        //if (randomNumber == 1) turnRight();    //
        //else moveForwardNoError();             //
      }
      else if (rightWall() && !leftWall()) {
        Serial.print("3:"); 
        turnLeft(DELAY_TURN);

        //if (randomNumber == 1) turnLeft();
        //else moveForwardNoError();
      }
      else {
        Serial.print("4:");
        turnLeft(DELAY_TURN);
        //moveForward();

        //if (randomNumber == 1) turnLeft();        //
        //else if (randomNumber == 2) turnRight();  //
        //else moveForwardNoError();                       // without error?
      }

    }else {
      if (!rightWall() && !leftWall()) {
        Serial.print("5:");
        turnLeft(0);
         
        //if (randomNumber == 1) turnRight();
        //else turnLeft();
      }
      else if (!rightWall() && leftWall()) {
        Serial.print("6:"); 
        turnRight(0);
      }
      else if (rightWall() && !leftWall()) {
        Serial.print("7:"); 
        turnLeft(0);
      }
      else {
        Serial.print("8:"); 
        turn180();
      }
    } 
  }
  moveForward();

}