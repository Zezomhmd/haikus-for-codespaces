#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>


/**********************************************************************************************/
/*************************           Bluetooth Module                 *************************/
/**********************************************************************************************/

// Define SoftwareSerial object and the pins used for RX and TX for bluetooth module
SoftwareSerial bluetoothSerial(2, 3);  // RX, TX    // for bluetooth (TX > 2 , RX > 3)

/**********************************************************************************************/




/**********************************************************************************************/
/*************************         Servo motor and it's driver        *************************/
/**********************************************************************************************/


// Servo pulse width limits
#define SERVOMIN 150  //0   deg
#define SERVOMAX 600  //180 deg

#define step 30  // the step is 5 degrees

#define SERVO_BASE_PIN 0    // Servo motor for the base
#define SERVO_LINK_1_PIN 2  // Servo motor for the link
#define SERVO_LINK_2_PIN 3  // Servo motor for the link
#define SERVO_GRIP_PIN 8    // Servo motor for the gripper
#define SERVO_HAND_PIN 9    // Servo motor for the hand

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int currentBaseAngle = 90;   // Initial angle for the base servo
int currentLink1Angle = 120;  // Initial angle for the link servo (for both servo motors that move the link)
int currentLink2Angle = 0;
int currentHandAngle = 90;     // Initial angle for the hand servo
int currentGripperAngle = 90;  // Initial angle for the gripper servo


int BaseAngle = 90;
int Link1Angle = 120;
int Link2Angle = 0;
int HandAngle = 90;
int GripperAngle = 90;



uint16_t angleToPulse(int angle);


/**********************************************************************************************/



/**********************************************************************************************/
/*************************                DC  Motor Driver            *************************/
/**********************************************************************************************/

#define DELAY_COUNT 300

/****** motor pins *****/
#define in1 6
#define in2 5
#define in3 8
#define in4 7

void up();
void back();
void right();
void left();
void of();

/**********************************************************************************************/

#define ARM 0
#define CAR 1

//Flag to indicate the mode(state)
unsigned char modeFlag = CAR;  //The initial mode is CAR mode

void changeMode(void);
unsigned char GetCurrentMode(void);

/***********************************************************************************************/





void setup() {

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  Serial.begin(9600);           // Serial communication for debugging
  bluetoothSerial.begin(9600);  // Bluetooth module communication

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {


  // Check if data is available from Bluetooth module
  if (bluetoothSerial.available() > 0) {
    char receivedChar = bluetoothSerial.read();

    if (receivedChar == 'M') {
      changeMode();
    }

    Serial.print("Received: ");
    Serial.println(receivedChar);

    Serial.print("Current Mode: ");
    Serial.println(modeFlag);

    if (GetCurrentMode() == CAR) {
      /*CAR mode*/
      switch (receivedChar) {
          /*Arm mode*/
        case 'U':
          up();
          break;

        case 'B':
          back();
          break;

        case 'R':
          right();
          break;

        case 'L':
          left();
          break;

        case 'S':
          of();
          break;
      }

    } else if (GetCurrentMode() == ARM) {
      /*ARM mode*/
      switch (receivedChar) {
          /*Arm mode*/
        case 'R':  // Rotate base servo clockwise
          currentBaseAngle += step;
          break;

        case 'L':  // Rotate base servo counterclockwise
          currentBaseAngle -= step;
          break;

        case 'U':  // Move link servo up
          currentLink1Angle += step;
          break;

        case 'B':  // Move link servo down
          currentLink1Angle -= step;
          break;

        case 'u':  // Move link servo up
          currentLink2Angle += step;
          break;

        case 'b':  // Move link servo down
          currentLink2Angle -= step;
          break;

        case 'H':  // Open hand
          currentHandAngle += step;
          break;

        case 'h':  // Close hand
          currentHandAngle -= step;
          break;

        case 'G':  // Open hand
          currentGripperAngle += step;
          break;

        case 'g':  // Close hand
          currentGripperAngle -= step;
          break;
      }
    }



  }  // end of > if (bluetoothSerial.available() > 0)

  // Ensure servo angles are within limits
  currentBaseAngle = constrain(currentBaseAngle, 0, 180);
  currentLink1Angle = constrain(currentLink1Angle, 0, 180);
  currentLink2Angle = constrain(currentLink2Angle, 0, 180);
  currentHandAngle = constrain(currentHandAngle, 0, 180);
  currentGripperAngle = constrain(currentGripperAngle, 0, 180);


  if (currentBaseAngle > BaseAngle) {
    for (int i = BaseAngle; i < currentBaseAngle; i++) {
      pwm.setPWM(SERVO_BASE_PIN, 0, angleToPulse(i));
      delay(20);
    }
    BaseAngle = currentBaseAngle;
  } else if (BaseAngle > currentBaseAngle) {
    for (int i = BaseAngle; i > currentBaseAngle; i--) {
      pwm.setPWM(SERVO_BASE_PIN, 0, angleToPulse(i));
      delay(20);
    }
    BaseAngle = currentBaseAngle;
  }



  if (currentLink1Angle > Link1Angle) {
    for (int i = Link1Angle; i < currentLink1Angle; i++) {
      pwm.setPWM(SERVO_LINK_1_PIN, 0, angleToPulse(i));
      delay(20);
    }
    Link1Angle = currentLink1Angle;
  } else if (Link1Angle > currentLink1Angle) {
    for (int i = Link1Angle; i > currentLink1Angle; i--) {
      pwm.setPWM(SERVO_LINK_1_PIN, 0, angleToPulse(i));
      delay(20);
    }
    Link1Angle = currentLink1Angle;
  }


  if (currentLink2Angle > Link2Angle) {
    for (int i = Link2Angle; i < currentLink2Angle; i++) {
      pwm.setPWM(SERVO_LINK_2_PIN, 0, angleToPulse(i));
      delay(20);
    }
    Link2Angle = currentLink2Angle;
  } else if (Link2Angle > currentLink2Angle) {
    for (int i = Link2Angle; i > currentLink2Angle; i--) {
      pwm.setPWM(SERVO_LINK_2_PIN, 0, angleToPulse(i));
      delay(20);
    }
    Link2Angle = currentLink2Angle;
  }


  if (currentHandAngle > HandAngle) {
    for (int i = HandAngle; i < currentHandAngle; i++) {
      pwm.setPWM(SERVO_HAND_PIN, 0, angleToPulse(i));
      delay(20);
    }
    HandAngle = currentHandAngle;
  } else if (HandAngle > currentHandAngle) {
    for (int i = HandAngle; i > currentHandAngle; i--) {
      pwm.setPWM(SERVO_HAND_PIN, 0, angleToPulse(i));
      delay(20);
    }
    HandAngle = currentHandAngle;
  }



  if (currentGripperAngle > GripperAngle) {
    for (int i = GripperAngle; i < currentGripperAngle; i++) {
      pwm.setPWM(SERVO_GRIP_PIN, 0, angleToPulse(i));
      delay(20);
    }
    GripperAngle = currentGripperAngle;
  } else if (GripperAngle > currentGripperAngle) {
    for (int i = GripperAngle; i > currentGripperAngle; i--) {
      pwm.setPWM(SERVO_GRIP_PIN, 0, angleToPulse(i));
      delay(20);
    }
    GripperAngle = currentGripperAngle;
  }


  // Update servo positions
  pwm.setPWM(SERVO_BASE_PIN, 0, angleToPulse(BaseAngle));     //Rotate the base
  pwm.setPWM(SERVO_LINK_1_PIN, 0, angleToPulse(Link1Angle));  //Move the link
  pwm.setPWM(SERVO_LINK_2_PIN, 0, angleToPulse(Link2Angle));
  pwm.setPWM(SERVO_HAND_PIN, 0, angleToPulse(HandAngle));     //Rotate the hand
  pwm.setPWM(SERVO_GRIP_PIN, 0, angleToPulse(GripperAngle));  //Open ond close the gripper

}  // end of > void loop()


// Function to convert angle to pulse width for the servo
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}


void changeMode(void) {
  if (modeFlag == ARM) {
    modeFlag = CAR;
  } else if (modeFlag == CAR) {
    modeFlag = ARM;
  }
}

unsigned char GetCurrentMode(void) {
  return modeFlag;
}


void up() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
}

void back() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
}


void right() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
}

void left() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
}


void of() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);
  digitalWrite(in3, 0);
  digitalWrite(in4, 0);
}
