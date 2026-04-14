/*
 * Team Id: KB_1758
 * Author List: Anuj Tale, Aditya Tangde, Prajwal Sontakke
 * Institute: Shri Guru Gobind Singhji Institute of Engineering and Technology, Nanded
 * Filename: KB_1758_Task6.ino
 * Theme: Krishi Balancer

 * Functions: rightEncoderISR(), ISR(PCINT2_vect), setMotors(int, int),
 *            processBluetooth(), updateAngle(), calculateBalance(),
 *            setup(), loop()

 * Global Variables: dmpReady, fifoBuffer, q, gravity, ypr,
 *                   currentAngle, prevAngle,
 *                   leftEncoderCount, rightEncoderCount,
 *                   theta, thetaDot, positionError, positionErrorDot,
 *                   prevTheta, prevPositionError, firstRun,
 *                   prevThetaDot, inRecoveryMode, recoveryStartTime,
 *                   targetPosition, moveForward, moveBackward, turboMode,
 *                   turning, turnTargetDiff, turnBias,
 *                   leftMotorPower, rightMotorPower,
 *                   armPos, clawPos,
 *                   buzzerActive, buzzerStartTime,
 *                   lastTime, sampleTime
 */

#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Servo.h>

/* ====================== */
/* ==== LQR GAINS ======= */
/* ====================== */
/*
 * Control Law:
 * u = -K1*theta - K2*thetaDot - K3*positionError - K4*positionErrorDot
 */

#define K_THETA          13.5    // Tilt angle gain
#define K_THETA_D        1.2    // Angular velocity gain
#define K_POSITION       0.12    // Position error gain
#define K_POSITION_D     0.040   // Position error rate gain

/* ====================== */
/* ==== CONSTANTS ======= */
/* ====================== */

#define BALANCE_ANGLE      -2.5     // Mechanical balance offset (deg)
#define POSITION_RAMP_STEP  12      // Acceleration increment per loop

#define FRICTION_OFFSET     16      // PWM added to overcome static friction
#define FRICTION_DEADBAND    8      // Minimum control signal to apply friction offset

#define TURN_PWM             35     // Differential PWM during a turn
#define TURN_ENCODER_COUNTS  180    // Encoder count difference to complete one turn

#define MAX_MOTOR_POWER 255         // Motor PWM saturation limit
#define FALL_ANGLE 45.0             // Tilt angle (deg) beyond which robot is considered fallen

#define BLUETOOTH_BAUD 9600         // UART baud rate for Bluetooth module

#define RIGHT_MOTOR_BOOST 1.19      // Right motor compensation multiplier

#define DISTURBANCE_THRESHOLD 15.0  // Angular acceleration threshold to trigger recovery
#define RECOVERY_BOOST 1.150         // Gain multiplier during disturbance recovery
#define RECOVERY_DURATION 350       // Duration (ms) for recovery boost after disturbance

/* ====================== */
/* ==== HARDWARE PINS ==== */
/* ====================== */

// Motor Driver Pins
#define ENA 6
#define IN1 A2
#define IN2 A3
#define ENB 5
#define IN3 9
#define IN4 4

// Encoder Pins
#define ENCODER_R_A 2
#define ENCODER_R_B 3
#define ENCODER_L_A 7
#define ENCODER_L_B 8

// Peripheral Devices
#define ARM_SERVO_PIN   10
#define CLAW_SERVO_PIN  11
#define BUZZER_PIN A1

/* ====================== */
/* ==== GLOBAL OBJECTS === */
/* ====================== */

MPU6050 mpu;      // MPU6050 IMU object communicating over I2C at address 0x68
Servo armServo;   // Servo object for arm actuation
Servo clawServo;  // Servo object for claw actuation

/* ====================== */
/* ==== MPU VARIABLES ==== */
/* ====================== */

bool dmpReady = false;     // Set true once MPU6050 DMP initializes successfully
uint8_t fifoBuffer[64];    // DMP FIFO packet buffer, 64 bytes
Quaternion q;              // Quaternion output from DMP
VectorFloat gravity;       // Gravity vector derived from quaternion
float ypr[3];              // Yaw, Pitch, Roll in radians from DMP

float currentAngle = 0.0; // Filtered tilt angle in degrees used for LQR control
float prevAngle    = 0.0; // Previous raw DMP angle used in complementary filter

/* ====================== */
/* ==== ENCODERS ========= */
/* ====================== */

volatile long leftEncoderCount  = 0; // Cumulative left wheel encoder ticks; range: -LONG_MAX to LONG_MAX
volatile long rightEncoderCount = 0; // Cumulative right wheel encoder ticks; range: -LONG_MAX to LONG_MAX

/* ====================== */
/* ==== LQR STATES ======= */
/* ====================== */

float theta             = 0.0; // Current tilt angle error from balance point in degrees
float thetaDot          = 0.0; // Angular velocity of tilt in degrees per second

float positionError     = 0.0; // Difference between target and current encoder position; clamped to ±350
float positionErrorDot  = 0.0; // Rate of change of position error in encoder counts per second

float prevTheta         = 0.0; // Theta value from previous control loop iteration
float prevPositionError = 0.0; // Position error from previous control loop iteration

bool firstRun = true; // True on the first valid loop iteration to prevent derivative kick

/* ====================== */
/* ==== RECOVERY MODE ==== */
/* ====================== */

float prevThetaDot         = 0.0;  // Angular velocity from previous iteration for disturbance detection
bool inRecoveryMode        = false; // True when sudden disturbance is detected
unsigned long recoveryStartTime = 0; // Timestamp in ms when recovery mode was triggered

/* ====================== */
/* ==== MOTION CONTROL ==== */
/* ====================== */

long targetPosition = 0;   // Target encoder count for position hold or motion; in encoder ticks

bool moveForward  = false; // True when 'F' Bluetooth command is active
bool moveBackward = false; // True when 'B' Bluetooth command is active
bool turboMode    = false; // True when turbo mode is toggled on via 'T' command

bool  turning        = false; // True during an active turn maneuver
long  turnTargetDiff = 0;     // Target (leftEncoderCount - rightEncoderCount) to complete a turn
float turnBias       = 0.0;   // Differential PWM bias for steering; positive = right turn

int leftMotorPower  = 0; // Final PWM for left motor; range: -MAX_MOTOR_POWER to +MAX_MOTOR_POWER
int rightMotorPower = 0; // Final PWM for right motor; range: -MAX_MOTOR_POWER to +MAX_MOTOR_POWER

/* ====================== */
/* ==== ARM & CLAW ======= */
/* ====================== */

int armPos  = 90; // Current arm servo position in degrees; range: ARM_DOWN(70) to ARM_UP(110)
int clawPos = 90; // Current claw servo position in degrees; range: CLAW_CLOSE(80) to CLAW_OPEN(125)

#define ARM_UP      92
#define ARM_DOWN    50
#define CLAW_OPEN  125
#define CLAW_CLOSE  95

/* ====================== */
/* ==== BUZZER ========== */
/* ====================== */

bool buzzerActive          = false; // True while buzzer is currently sounding
unsigned long buzzerStartTime = 0;  // Timestamp in ms when buzzer was last activated
#define BUZZER_DURATION  2000       // Buzzer on-time in milliseconds
#define BUZZER_FREQUENCY 2000       // Buzzer tone frequency in Hz

/* ====================== */
/* ==== TIMING ========== */
/* ====================== */

unsigned long lastTime = 0;    // Timestamp in ms of the previous control loop execution
float sampleTime       = 0.01; // Measured loop period in seconds; clamped between 0 and 0.05

/* ====================== */
/* ==== ENCODER ISRs ===== */
/* ====================== */

/*
 * Function Name: rightEncoderISR
 * Input: None (triggered by hardware interrupt on ENCODER_R_A pin change)
 * Output: None; updates global rightEncoderCount
 * Logic: Reads both encoder channels on every CHANGE edge of Channel A.
 *        If A == B the motor is spinning forward (increment count),
 *        else it is spinning in reverse (decrement count).
 *        This implements single-channel quadrature decoding.
 * Example Call: attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);
 */
void rightEncoderISR() {
  bool a = digitalRead(ENCODER_R_A);
  bool b = digitalRead(ENCODER_R_B);
  if (a == b) rightEncoderCount++; // Channels in phase: forward rotation
  else        rightEncoderCount--; // Channels out of phase: reverse rotation
}

/*
 * Function Name: ISR(PCINT2_vect)
 * Input: None (triggered by pin change interrupt on PCINT23 = ENCODER_L_A)
 * Output: None; updates global leftEncoderCount
 * Logic: Uses a static variable lastA to detect actual rising/falling edges
 *        on Channel A and ignore spurious triggers from other PCINT2 group pins.
 *        Direction is determined from relative phase of Channel A and Channel B.
 *        Sign convention for left wheel is inverted relative to right wheel
 *        to account for mirrored motor mounting.
 * Example Call: Called automatically on pin change; enabled via PCICR and PCMSK2
 */
ISR(PCINT2_vect) {
  static bool lastA = HIGH;
  bool a = digitalRead(ENCODER_L_A);
  // Ignore if no actual edge on Channel A (other PCINT2 pin triggered ISR)
  if (a == lastA) return;
  bool b = digitalRead(ENCODER_L_B);
  if (a == b) leftEncoderCount--; // Inverted sign for left wheel due to mirrored mounting
  else        leftEncoderCount++;
  lastA = a;
}

/*
 * Function Name: setMotors
 * Input: leftSpeed  -> PWM value for left motor; range: -MAX_MOTOR_POWER to +MAX_MOTOR_POWER
 *        rightSpeed -> PWM value for right motor; range: -MAX_MOTOR_POWER to +MAX_MOTOR_POWER
 *        Positive values = forward direction, negative values = reverse direction
 * Output: None; drives ENA/IN1/IN2 and ENB/IN3/IN4 pins on the H-Bridge motor driver
 * Logic: Applies RIGHT_MOTOR_BOOST to the right motor to compensate for mechanical
 *        asymmetry between the two motors. Constrains both values to MAX_MOTOR_POWER
 *        before writing. Sets direction pins and PWM pin based on sign of speed value.
 * Example Call: setMotors(150, -150);  // Spin in place clockwise
 */
void setMotors(int leftSpeed, int rightSpeed) {

  // Apply hardware compensation boost to right motor before clamping
  rightSpeed = (int)(rightSpeed * RIGHT_MOTOR_BOOST);

  // Clamp both motor speeds to safe PWM range
  leftSpeed  = constrain(leftSpeed,  -MAX_MOTOR_POWER, MAX_MOTOR_POWER);
  rightSpeed = constrain(rightSpeed, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

  // Set left motor direction and speed
  if (leftSpeed >= 0) {
    analogWrite(ENA, leftSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    analogWrite(ENA, -leftSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // Set right motor direction and speed
  if (rightSpeed >= 0) {
    analogWrite(ENB, rightSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    analogWrite(ENB, -rightSpeed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

/*
 * Function Name: processBluetooth
 * Input: None; reads from Serial (Bluetooth UART)
 * Output: None; updates motion flags, servo positions, buzzer state, and targetPosition
 * Logic: Reads one character from the serial buffer and flushes any remaining bytes
 *        to prevent stale command buildup. Each character maps to a robot action:
 *          'F' -> Set moveForward flag, clear moveBackward
 *          'B' -> Set moveBackward flag, clear moveForward
 *          'S' -> Stop all motion, lock targetPosition to current encoder average
 *          'L' -> Begin left turn by setting negative turnBias and decremented turnTargetDiff
 *          'R' -> Begin right turn by setting positive turnBias and incremented turnTargetDiff
 *          'U' -> Raise arm servo to ARM_UP angle
 *          'D' -> Lower arm servo to ARM_DOWN angle
 *          'O' -> Open claw servo to CLAW_OPEN angle
 *          'C' -> Close claw servo to CLAW_CLOSE angle
 *          'Z' -> Activate buzzer for BUZZER_DURATION ms at BUZZER_FREQUENCY Hz
 *          'T' -> Toggle turboMode on/off
 *        Returns immediately if no data is available in the serial buffer.
 * Example Call: processBluetooth();
 */
void processBluetooth() {

  if (!Serial.available()) return; // Exit early if no Bluetooth data in buffer

  char cmd = Serial.read();
  // Flush remaining bytes to avoid acting on stale queued commands
  while (Serial.available()) Serial.read();

  // Snapshot current encoder state for position and turn reference
  long avg  = (leftEncoderCount + rightEncoderCount) / 2;
  long diff = leftEncoderCount - rightEncoderCount;

  switch (cmd) {

    case 'F': moveForward = true; moveBackward = false; break;
    case 'B': moveBackward = true; moveForward = false; break;

    case 'S':
      // Stop motion and hold current encoder position
      moveForward = false;
      moveBackward = false;
      turning = false;
      turnBias = 0;
      targetPosition = avg;
      prevPositionError = 0;
      break;

    case 'L':
      turning = true;
      turnBias = -TURN_PWM;
      turnTargetDiff = diff - TURN_ENCODER_COUNTS;
      break;

    case 'R':
      turning = true;
      turnBias = TURN_PWM;
      turnTargetDiff = diff + TURN_ENCODER_COUNTS;
      break;

    case 'U': armPos = ARM_UP;    armServo.write(armPos);  break;
    case 'D': armPos = ARM_DOWN;  armServo.write(armPos);  break;
    case 'O': clawPos = CLAW_OPEN;  clawServo.write(clawPos); break;
    case 'C': clawPos = CLAW_CLOSE; clawServo.write(clawPos); break;

    case 'Z':
      // Activate buzzer; auto-shutoff is handled in loop() via millis()
      tone(BUZZER_PIN, BUZZER_FREQUENCY);
      buzzerActive = true;
      buzzerStartTime = millis();
      break;

    case 'T':
      turboMode = !turboMode;
      break;
  }
}

/*
 * Function Name: updateAngle
 * Input: None; reads from MPU6050 DMP FIFO buffer via I2C
 * Output: bool -> true if a new DMP packet was read and currentAngle updated,
 *                 false if FIFO was empty (caller should skip control update)
 * Logic: Fetches the latest DMP packet from the MPU6050 FIFO.
 *        Extracts quaternion, computes gravity vector, and derives roll angle
 *        (ypr[2]) as the tilt angle in degrees.
 *        Applies a complementary filter to fuse the DMP-integrated angle
 *        with the raw measurement to suppress drift and noise:
 *          currentAngle = 0.98*(currentAngle + delta) + 0.02*newAngle
 *        where delta = (newAngle - prevAngle) is the change since last reading.
 * Example Call: if (!updateAngle()) return;
 */
bool updateAngle() {

  // Return false if no new DMP packet is ready in FIFO
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Convert roll angle from radians to degrees
  float newAngle = ypr[2] * 180.0 / M_PI;

  // Complementary filter: high-pass on integrated gyro angle, low-pass on absolute accel angle
  currentAngle = 0.98 * (currentAngle + newAngle - prevAngle) + 0.02 * newAngle;
  prevAngle = newAngle;

  return true;
}

/*
 * Function Name: calculateBalance
 * Input: None; reads global encoder counts, currentAngle, motion flags, and sampleTime
 * Output: None; updates global leftMotorPower and rightMotorPower
 * Logic: Implements discrete-time LQR state-feedback control as follows:
 *        1. Ramp targetPosition by POSITION_RAMP_STEP each loop when moving
 *           (turboMode increases ramp step by 1.3x for faster response)
 *        2. Compute theta (tilt error) and thetaDot (numerical derivative)
 *        3. Detect external disturbances via sudden change in thetaDot exceeding
 *           DISTURBANCE_THRESHOLD; activate inRecoveryMode for RECOVERY_DURATION ms
 *        4. Compute positionError and positionErrorDot from encoder average
 *        5. Apply RECOVERY_BOOST to kTheta and kThetaD during disturbance recovery
 *        6. Compute LQR control signal:
 *           u = -(kTheta*theta) - (kThetaD*thetaDot)
 *               - (K_POSITION*positionError) - (K_POSITION_D*positionErrorDot)
 *        7. Add FRICTION_OFFSET to overcome static friction if |control| > FRICTION_DEADBAND
 *        8. Constrain control to MAX_MOTOR_POWER
 *        9. Check if turn encoder target is reached and clear turn flags if so
 *       10. Apply turnBias differentially to left and right motor power for steering
 * Example Call: calculateBalance();
 */
void calculateBalance() {

  // Increase ramp step in turbo mode for faster acceleration
  int rampStep = turboMode ? (POSITION_RAMP_STEP * 1.3) : POSITION_RAMP_STEP;

  // Increment or decrement target position to smoothly ramp robot speed
  if (moveForward  && !turning) targetPosition += rampStep;
  if (moveBackward && !turning) targetPosition -= rampStep;

  // Compute tilt state variables
  theta    = currentAngle - BALANCE_ANGLE;
  thetaDot = (theta - prevTheta) / sampleTime;

  // Detect sudden disturbance via angular acceleration spike
  float thetaDotChange = abs(thetaDot - prevThetaDot);
  if (thetaDotChange > DISTURBANCE_THRESHOLD && !inRecoveryMode) {
    inRecoveryMode    = true;
    recoveryStartTime = millis();
  }
  // Automatically exit recovery after RECOVERY_DURATION ms
  if (inRecoveryMode && (millis() - recoveryStartTime > RECOVERY_DURATION)) {
    inRecoveryMode = false;
  }

  prevThetaDot = thetaDot;
  prevTheta    = theta;

  // Compute position state variables from encoder average
  long currentPosition = (leftEncoderCount + rightEncoderCount) / 2;
  positionError    = targetPosition - currentPosition;
  positionErrorDot = (positionError - prevPositionError) / sampleTime;
  prevPositionError = positionError;

  // Clamp position error to prevent excessively aggressive correction
  positionError = constrain(positionError, -350, 350);

  // Apply adaptive gain boost during recovery from disturbance
  float kTheta  = K_THETA;
  float kThetaD = K_THETA_D;
  if (inRecoveryMode) {
    kTheta  *= RECOVERY_BOOST;
    kThetaD *= RECOVERY_BOOST;
  }

  // Compute LQR control signal from all four state variables
  float control =
    -(kTheta      * theta)           -
    (kThetaD      * thetaDot)        -
    (K_POSITION   * positionError)   -
    (K_POSITION_D * positionErrorDot);

  // Add static friction compensation only above deadband threshold
  if (abs(control) > FRICTION_DEADBAND) {
    if (control > 0) control += FRICTION_OFFSET;
    else             control -= FRICTION_OFFSET;
  }

  // Saturate control output to valid motor PWM range
  control = constrain(control, -MAX_MOTOR_POWER, MAX_MOTOR_POWER);

  // Check if turn is complete by comparing encoder differential to target
  if (turning) {
    long diff = leftEncoderCount - rightEncoderCount;
    if ((turnBias < 0 && diff <= turnTargetDiff) ||
        (turnBias > 0 && diff >= turnTargetDiff)) {
      turning  = false;
      turnBias = 0;
    }
  }

  // Apply differential bias for steering while maintaining balance
  leftMotorPower  = control - turnBias;
  rightMotorPower = control + turnBias;
}

/*
 * Function Name: setup
 * Input: None
 * Output: None
 * Logic: Initializes all hardware peripherals in the following sequence:
 *        1. Serial UART at BLUETOOTH_BAUD for Bluetooth communication
 *        2. I2C at 400kHz fast mode for MPU6050
 *        3. Motor driver GPIO pins as outputs
 *        4. Buzzer pin as output
 *        5. Encoder pins as INPUT_PULLUP
 *        6. External interrupt (INT0) for right encoder Channel A
 *        7. Pin Change Interrupt (PCINT23) for left encoder Channel A
 *        8. Servo motors attached and set to default 90 degree position
 *        9. 1 second delay for power rail stabilization
 *        10. MPU6050 initialized with custom clock, ranges, DLPF, and calibration offsets
 *        11. DMP firmware loaded; 3 second delay for stable quaternion output
 *        12. lastTime initialized for loop timing
 * Example Call: Called automatically once by the Arduino runtime at startup
 */
void setup() {

  Serial.begin(BLUETOOTH_BAUD);
  Wire.begin();
  Wire.setClock(400000); // I2C fast mode required for stable DMP data rate

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // Encoder pins use internal pull-ups for open-collector encoder compatibility
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  // External interrupt for right encoder (INT0 on pin 2)
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, CHANGE);

  // Pin Change Interrupt for left encoder Channel A (PCINT23 = digital pin 7)
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT23);

  armServo.attach(ARM_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);
  armServo.write(armPos);
  clawServo.write(clawPos);

  delay(1000); // Wait for power rails to stabilize before IMU initialization

  mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);       // Use gyro Z-axis PLL for stable clock source
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);    // ±1000 deg/s gyro range
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);     // ±4g accelerometer range
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);                // 42Hz digital low-pass filter on both sensors

  // Calibration offsets obtained from MPU6050 calibration routine
  mpu.setXAccelOffset(5257);
  mpu.setYAccelOffset(-830);
  mpu.setZAccelOffset(13522);
  mpu.setXGyroOffset(-56);
  mpu.setYGyroOffset(67);
  mpu.setZGyroOffset(9);

  // Load DMP firmware and enable it; check return value 0 for success
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    delay(3000); // Allow DMP to produce stable quaternion output before control starts
  }

  lastTime = millis();
}

/*
 * Function Name: loop
 * Input: None
 * Output: None
 * Logic: Main control loop executed continuously by the Arduino runtime.
 *        Performs the following in each iteration:
 *        1. Exits immediately if DMP is not ready
 *        2. Reads and processes any pending Bluetooth command
 *        3. Handles non-blocking buzzer auto-shutoff via millis()
 *        4. Fetches updated tilt angle from MPU6050 DMP; skips if no new packet
 *        5. Cuts motors and resets state if robot has fallen beyond FALL_ANGLE
 *        6. Measures actual sampleTime from elapsed milliseconds;
 *           clamps to 0.01s default if out of valid range (0, 0.05]
 *        7. On firstRun, initializes previous state variables to prevent
 *           derivative kick and returns without running the controller
 *        8. Calls calculateBalance() to compute LQR motor outputs
 *        9. Calls setMotors() to apply computed PWM values to drive motors
 * Example Call: Called automatically and continuously by the Arduino runtime
 */
void loop() {

  if (!dmpReady) return; // Halt all processing until DMP is initialized

  processBluetooth();

  // Non-blocking buzzer shutoff check
  if (buzzerActive && (millis() - buzzerStartTime >= BUZZER_DURATION)) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }

  if (!updateAngle()) return; // Skip this iteration if no new DMP data available

  // Safety cutoff: disable motors and reset state if robot has fallen
  if (abs(currentAngle) > FALL_ANGLE) {
    setMotors(0, 0);
    turning        = false;
    turnBias       = 0;
    firstRun       = true;
    inRecoveryMode = false;
    return;
  }

  // Measure actual loop period in seconds for accurate derivative computation
  unsigned long now = millis();
  sampleTime = (now - lastTime) / 1000.0;
  // Clamp sampleTime to prevent zero-division or derivative spike on timing anomalies
  if (sampleTime <= 0 || sampleTime > 0.05) sampleTime = 0.01;
  lastTime = now;

  // Initialize previous state on first valid loop to avoid derivative kick
  if (firstRun) {
    prevTheta         = currentAngle - BALANCE_ANGLE;
    prevThetaDot      = 0;
    prevPositionError = 0;
    firstRun          = false;
    return;
  }

  calculateBalance();
  setMotors(leftMotorPower, rightMotorPower);
}
