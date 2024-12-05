/*
-------------------------------
  INITIALIZE CONSTANTS (PINS)
-------------------------------
*/

// Use constants to store the values for ultrasonic sensor pins
const int MIC_PIN_1 = A0;  // Analog pin A0
const int MIC_PIN_2 = A1;  // Analog pin A1

// Use constants to store the values for ultrasonic sensor pins
const int TRIG_PIN = 5;  // Digital pin 5
const int ECHO_PIN = 4;  // Digital pin 4

// Constants relating to the H-Bridge (i.e. L298N)
const int IN1 = 8;        // Motor 1
const int IN2 = 9;        // Motor 1
const int IN3 = 12;       // Motor 2
const int IN4 = 13;       // Motor 2
const int ENGINE_A = 11;  // To control motor 1 speed
const int ENGINE_B = 10;  // To control motor 2 speed

/*
-------------------------------
  ADJUSTABLE CONSTANTS
-------------------------------
*/

const int distance_threshold = 10; // Measured in centimeters. If the robot is less than this threshold, it won't move
const int forward_duration = 500; // Amount of milliseconds that the robot moves forward (higher value = more distance forward)
const int turn_duration = 400; // Amount of milliseconds that the robot turns (higher value = bigger angle)
const int right_mic_coeff = 4; // To fix the sensitivity of the right microphone (higher value = more sensitive)
const int motor_speed = 128; // How fast the motors spin (higher value = faster)

/*
-------------------------------
  SETUP
-------------------------------
*/

void setup() {
  // for the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);  // initialize trigger pin of ultrasonic sensor as output
  pinMode(ECHO_PIN, INPUT);   // initialize echo pin of ultrasonic sensor as input

  analogWrite(ENGINE_A, motor_speed);
  analogWrite(ENGINE_B, motor_speed);

  // Set the baud rate to 9600 and begin serial communication
  Serial.begin(9600);
}

/*
-------------------------------
  MICROPHONES
-------------------------------
*/

int MicrophoneFunction() {

  // Center = 0
  // Left = 1
  // Right = 2
  int direction = 0;

  // use constant to define threshold amplitude (Used to filter background noise)
  const int AMPLITUDE_THRESHOLD = right_mic_coeff;

  // Microphone 1
  int min_vol_mic1 = 1024;  // initialize minimum value to be the maximum possible value
  int max_vol_mic1 = 0;     // initialize maximum value to be the minimum possible value

  // Microphone 2
  int min_vol_mic2 = 1024;  // initialize minimum value to be the maximum possible value
  int max_vol_mic2 = 0;     // initialize maximum value to be the minimum possible value

  // Takes 5000 readings fo volume and saves the max & min of those values
  for (int i = 0; i < 5000; ++i) {

    int vol_reading_mic1 = analogRead(MIC_PIN_1);  // Get reading from microphone 1
    int vol_reading_mic2 = analogRead(MIC_PIN_2);  // Get reading from microphone 2

    // Comparing the new value read from microphone 1 with the currently stored maximum & minimum
    min_vol_mic1 = min(min_vol_mic1, vol_reading_mic1);
    max_vol_mic1 = max(max_vol_mic1, vol_reading_mic1);

    // Comparing the new value read from microphone 2 with the currently stored maximum & minimum
    min_vol_mic2 = min(min_vol_mic2, vol_reading_mic2);
    max_vol_mic2 = max(max_vol_mic2, vol_reading_mic2);

    // -----Debugging------

    // Serial.print("Left:");
    // Serial.print(vol_reading_mic1);
    // Serial.print("\tRight:");
    // Serial.println(vol_reading_mic2);
    // delay(200);

    // --------End---------
  }

  // Amplitude is defined as the difference between the highest and lowest points of a wave
  int amplitude_mic1 = max_vol_mic1 - min_vol_mic1 - 1;
  int amplitude_mic2 = right_mic_coeff*(max_vol_mic2 - min_vol_mic2 - 1);

  int amplitude_difference = abs(amplitude_mic1 - amplitude_mic2);  // used to ensure sound is within threshold

  // -----Debugging------

  Serial.print("--------------------------------------------\n");
  Serial.print("Amp1 = ");
  Serial.println(amplitude_mic1);
  Serial.print("Amp2 = ");
  Serial.println(amplitude_mic2);
  // Serial.print("Difference between Amplitudes =");  // For debugging purposes
  // Serial.println(amplitude_difference);

  // --------End---------

  if (amplitude_mic1 <= AMPLITUDE_THRESHOLD && amplitude_mic2 <= AMPLITUDE_THRESHOLD) 
    return -1;
  else if (amplitude_difference <= AMPLITUDE_THRESHOLD)
    return 0;
  else if (amplitude_difference > AMPLITUDE_THRESHOLD && amplitude_mic1 > amplitude_mic2)
    return 1;
  else
    return 2;
}

/*
-------------------------------
  ULTRASONIC SENSOR
-------------------------------
*/

float UltrasonicFunction() {
  long duration;
  float distance;

  // Set trigger pin LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10µs HIGH pulse to the trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);  // Measure time taken for the echo to return

  distance = (duration * 0.0343) / 2;  // Calculate the distance (speed of sound = 343 m/s or 0.0343 cm/µs)

  //Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(100);  // Wait before the next measurement

  return distance;
}

/*
-------------------------------
  MOTOR TURNING
-------------------------------
*/

void MotorStop() {
  Serial.println("Stopping...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void MotorForward() {
  Serial.println("Moving FORWARD...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void MotorRight() {
  Serial.println("Turning RIGHT...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void MotorLeft() {
  Serial.println("Turning LEFT...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/*
-------------------------------
  MOTOR CONTROL
-------------------------------
*/
void MotorFunction(float distance, int direction) {
  if (distance <= distance_threshold) {
    Serial.println("Too close! (distance <= 10cm)");
    MotorStop();
    return;
  }

  if (direction == -1) {
    MotorStop();

  } else if (direction == 0) {
    MotorForward();
    delay(forward_duration);
    MotorStop();

  } else if (direction == 1) {
    MotorLeft();
    delay(turn_duration);
    MotorForward();
    delay(forward_duration);
    MotorStop();

  } else if (direction == 2) {
    MotorRight();
    delay(turn_duration);
    MotorForward();
    delay(forward_duration);
    MotorStop();

  }
}

/*
-------------------------------
  MAIN LOOP
-------------------------------
*/

void loop() {
  int direction = MicrophoneFunction();
  float distance = UltrasonicFunction();
  MotorFunction(distance, direction);
}
