/*
-------------------------------
  INITIALIZE CONSTANTS (PINS)
-------------------------------
*/

// Use constants to store the values for ultrasonic sensor pins
const int MIC_PIN_1 = A0; // Analog pin A0
const int MIC_PIN_2 = A1; // Analog pin A1

// Use constants to store the values for ultrasonic sensor pins
const int TRIG_PIN = 9; // Digital pin 9
const int ECHO_PIN = 10; // Digital pin 10

/*
-------------------------------
  SETUP
-------------------------------
*/

void setup() {
  // for the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT); // initialize trigger pin of ultrasonic sensor as output
  pinMode(ECHO_PIN, INPUT); // initialize echo pin of ultrasonic sensor as input

  // Set the baud rate to 9600 and begin serial communication
  Serial.begin(9600);
}

/*
-------------------------------
  MICROPHONES
-------------------------------
*/

void MicrophoneFunction(){

  // use constant to define threshold amplitude (Used to filter background noise)
  const int AMPLITUDE_THRESHOLD = 0;
  
  // Microphone 1
  int min_vol_mic1 = 1024; // initialize minimum value to be the maximum possible value
  int max_vol_mic1 = 0; // initialize maximum value to be the minimum possible value

  // Microphone 2
  int min_vol_mic2 = 1024; // initialize minimum value to be the maximum possible value
  int max_vol_mic2 = 0; // initialize maximum value to be the minimum possible value

  // Takes 5000 readings fo volume and saves the max & min of those values
  for (int i = 0; i < 5000; ++i) {

    int vol_reading_mic1 = analogRead(MIC_PIN_1); // Get reading from microphone 1
    int vol_reading_mic2 = analogRead(MIC_PIN_2); // Get reading from microphone 2

    // Comparing the new value read from microphone 1 with the currently stored maximum & minimum
    min_vol_mic1 = min(min_vol_mic1, vol_reading_mic1); 
    max_vol_mic1 = max(max_vol_mic1, vol_reading_mic1); 

    // Comparing the new value read from microphone 2 with the currently stored maximum & minimum
    min_vol_mic2 = min(min_vol_mic2, vol_reading_mic2); 
    max_vol_mic2 = max(max_vol_mic2, vol_reading_mic2); 

  }

  // Amplitude is defined as the difference between the highest and lowest points of a wave
  int amplitude_mic1 = max_vol_mic1 - min_vol_mic1;
  int amplitude_mic2 = max_vol_mic2 - min_vol_mic2;
  
  int amplitude_difference = abs(amplitude_mic1 - amplitude_mic2); // used to ensure sound is within threshold

  // Output mic results
  Serial.print("--------------------------------------------\n");
  Serial.print("Difference between Amplitudes ="); // For debugging purposes
  Serial.println(amplitude_difference);

  if (amplitude_difference < AMPLITUDE_THRESHOLD){
    Serial.println("sound is coming from: CENTER");
  }
  else if(amplitude_difference > AMPLITUDE_THRESHOLD && amplitude_mic1 > amplitude_mic2){
    Serial.println("sound is coming from: RIGHT");
  }
  else{
    Serial.println("sound is coming from: LEFT");
  }

}

/*
-------------------------------
  ULTRASONIC SENSOR
-------------------------------
*/

void UltrasonicFunction(){
  long duration;
  float distance;
  
  // Set trigger pin LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10µs HIGH pulse to the trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH); // Measure time taken for the echo to return

  distance = (duration * 0.0343) / 2; // Calculate the distance (speed of sound = 343 m/s or 0.0343 cm/µs)

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(100); // Wait before the next measurement
}

/*
-------------------------------
  MAIN LOOP
-------------------------------
*/

void loop() {
  MicrophoneFunction();
  UltrasonicFunction();
}


