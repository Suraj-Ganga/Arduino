int in1 = 13;   // variable names for the L298N
int in2 = 12;   // variable names for the L298N
int in3 = 8;   // variable names for the L298N
int in4 = 7;  // variable names for the L298N
int enA = 3;
int enB = 11;

void setup() {
  // all L298N digital pins are outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  analogWrite(enA, 128);
  analogWrite(enB, 128);
}

void loop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}