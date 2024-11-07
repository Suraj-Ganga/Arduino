const int microphonePinA = A0;
const int microphonePinB = A1;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int mn1 = 1024;
  int mx1 = 0;

  int mn2 = 1024;
  int mx2 = 0;

  //10000 inputs and saves max/min of those voltages
  for (int i = 0; i < 10000; ++i) {

    int val1 = analogRead(microphonePinA);
    int val2 = analogRead(microphonePinB);

    mn1 = min(mn1, val1);
    mx1 = max(mx1, val1);

    mn2 = min(mn2, val2);
    mx2 = max(mx2, val2);
  }

  int delta1 = mx1 - mn1;
  int delta2 = mx2 - mn2;
  int diff = delta1-delta2;

  Serial.print("-------------------------\n");

  Serial.print("diff=");
  Serial.println(diff);

  if (diff == 0){
    Serial.println("CENTER");
  }
  else if(diff > 0){
    Serial.println("RIGHT");
  }
  else{
    Serial.println("LEFT");
  }

}
