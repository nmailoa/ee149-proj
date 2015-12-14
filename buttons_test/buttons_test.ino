void setup() {
  // put your setup code here, to run once:
  pinMode(3, INPUT);
  pinMode(1, INPUT);
  Serial.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  int r = 0;
  int g = 0;
  int b = 0;
  int d = digitalRead(1);
  
  if(digitalRead(3)) r = 255;
  if(d) g = 255;
  
  Bean.setLed(r, g, b);
  Serial.println(d);
  Bean.sleep(100);
}
