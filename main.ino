
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}
int counter = 0;

void loop() {
  int value = analogRead(0);
/**  
  if(value > 1000) {
    counter = 0;
  }
  else {
    counter++;
    }
    **/
   Serial.println(value);
   Serial.println(value);
  //Serial.println(analogRead(0));
  // put your main code here, to run repeatedly:

}
