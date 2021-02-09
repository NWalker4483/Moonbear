void Blink(int times){
  while(times<0){
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    times-=1; 
  }
}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int parseIntFast(int numberOfDigits){
  /*
  This function returns the converted integral number as an int value.
  If no valid conversion could be performed, it returns zero.*/
  char theNumberString[numberOfDigits + 1];
  int theNumber;
  for (int i = 0; i < numberOfDigits; theNumberString[i++] = Serial.read()){
    delay(5);
    };
  theNumberString[numberOfDigits] = 0x00;
  theNumber = atoi(theNumberString);
  return theNumber;
}