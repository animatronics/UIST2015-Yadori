#include <QueueList.h>

QueueList <int> Photo_refrector_queue;

int PhotoRefrector_value      = 0;
int PhotoRefrector_prev_value = 0;
bool mouth_status = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // analogWrite(5,100);
  Serial.print(analogRead(A0));
  Serial.print("\t");

  PhotoRefrector_value = analogRead(A0);
  Photo_refrector_queue.push(PhotoRefrector_value);
  int value_delta = 0;

  if(Photo_refrector_queue.count > 5){
     value_delta = PhotoRefrector_value - Photo_refrector_queue.pop();
  }

  
  
  Serial.print(value_delta);
  Serial.print("\t");


  if(value_delta > 3){
    mouth_status = true;
  }
  if(value_delta < -3){
    mouth_status = false;
  }


  
  if(mouth_status == true){
    Serial.print("open");
  }
  else{
    Serial.print("close");
  }

  PhotoRefrector_prev_value = PhotoRefrector_value;

  Serial.print("\n");
} 