import processing.serial.*;

Serial port; 

int g_yaw = 0;
int g_pitch = 0;
int g_roll = 0;

int counter=0;

void setup() {
  println(Serial.list());
  String arduinoPort = Serial.list()[2];
  port = new Serial(this, arduinoPort, 9600);
}

void draw() {
  
}

void serialEvent(Serial p) {
  if (port.available() > 5) {
    if(port.read() == '\0'){
      g_yaw   = port.read();
      g_pitch = port.read();
      g_roll  = port.read();
      g_roll  = port.read();

      print("yaw : " );
      print(g_yaw);
      print("\t" );
      print("pitch : " );
      print(g_pitch);
      print("\t" );
      print("roll : " );
      print(g_roll);
      print("\t" );
      
      if(millis() < 8000 && millis() > 5000){
            counter++;
            
      }
      print("count : " + counter + "\t");
      print("time : " + millis() + "\t");
      print("\n");
      
    }
  }
}