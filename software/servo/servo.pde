import processing.net.*;
import cc.arduino.*;
import org.firmata.*;
import processing.serial.*;

int g_yaw = 90;
int g_pitch = 90;
int g_roll = 90;
int mouth_status = 0;
int islive = 0;

Arduino arduino;
Server server;
Serial port; 

HeadClass head;
BodyClass body;
MouthClass mouth;
RightPanClass right_pan;
RightTiltClass right_tilt;
LeftPanClass left_pan;
LeftTiltClass left_tilt;

QueueClass body_queue, head_queue;

void setup() {
  println(Arduino.list());
  arduino = new Arduino(this, Arduino.list()[0], 57600);  //servo
  server = new Server(this, 50000);

  String arduinoPort = "COM7";      // mask
  port = new Serial(this, arduinoPort, 9600);

  jack_in();
}

void draw()
{  
  //right_pan.move(90);
  //left_pan.move(90);
  //right_tilt.move(45);
  //left_tilt.move(45);
  //head.servoWrite(90);
  //mouth.servoWrite(110);
  //body.servoWrite(90);
  //head.servoWrite(90);
  //delay(3000);
  //jack_out();
  //delay(5000);
  if (islive == 1) {
    Client c = server.available();        //getting datum from Kinect
    for (int i=0; i<4; i++) {
      if (c != null) {
        String s = c.readStringUntil('\n');
        if (s != null) {
          //println("sever received: " + s);
          String[] ss = splitTokens(s);
          String servo_name = ss[0];
          int kinect_value = int(ss[1]);
          //println(servo_name + " : Value: " + kinect_value);
          if ( servo_name.equals("right_pan") == true ) {
            right_pan.move(kinect_value);
          } else if  ( servo_name.equals("right_tilt") == true ) {
            right_tilt.move(kinect_value);
          } else if  ( servo_name.equals("left_pan") == true ) {
            left_pan.move(kinect_value);
          } else if  ( servo_name.equals("left_tilt") == true ) {
            left_tilt.move(kinect_value);
          }
        }
      }
    }
  
    body_queue.enq(g_yaw);
    body_queue.deq();
  
    head_queue.enq(g_pitch);
    head_queue.deq();
  
    body.servoWrite(body_queue.averageDeg());
    head.servoWrite(head_queue.averageDeg());
    
    if ( mouth_status == 1 ) {
      mouth.closeMouth();
    } else if ( mouth_status == 0 ){
      mouth.openMouth();
    }
    
    print(millis() + "\t" + g_yaw + "\n");
  } else if ( islive == 0 ) {
    jack_out();
  }
}

void exit() {
  jack_out();
  println("exit");
  super.exit();
}

void serialEvent(Serial mask) {
  if (mask.available() > 5) {
    if (mask.read() == '\0') {
      g_yaw   = mask.read();
      g_pitch = mask.read();
      g_roll  = mask.read();
      mouth_status = mask.read();
      islive = mask.read();
      //print("yaw : " );

      //print(g_yaw);
      //print("\t\t" );
      //print("pitch : " );
      //print(g_pitch);
      //print("\t\t" );
      //print("roll : " );
      //print(g_roll);
      //print("\n");
      //println(frameRate);
    }
  }
}

class ServoClass {
  int pin = 0;
  int max = 0;
  int min = 0;

  void init(int pinnum, int maxnum, int minnum) {
    pin = pinnum;
    max = maxnum;
    min = minnum;
  }

  int max() {
    return max;
  }

  int min() {
    return min;
  }

  int pin() {
    return pin;
  }

  void servoWrite(int angle) {
    angle = limitAngle(angle);
    arduino.pinMode(pin, Arduino.SERVO);
    arduino.servoWrite(pin, angle);
    //println(pin + " : " + angle);
  }

  int limitAngle(int angle) {
    int limitangle = angle;
    if ( angle > max ) {
      limitangle = max;
    } else if ( angle < min ) {
      limitangle = min;
    }
    return limitangle;
  }
}

class MouthClass extends ServoClass {
  void openMouth() {
    servoWrite(min);
  }

  void closeMouth() {
    servoWrite(max);
  }
}

class HeadClass extends ServoClass {
}

class BodyClass extends ServoClass {
}

class RightPanClass extends ServoClass {
  void move(int angle) {
    servoWrite(angle);
  }
}

class LeftPanClass extends ServoClass {
  void move(int angle) {
    servoWrite(180 - angle);
  }
}

class RightTiltClass extends ServoClass {
  void move(int angle) {
    servoWrite(angle);
  }
}

class LeftTiltClass extends ServoClass {
  void move(int angle) {
    servoWrite(angle);
  }
}


class QueueClass {
  int QUEUE_SIZE = 0;
  float[] queue;
  int head = 0;
  int tail = 0;

  void init(int size, float n) {
    QUEUE_SIZE = size;
    queue = new float[QUEUE_SIZE];
    tail = 0;
    head = QUEUE_SIZE-2;
    for (int i=0; i<QUEUE_SIZE; i++) {
      queue[i] = n;
    }
  }

  int enq(float n) {
    if (head % QUEUE_SIZE != (tail + 1) % QUEUE_SIZE) {
      queue[(tail)++ % QUEUE_SIZE] = n;
      return tail - head;
    } else {
      return -1;
    }
  }

  float deq() {
    if (head != tail) {
      return queue[(head)++ % QUEUE_SIZE];
    } else {
      return -1;
    }
  }

  int averageDeg() {
    float average=0;
    for (int i=0; i<QUEUE_SIZE; i++) {
      average += queue[i];
    }
    return (int)average/QUEUE_SIZE;
  }
}

void jack_out() {
  for (int i=5; i<12; i++) {
    arduino.pinMode(i, Arduino.OUTPUT);
    //arduino.digitalWrite(i, Arduino.LOW);
  }
}

void jack_in() {
  body_queue = new QueueClass();                  // Prepare queue for right degree
  body_queue.init(30, 90);                            // this queue will be full of 60 degree

  head_queue = new QueueClass();                  // Prepare queue for right degree
  head_queue.init(30, 90);                            // this queue will be full of 60 degree

  right_pan = new RightPanClass();
  right_pan.init(5, 170, 10);
  right_tilt = new RightTiltClass();
  right_tilt.init(6, 90, 20);
  left_pan = new LeftPanClass();
  left_pan.init(7, 170, 10);
  left_tilt = new LeftTiltClass();
  left_tilt.init(8, 90, 20);
  head = new HeadClass();
  head.init(9, 135, 45);
  mouth = new MouthClass();    //mouth based on 105(up)-80(down)
  mouth.init(10, 118, 80);
  body = new BodyClass();
  body.init(11, 160, 45);
}