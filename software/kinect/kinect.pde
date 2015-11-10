import processing.net.*;
Client client;

import KinectPV2.KJoint;
import KinectPV2.*;

KinectPV2 kinect;

QueueClass rightPan_queue;
QueueClass leftPan_queue;
QueueClass rightTilt_queue;
QueueClass leftTilt_queue;

float zVal = 300;
float rotX = PI;

void setup() {
  size(1024, 768, P3D);

  kinect = new KinectPV2(this);

  kinect.enableColorImg(true);

  //enable 3d  with (x,y,z) position
  kinect.enableSkeleton3DMap(true);

  rightPan_queue = new QueueClass();                  // Prepare queue for right degree
  rightPan_queue.init(15, 60);                            // this queue will be full of 60 degree
  
  leftPan_queue = new QueueClass();                  // Prepare queue for left degree
  leftPan_queue.init(15, 60);                            // this queue will be full of 60 degree 
  
  rightTilt_queue = new QueueClass();                  // Prepare queue for right degree
  rightTilt_queue.init(15, 60);                            // this queue will be full of 60 degree

  leftTilt_queue = new QueueClass();                  // Prepare queue for right degree
  leftTilt_queue.init(15, 60);                            // this queue will be full of 60 degree

  kinect.init();
  
  client = new Client(this, "localhost" , 50000);
  frameRate(30);
}

void draw() {
  background(0);

  image(kinect.getColorImage(), 0, 0, 320, 240);

  //translate the scene to the center 
  pushMatrix();
  translate(width/2, height/2, 0);
  scale(zVal);
  rotateX(rotX);

  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();



  //individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked() && i == 0) {
      KJoint[] joints = skeleton.getJoints();
        float s_x, s_y, s_z;
        float sb_x, sb_y, sb_z;
        float sr_x, sr_y, sr_z;
        float sl_x, sl_y, sl_z;
        float r_x, r_y, r_z;
        float l_x, l_y, l_z;
      //draw different color for each hand state
      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);

      sb_x = joints[KinectPV2.JointType_SpineBase].getX();
      sb_y = joints[KinectPV2.JointType_SpineBase].getY();
      sb_z = joints[KinectPV2.JointType_SpineBase].getZ();
      sr_x = joints[KinectPV2.JointType_ShoulderRight].getX();
      sr_y = joints[KinectPV2.JointType_ShoulderRight].getY();
      sr_z = joints[KinectPV2.JointType_ShoulderRight].getZ();
      sl_x = joints[KinectPV2.JointType_ShoulderLeft].getX();
      sl_y = joints[KinectPV2.JointType_ShoulderLeft].getY();
      sl_z = joints[KinectPV2.JointType_ShoulderLeft].getZ();
      s_x = joints[KinectPV2.JointType_SpineShoulder].getX();
      s_y = joints[KinectPV2.JointType_SpineShoulder].getY();
      s_z = joints[KinectPV2.JointType_SpineShoulder].getZ();
      r_x = joints[KinectPV2.JointType_HandRight].getX();
      r_y = joints[KinectPV2.JointType_HandRight].getY();
      r_z = joints[KinectPV2.JointType_HandRight].getZ();
      l_x = joints[KinectPV2.JointType_HandLeft].getX();
      l_y = joints[KinectPV2.JointType_HandLeft].getY();
      l_z = joints[KinectPV2.JointType_HandLeft].getZ();

      print("SpineBase: " + sb_x+ " / " + sb_y + " / " + sb_z + "\n");
      print("SpineShoulder: " + s_x+ " / " + s_y + " / " + s_z + "\n");
      print("ShoulderRight: " + r_x+ " / " + r_y + " / " + r_z + "\n");
      print("HandRight: " + sr_x+ " / " + sr_y + " / " + sr_z + "\n");
      print("HandLeft: " + l_x+ " / " + l_y + " / " + l_z + "\n");
      print("ShoulderRight: " + sl_x+ " / " + sl_y + " / " + sl_z + "\n");

      rightPan_queue.enq(returnDegree(r_x, r_y, r_z, s_x , s_y , s_z , sb_x , sb_y , sb_z));            //put right_degree into queue
      rightPan_queue.deq();

      rightTilt_queue.enq(-90 + returnDegree(r_x, r_y, r_z, sr_x , sr_y , sr_z , s_x , s_y , s_z));                    //put right_degree into queue
      rightTilt_queue.deq();

      leftPan_queue.enq(returnDegree(l_x, l_y, l_z, s_x , s_y , s_z , sb_x , sb_y , sb_z));                  //put left_degree into queue
      leftPan_queue.deq();

      leftTilt_queue.enq(-90 + returnDegree(l_x, l_y, l_z, sl_x , sl_y , sl_z , s_x , s_y , s_z));               //put left_degree into queue
      leftTilt_queue.deq();

      print("R_Pan_Average = " + rightPan_queue.averageDeg() + " / L_Pan_Average = " + leftPan_queue.averageDeg() + "\n");
      print("R_Tilt_Average" + rightTilt_queue.averageDeg() + " / L_Tilt_Average = " + leftTilt_queue.averageDeg()+ "\n");
      print("\n");
      
      client.write("right_pan " + rightPan_queue.averageDeg() + "\n");  //send to sever
      client.write("right_tilt " + rightTilt_queue.averageDeg() + "\n");
      client.write("left_pan " + leftPan_queue.averageDeg() + "\n");
      client.write("left_tilt " + leftTilt_queue.averageDeg() + "\n");

      //Draw body
      color col  = skeleton.getIndexColor();
      stroke(col);
      drawBody(joints);
    }
  }
  popMatrix();


  fill(255, 0, 0);
  text(frameRate, 50, 50);
}

void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);

  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm    
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);

  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

void drawJoint(KJoint[] joints, int jointType) {
  strokeWeight(2.0f + joints[jointType].getZ()*8);
  point(joints[jointType].getX(), joints[jointType].getY(), joints[jointType].getZ());
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  strokeWeight(2.0f + joints[jointType1].getZ()*8);
  point(joints[jointType2].getX(), joints[jointType2].getY(), joints[jointType2].getZ());
}

void drawHandState(KJoint joint) {
  handState(joint.getState());
  strokeWeight(5.0f + joint.getZ()*8);
  point(joint.getX(), joint.getY(), joint.getZ());
}

void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    stroke(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    stroke(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    stroke(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    stroke(100, 100, 100);
    break;
  }
}

float returnPanDegree(float X1, float Y1, float X2, float Y2 ) {
  float degree = (float)Math.toDegrees(Math.atan((Y2-Y1)/(X2-X1)));
  return 90 + degree;
}

float returnTiltDegree(float X1, float Z1, float X2, float Z2 ) {
  float degree = (float)Math.toDegrees(Math.atan((Z2-Z1)/(X2-X1)));
  return 90 - degree;
}

float returnDegree(float X1, float X2, float X3, float Y1, float Y2, float Y3,float Z1, float Z2, float Z3 ) {
  float[] yx = {X1 - Y1, X2 - Y2, X3 - Y3};
  float[] yz = {Z1 - Y1, Z2 - Y2, Z3 - Y3};

  float lyx = (float)Math.sqrt(yx[0]*yx[0]+yx[1]*yx[1]+yx[2]*yx[2]);
  float lyz = (float)Math.sqrt(yz[0]*yz[0]+yz[1]*yz[1]+yz[2]*yz[2]);

  float innerProduct = yx[0]*yz[0] + yx[1]*yz[1] + yx[2]*yz[2];

  float degree = (float)Math.toDegrees(Math.acos(innerProduct/(lyx*lyz)));
  return degree;
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
      for(int i=0; i<QUEUE_SIZE; i++) {
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
      for(int i=0; i<QUEUE_SIZE; i++) {
        average += queue[i];
      }
      return (int)average/QUEUE_SIZE;
    }
    
}