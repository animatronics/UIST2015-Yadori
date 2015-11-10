//IMU 
 // #define DEBUG_MODE 


// photo refrector
#include <QueueList.h>

QueueList <int> Photo_refrector_queue;
int PhotoRefrector_value      = 0;
int PhotoRefrector_prev_value = 0;
byte mouth_status     = 0; // 0:open , 1 : close
byte enable_send_data = 0;




// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE 
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define CARIBRATE_BUTTON 7

#define CALIBRATION_OFFSET_TIME 0

#define PhotoRefrector_SIG A0


bool blinkState = false; 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte ypr_packet[3];


//calibration values
float offset_ypr[3] = {0,0,0};

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    pinMode(7, INPUT_PULLUP);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

        // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(200, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        #ifdef DEBUG_MODE
            Serial.println(F("Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef DEBUG_MODE
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        #endif
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // for calibration
    pinMode(CARIBRATE_BUTTON, INPUT_PULLUP);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // caibration 
    if (digitalRead(CARIBRATE_BUTTON) == LOW) {
        offset_ypr[0] = ypr[0];
        offset_ypr[1] = ypr[1];
        offset_ypr[2] = ypr[2];
        enable_send_data = !enable_send_data;
        delay(300);
    }

    //photo refrector
    PhotoRefrector_value = analogRead(PhotoRefrector_SIG);
    Photo_refrector_queue.push(PhotoRefrector_value);

    int value_delta = 0;

    if(Photo_refrector_queue.count() > 10){
        value_delta = PhotoRefrector_value - Photo_refrector_queue.pop();
        if(value_delta > 2 ) { mouth_status = 0; } 
        if(value_delta < -2) { mouth_status = 1; }
    }


    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        #ifdef DEBUG_MODE
            Serial.println(F("FIFO overflow!"));
        #endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // first calibration
        if(millis() < CALIBRATION_OFFSET_TIME){
            offset_ypr[0] = ypr[0];
            offset_ypr[1] = ypr[1];
            offset_ypr[2] = ypr[2];
        }

        if(millis() > CALIBRATION_OFFSET_TIME){

            ypr_packet[0] = formatConvert((ypr[0]-offset_ypr[0]) * 180/M_PI + 90);
            ypr_packet[1] = formatConvert((ypr[1]-offset_ypr[1]) * 180/M_PI + 90);
            ypr_packet[2] = formatConvert((ypr[2]-offset_ypr[2]) * 180/M_PI + 90);
            PhotoRefrector_value = analogRead(PhotoRefrector_SIG);

            #ifdef DEBUG_MODE
                Serial.print("raw ypr");
                Serial.print("\t");
                Serial.print((ypr[0]-offset_ypr[0]) * 180/M_PI + 90 );
                Serial.print("\t");
                Serial.print((ypr[1]-offset_ypr[1]) * 180/M_PI + 90 );
                Serial.print("\t");
                Serial.print((ypr[2]-offset_ypr[2]) * 180/M_PI + 90 );


                Serial.print("\t");

                Serial.print("converted ypr");
                
                Serial.print("\t");
                
                Serial.print(ypr_packet[0]);
                Serial.print("\t");
                Serial.print(ypr_packet[1]);
                Serial.print("\t");
                Serial.print(ypr_packet[2]);



                // Serial.print("\t PhotoRefrector \t");
                // Serial.print(PhotoRefrector_value);
                // Serial.print("\t");
                // Serial.print(value_delta);
                // Serial.print("\t");
                // Serial.print(mouth_status);   
                // Serial.print("\t");
                // Serial.print(enable_send_data);           


                Serial.print("\n");

            #else
                Serial.write('\0');
                Serial.write(ypr_packet[0]);
                Serial.write(ypr_packet[1]);
                Serial.write(ypr_packet[2]);
                Serial.write(mouth_status);
                Serial.write(enable_send_data);
            #endif
        }


        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        // delay(50);
    }
}

byte formatConvert(float gyro_value){
    if(gyro_value > 360) return formatConvert(gyro_value - 360) ;
    if(gyro_value > 170) return (byte)170;
    if(gyro_value < 10  ) return (byte)10;
    return (byte)((int)gyro_value);
}