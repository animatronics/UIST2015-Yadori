# Yadori: Mask-type User Interface for Animatronics

concept movie is avalable in [here](https://www.youtube.com/watch?v=ffLGg4ycKXo)

UIST2015 Student Innovation Contest
Team Yadori

University of Tsukuba

- Keisuke Kawahara(@ktansai)
- Eimy Koike
- Mose Skashita
- Kenta Suzuki

## Introduction
 In Japan's theatrical arts, there is a cultural show called "Noh" where the actors cover their faces with special masks, and act to music to convey a story. Wearing a mask of certain character help noh-actors to play the role in the story as if they were the character.
 In puppetry, performers manipulate puppets using their hands above the desk while rest of the body is below the desk. This poses a challenge in coordination of actions mostly so if there are multiple actors.

## Our System
We propose a system for animatronics storytelling that enables performers to manipulate puppets by wearing a mask-type device on their faces. Mask-type device for animatronics is New User Interface that allows users to manipulate puppets more easily. It facilitates coordination of movements to appear more natural since the actors one relaxed and in direct eye contact with each other. Performers can wear a mask of any character on their face that they want to manipulate. It allows performers to be excited in manipulating puppets and get right into their own part.
Enjoy it!!

## Implementation
Our mask-type device captures voices and facial movements. Inside the mask-device, we have a microphone to record performer 's voices, photo reflector that detects the different states of opening and closing performer’s mouth and an IMU that captures the direction of performer's face.
As an additional function, we use Kinect depth-camera to capture movements of user's hands. Our system combines these body and facial movements to manipulate puppets toward supporting smooth synchronization between performers and puppets in puppet play.

## Requirement for using our system
+ A Kinect for Windows v2 Device
+ [Kinect SDK v2](http://www.microsoft.com/en-us/kinectforwindows/default.aspx)
+ [Processing 3.0](http://processing.org/)
+ Arduino Uno( [StandardFirmata](https://www.arduino.cc/en/Reference/Firmata) )
+ [Kinect for Windows v2 library for Processing](http://codigogenerativo.com/kinectpv2/)

