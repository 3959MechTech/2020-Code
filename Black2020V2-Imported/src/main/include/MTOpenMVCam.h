/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/SerialPort.h>
#include <frc/Notifier.h>

#include <shared_mutex>

struct MTOpenMVData
{
  bool isNewData;
  bool targetFound;
  double x;
  double y;
  double Width;
  double Height;
  double Angle;
  double distance;
  double strength;
};

class MTOpenMVCam {

  enum MessageDef{
    kStartFlag = 0,
    kTargetFound = 1,
    kXCenterLower = 2,
    kXCenterUpper = 3,
    kYCenterLower = 4,
    kYCenterUpper = 5,
    kWidthLower = 6,
    kWidthUpper = 7,
    kHeightLower = 8,
    kHeightUpper = 9,
    kAngle = 10,
    kDistanceLower = 11,
    kDistanceUpper = 12,
    kStrengthLower = 13,
    kStregthUpper = 14
    
  };

  frc::SerialPort _port{115200 ,frc::SerialPort::kMXP};

  MTOpenMVData _latest = {false, false, 0,0,0,0,0,0,0};
  
  std::shared_timed_mutex _mutex;
  frc::Notifier *_thread;

  static const int MessageLength = 16;
  
  void read();
  

 public:
  MTOpenMVCam(double period);

  MTOpenMVData Get();

};
