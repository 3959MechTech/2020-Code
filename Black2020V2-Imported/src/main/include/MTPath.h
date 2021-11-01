/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>

class MTPath {
  double _ticksPerInch;

 public:
  MTPath(double ticksPerInch){ _ticksPerInch = ticksPerInch;};
  MTPath(double wheelDiameter, double ticksPerMotorRev, double gearRatio=1.0);
  ~MTPath(){};
  
  enum column{position_left, velocity_left, position_right, velocity_right, duration};
  
  void GetStreamFromArray(BufferedTrajectoryPointStream& stream, const double traj[][5], int trajLen, bool left, bool forward);
  
  
};
