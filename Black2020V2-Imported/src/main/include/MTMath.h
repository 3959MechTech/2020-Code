/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <math.h>

class MTMath {
 public:
  MTMath();

  static inline double CleanAngle(double radians)
	{
		return atan2(sin(radians),cos(radians));
	}
  
  static inline double DegreesToRadians(double angle)
	{
		return M_PI*angle/180.0;
	}

  static inline double RadiansToDegrees(double angle)
	{
		return (angle*180.0)/M_PI;
	}

  static inline double TicksPerInch(double wheelDiameter, double gearRatio, double ticksPerRev)
  {
    return  (ticksPerRev*gearRatio)/(wheelDiameter*M_PI);
  }
  static inline double InchPerTicks(double wheelDiameter, double gearRatio, double ticksPerRev)
  {
    return  (wheelDiameter*M_PI)/(ticksPerRev*gearRatio);
  }

};
