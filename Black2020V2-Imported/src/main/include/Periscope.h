#pragma once


#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>


class Periscope
{
     public:
     Periscope();
     ~Periscope(){};
     
        enum PPos
        {
            Home = 0,
            Low = 1,
            Level = 2,
            High = 3
        };
private: 
    static const int MaxPos = 4;
    static const int kTimeOut = 6;

    WPI_TalonSRX _left;
    WPI_TalonSRX _right;

    double posVals[MaxPos][3]; //0 = position, 1 = ramp, 2 = max speed

    double maxRamp;

    PPos current;
   public:

   Periscope(int left, int right); 
   double GetHeight(PPos);
   double GetRamp(PPos);
   void SetMaxRamp(double ramp);
  
   PPos GetPPos(){return current;};
   double GetError();
  
   double GetMaxSpeed(PPos);
   double GetSetPoint();
   void SetPPos(PPos);
   double GetEncoderPos();

   void ConfigureLeft(TalonSRXConfiguration left);
   void ConfigureRight(TalonSRXConfiguration right);

   void SetMotorSpeed(double speed);
   void SetPosition(double pos);
};


