#pragma once

#include <frc/WPILib.h>
#include <ctre/phoenix.h>
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <Rev/CANSparkMax.h>
#include "rev/ControlType.h"

using namespace frc;
class Feeder
{
  private:
    WPI_TalonFX intake;

  public:
    Feeder(int feeder);
    bool Feed (double speed);
    void Eject(double speed);
  
    void SendData(std::string name= "Feeder");
};