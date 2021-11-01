#include "Feeder.h"
//#include "frc/WPILib.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

Feeder::Feeder(int feeder): intake(feeder)
{
    intake.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 50);
    intake.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 50);

    intake.SetInverted(true);

}

void Feeder::Eject(double speed)
{
    intake.Set(ControlMode::PercentOutput, -speed);
}

bool Feeder::Feed(double speed)
{
    intake.Set(ControlMode::PercentOutput, speed);
}

void Feeder::SendData(std::string name)
{
    frc::SmartDashboard::PutNumber(name + " speed", intake.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber(name + " Temperature (C)", intake.GetTemperature());
}