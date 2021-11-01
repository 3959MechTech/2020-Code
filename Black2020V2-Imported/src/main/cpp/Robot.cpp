/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <unistd.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "TrenchRun_TrenchRun1.h"
#include "TrenchRun_TrenchRun2.h"
#include "Power10_Powerpath1.h"
#include "Power10_Powerpath2.h"
#include "Power10_Powerpath3.h"
#include "Power10_Powerpath4.h"
#include "Power10_Powerpath5.h"
#include "Power10_Powerpath6.h"
#include "Power10_Powerpath7.h"


/*
INDEXER
Front Index (Device ID 30)
Mid Index (Device ID 31)
Back Index (Device ID 32)

FEEDER 
Intake (Device ID 60)

PERISCOPE
Left Climber (Device ID 50)
Right Climber (Device ID 51)

PIGEON IMU
Base Pigeon (Device ID 1)
Shooter Pigeon (Device ID 2)

SHOOTER
Left Shooter (Device ID 20)
Right Shooter (Device ID 21)
Shooter Turret (Device ID 22)

DRIVETRAIN
LeftMaster (Device ID 11)
LeftSlave (Device ID 13)
RightMaster (Device ID 10)
RightSlave (Device ID 12)

OTHER DEVICES
PCM (Device ID 5)
PDP (Device ID 0)

currently no probe ID
*/

void Robot::RobotInit() 
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //Initializes a smart timer to begin
    smTimer.Start();

    drive.ConfigRobot(6.0,27.0,33.0,2048.0);

    _telemetryThread = new std::thread(&Robot::sendData, this);
    
    _PoseThread = new frc::Notifier(&Robot::updatePose, this);
    _PoseThread->StartPeriodic(.010);
    _PoseTimer.Start();
    


    cam.EnableTermination();
    cam.SetReadBufferSize(16);
    //cam.SetTimeout(1.0);
    //cam1.DisableTermination();
    //cam1.SetReadBufferSize(5);
    //cam1.SetTimeout(1.0);
    //cam2.DisableTermination();
    //cam2.SetReadBufferSize(5);
    //cam2.SetTimeout(1.0);

    NetTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

}

void Robot::updatePose()
{
    drive.UpdatePose();
    _poseThreadMutex.lock();
    _poseThreadPeriod = _PoseTimer.Get();
    _poseThreadMutex.unlock();
    _PoseTimer.Reset();
}

void Robot::sendData()
{
    usleep(100000);//give system time to finish starting up.
    _telemetryTimer.Start();
    while (!_stopTelemetryThread)
    {
        frc::SmartDashboard::PutNumber("state", state);
        //char data[128];
        //int recved = cam.Read(data, 127);
        //data[recved] = '\0';
        //frc::SmartDashboard::PutNumber("cam Data len", recved);
        //frc::SmartDashboard::PutString("cam Data", std::string(data));
        int rxLen = cam.GetBytesReceived();
        frc::SmartDashboard::PutNumber("cam Data len", rxLen);

        if(rxLen>12)
        {
            char* buf = new char[14];
            cam.Read(buf, 14);
            frc::SmartDashboard::PutString("UART", buf);
        }
/*
        recved = cam1.Read(data, 127);
        data[recved] = '\0';
        frc::SmartDashboard::PutNumber("cam1 Data len", recved);
        frc::SmartDashboard::PutString("cam1 Data", std::string(data));

        recved = cam2.Read(data, 127);
        data[recved] = '\0';
        frc::SmartDashboard::PutNumber("cam2 Data len", recved);
        frc::SmartDashboard::PutString("cam2 Data", std::string(data));
*/
        double x = NetTable->GetNumber("tx", 0.0);
        frc::SmartDashboard::PutNumber("limelight X", x);


        double posePeriod;
        _poseThreadMutex.lock();
        posePeriod = _poseThreadPeriod;
        _poseThreadMutex.unlock();
        frc::SmartDashboard::PutNumber("Pose Thread Period", posePeriod);

        feeder.SendData();
        drive.SendData();
        frc::SmartDashboard::PutNumber("Telemetery Thread Period", _telemetryTimer.Get());
        _telemetryTimer.Reset();       
        usleep(10000);//sleep for 20ms aka 20,000 us
    }
    
}

void Robot::devStick()
{
    if(stick3.GetXButton())
    {
        indexer.DirectDrive(-stick3.GetY(frc::GenericHID::kLeftHand)/2.0,
                            -stick3.GetY(frc::GenericHID::kLeftHand)/2.0,
                            -stick3.GetY(frc::GenericHID::kLeftHand));
    }

    if(stick3.GetAButton())
    {
        indexer.SetM1(-stick3.GetY(frc::GenericHID::kLeftHand));
    }

    if(stick3.GetBButton())
    {
        indexer.SetM2(-stick3.GetY(frc::GenericHID::kLeftHand));
    }

    if(stick3.GetYButton())
    {
        indexer.SetM3(-stick3.GetY(frc::GenericHID::kLeftHand));
    }

    if(stick3.GetTriggerAxis(frc::GenericHID::kRightHand)>.1)
    {
        shooter.setWheelSpeed(stick3.GetTriggerAxis(frc::GenericHID::kRightHand)/2.0);
    }else
    {  
        shooter.setWheelSpeed(0);
    }
    
    
    if(stick3.GetTriggerAxis(frc::GenericHID::kLeftHand)>.1)
    {
        feeder.Feed(stick3.GetTriggerAxis(frc::GenericHID::kLeftHand)*.75);
    }else
    {
        feeder.Feed(0);
    }
    
   
    
    periscope.SetMotorSpeed(-stick3.GetY(frc::GenericHID::kRightHand));
}

void Robot::executeTasks()
{
    stick.GetY(frc::GenericHID::kLeftHand);
    stick.GetY(frc::GenericHID::kRightHand);

    double v = -stick.GetY(frc::GenericHID::kLeftHand);
    double w = stick.GetX(frc::GenericHID::kRightHand);

    //rm.Set( (2.0*v-w)/2.0 );
    //lm.Set( (2.0*v+w)/2.0 );
    //rs.Set( (2.0*v-w)/2.0 );
    //ls.Set( (2.0*v+w)/2.0 );

    double lY = -stick.GetY(frc::GenericHID::kLeftHand);
    double rY = -stick.GetY(frc::GenericHID::kRightHand);

    if(lY>-.2 && lY<.2)
    {
        lY = 0.0;
    }
    if(rY>-.2 && rY<.2)
    {
        rY = 0.0;
    }
    double lX = -stick.GetX(frc::GenericHID::kLeftHand);
    double rX = -stick.GetX(frc::GenericHID::kRightHand);

    if(lX>-.2 && lX<.2)
    {
        lX = 0.0;
    }
    if(rX>-.2 && rX<.2)
    {
        rX = 0.0;
    }

    drive.ArcadeDrive(ControlMode::Velocity, lY*18000, rX*18000);//21000 highest

    if(stick.GetStartButtonPressed())
    {
        //Shift Gears
    }

    if(stick2.GetAButtonPressed())
    {
        //Overhead Pneumatic release
    }

    if(stick2.GetXButtonPressed())
    {
        //Set Value for shooter
    }

    if(stick2.GetTriggerAxis(frc::GenericHID::kRightHand)>.1)
    {
        feeder.Feed(stick2.GetTriggerAxis(frc::GenericHID::kRightHand));
    }else{
        feeder.Feed(0.0);
    }

    if(stick2.GetTriggerAxis(frc::GenericHID::kLeftHand))
    {
        //Indexer
    }

    if(stick2.GetBumperPressed(frc::GenericHID::kRightHand))
    {
        //Control Panel turn 5 times
    }

    if(stick2.GetBumperPressed(frc::GenericHID::kLeftHand))
    {
        //Control Panel moves to specific color 
    }

    if(stick2.GetBackButtonPressed())
    {
        //Control Panel Override
    }
}

void Robot::RobotPeriodic() {
  if(smTimer.Get()>.02)
  {
    smTimer.Reset();
    //drive.SendData();
  }

}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();

  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
  } else {
  }
}

void Robot::TeleopInit() {
  drive.ResetEncoders();
  state  = 0;
}

void Robot::TeleopPeriodic() {
  
    devStick();
    executeTasks();
    
    //frc::SmartDashboard::PutNumber("rv", rv);

    //A button = serpintine
    //B button = right turn
    //X button = left turn
    //Y button = straight
/*
    if(stick.GetAButton())
    {
        if(stick.GetAButtonPressed())
        {
            timer.Reset();//reset timer
            //zero the encoders and state machine variable
            state = 0;
            drive.ResetEncoders();

            //load trajectory into buffer
            tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, true, true);
            tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun1Points, TrenchRun_TrenchRun1Len, false, true);
        }
        switch (state)
        {
        case 0: //start traj
            drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
            state++;
            break;
        case 1: //wait for traj to finish
            if(drive.IsMotionProfileFinished()){state++;}
            break;
        case 2: //prepare to reverse traj
            tragTool.GetStreamFromArray(left_bufferedStream, TrenchRun_TrenchRun2Points, TrenchRun_TrenchRun2Len, true, false);
            tragTool.GetStreamFromArray(right_bufferedStream, TrenchRun_TrenchRun2Points, TrenchRun_TrenchRun2Len, false, false);
            timer.Reset();
            timer.Start();
            state++;
            break;
        case 3: 
            if(timer.Get()>2.0){state++;}//wait 2 seconds
            break;
        case 4: //start rev traj
            drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
            state++;
            break;
        default:
            break;
        }
        
    }else
    {

        if(stick.GetYButton())
        {
            if(stick.GetYButtonPressed())
            {
                timer.Reset();//reset timer
                state = 0;
                drive.ResetEncoders();
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath1Points, Power10_Powerpath1Len, true, true);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath1Points, Power10_Powerpath1Len, false, true);
            }
            switch (state)
            {
            case 0: //start traj
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 1: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 2: //prepare traj and start next traj
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath2Points, Power10_Powerpath2Len, true, false);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath2Points, Power10_Powerpath2Len, false, false);
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 3: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 4: //start rev traj
                timer.Reset();
                timer.Start();
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath3Points, Power10_Powerpath3Len, true, false);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath3Points, Power10_Powerpath3Len, false, false);
                state++;
                break;
            case 5:
                if(timer.Get()>0.50){state++;}//wait for FIRE
                break;
            case 6: 
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 7: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 8: //prepare traj and start next traj
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath4Points, Power10_Powerpath4Len, true, true);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath4Points, Power10_Powerpath4Len, false, true);
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 9: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 10: //prepare traj and start next traj
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath5Points, Power10_Powerpath5Len, true, false);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath5Points, Power10_Powerpath5Len, false, false);
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 11: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 12: //prepare traj and start next traj
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath6Points, Power10_Powerpath6Len, true, true);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath6Points, Power10_Powerpath6Len, false, true);
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 13: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            case 14: //prepare traj and start next traj
                tragTool.GetStreamFromArray(left_bufferedStream, Power10_Powerpath7Points, Power10_Powerpath7Len, true, false);
                tragTool.GetStreamFromArray(right_bufferedStream, Power10_Powerpath7Points, Power10_Powerpath7Len, false, false);
                drive.StartMotionProfile(left_bufferedStream, right_bufferedStream, ControlMode::MotionProfile); 
                state++;
                break;
            case 15: //wait for traj to finish
                if(drive.IsMotionProfileFinished()){state++;}
                break;
            default:
                break;
            }

        }else
        {
            drive.SetSpeed(ControlMode::PercentOutput, lv, rv);
        }
    }
*/
    
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
