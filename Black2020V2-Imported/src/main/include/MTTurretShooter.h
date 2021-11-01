/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/SerialPort.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <thread>
#include <mutex>

#include "MTPose.hpp"

class MTTurretShooter {
  WPI_TalonFX _lm;
  WPI_TalonFX _rm;

  WPI_TalonSRX _turretMotor;

  frc::SerialPort _serialPort{frc::SerialPort::kMXP};
  PigeonIMU _imu;


  std::thread *_thread;
  std::mutex _mutex;
  bool _autoTrack; 
  bool _threadRunning;
  double threadPeriod = 10000;//10ms

  double _targetHeading;
  double _targetWheelSpeed;

 public:
  MTTurretShooter(int leftMotor, int rightMotor, int turretMotor, int imu);
  ~MTTurretShooter();

  void shoot(double speed);
  void spin(double speed);
  void setWheelSpeed(double speed);
  void sendData(std::string name = "Shooter");

  void updateTargetHeading(double degrees);
  void advTrack();
  void track(MTPoseData drivePose);
  
  void GetHeading();




};
