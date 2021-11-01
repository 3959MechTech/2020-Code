#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "MechTechDifferential.hpp"
#include "MTPath.h"
#include "thread"
#include "mutex"
#include "frc/SerialPort.h"
#include <frc/XboxController.h>
#include <frc/Notifier.h>
#include "Indexer.hpp"
#include <rev/CANSparkMax.h>
#include "Feeder.h"
#include "Periscope.h"
#include "MTTurretShooter.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


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


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void sendData();

 private:

  void updatePose();
  void executeTasks();
  void devStick();

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc::Timer smTimer{};

  frc::XboxController stick{0};
  frc::XboxController stick2{1};
  frc::XboxController stick3{3};

  MTDifferential drive{10,12,11,13,0,0};
  frc::Timer timer{};
  int state = 0;

  MTPath tragTool{6.0, 2048.0,9.167};//6" wheel diameter, encoder resolution 2048, gear ratio= 20.83/9.167
  BufferedTrajectoryPointStream left_bufferedStream{};
	BufferedTrajectoryPointStream right_bufferedStream{};


  std::thread *_telemetryThread;
  frc::Timer _telemetryTimer{};
  bool _stopTelemetryThread = false;

  frc::Notifier *_PoseThread;
  frc::Timer _PoseTimer{};
  std::mutex _poseThreadMutex;
  double _poseThreadPeriod;

  Periscope periscope{50,51};

  Feeder feeder{60};

  Indexer indexer{30,31,32};

  MTTurretShooter shooter{20,21,22,1};

  frc::SerialPort cam{115200, frc::SerialPort::kMXP};
  //frc::SerialPort cam1{9600, frc::SerialPort::kUSB1};
  //frc::SerialPort cam2{9600, frc::SerialPort::kUSB2};

  std::shared_ptr<NetworkTable> NetTable;


};
