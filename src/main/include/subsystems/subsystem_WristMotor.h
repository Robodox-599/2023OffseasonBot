// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <units/voltage.h>

class subsystem_WristMotor : public frc2::SubsystemBase {
 public:
  subsystem_WristMotor();
  frc2::CommandPtr ResetWrist();
  //frc2::CommandPtr Substation();
  // frc2::CommandPtr HighCone();
  // frc2::CommandPtr MidCone();
  // frc2::CommandPtr MidCube();
  // frc2::CommandPtr HighCube();
  void SetWristByPosition(double WristPos);
  frc2::CommandPtr MoveWristCommand(double EncPosition);
  void ManualWrist(double EncPosition);
  frc2::CommandPtr ZeroWrist();
  bool IsWristAtDesiredPosition();
  bool WristThreshold(double Threshold);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 
  rev::CANSparkMax m_Motor;
  rev::SparkMaxPIDController m_WristPID;
  rev::SparkMaxRelativeEncoder m_WristEncoder; 
  double m_Angle; //set to starting angle
  double WristEnc = 0.0;
  double DesiredWristPosition; //set to starting angle
  



  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
