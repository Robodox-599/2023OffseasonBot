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

class subsystem_DriveTank : public frc2::SubsystemBase {
 public:
  subsystem_DriveTank();
  frc2::CommandPtr Drive(double speed);
  frc2::CommandPtr Turn(double pos);
  bool IsLeftAngleAtDesired();
  bool IsRightAngleAtDesired();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:\
 rev::CANSparkMax m_LeftBackMotor;
 rev::CANSparkMax m_RightBackMotor;
 rev::CANSparkMax m_LeftFrontMotor;
 rev::CANSparkMax m_RightFrontMotor;
 rev::SparkMaxPIDController m_LeftFrontMotorPID;
 rev::SparkMaxPIDController m_RightFrontMotorPID;

  rev::SparkMaxRelativeEncoder m_LeftFrontEncoder; 
  rev::SparkMaxRelativeEncoder m_RightFrontEncoder; 
  double LeftEncValue;
  double RightEncValue;
  double DesiredTurnPosition; 
  double DesiredSpeed;

  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
