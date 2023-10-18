// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc/DutyCycleEncoder.h>
#include "rev/SparkMaxRelativeEncoder.h"
#include <subsystems/subsystem_Elevator.h>

class subsystem_Elevator : public frc2::SubsystemBase {
 public:

  subsystem_Elevator();


  //void LockElevator();
  //void UnlockElevator();
  void MoveElevator(double angle);

  void ResetElevatorEncoder();
  //void SetElevatorEncoder(double position);
  void SetElevatorPosition(double position);
  //void SetElevatorwPIDByDirection(double desiredElevatorPos);
  void MoveElevatorManually(double axis);
  void SetElevatorMotorSpeed(double speed);

  // frc2::CommandPtr ToMidCone();
  // frc2::CommandPtr ToHighCone();
  // frc2::CommandPtr ToMidCube();
  // frc2::CommandPtr ToHighCube();
  // frc2::CommandPtr ToStow();
  // frc2::CommandPtr ToHybrid();
  // frc2::CommandPtr ToDoubleSubstation();

  double GetElevatorPosition();
  double GetElevatorsMotorPosition();

  bool IsElevatorAtDesiredPosition();
  bool ElevatorThreshold(double Threshold);

  // frc2::CommandPtr SetElevatorMotorSpeedCommand(std::function<double()> speed);
  //void MoveToAngleMotionMagic();

  void Periodic() override;

 private:

  rev::CANSparkMax m_ElevatorMotor;
  rev::CANSparkMax m_ElevatorFollower;
  frc::DutyCycleEncoder m_ElevatorEncoder;
  rev::SparkMaxPIDController m_ElevatorMotorPID;
  rev::SparkMaxPIDController m_ElevatorFollowerPID;
  rev::SparkMaxRelativeEncoder m_MotorEncoder;
  rev::SparkMaxRelativeEncoder m_MotorFollowerEncoder;

  
  double DesiredElevatorPosition = 0.0;
  double ElevatorEnc = 0.0;
};
