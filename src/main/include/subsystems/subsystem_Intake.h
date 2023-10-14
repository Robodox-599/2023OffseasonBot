// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include <rev/CANSparkMax.h>

class subsystem_Intake : public frc2::SubsystemBase {
 public:
  subsystem_Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  bool CheckGamepieceIntake();
  IntakeConstants::intakeState getState();
  void setState(IntakeConstants::intakeState state);
  void setBrakeMode();
  void setCoastMode();
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  IntakeConstants::intakeState m_intakeState = IntakeConstants::noGamepiece;
  rev::CANSparkMax m_IntakeMotor;
};
