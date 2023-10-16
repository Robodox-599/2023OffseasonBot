// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

command_Intake::command_Intake(subsystem_Intake* intake, subsystem_LED* LED) : m_Intake{intake}, m_LED{LED}{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
  AddRequirements({m_LED});
}

// Called when the command is initially scheduled.
void command_Intake::Initialize() {
  // if(m_LED->GetLEDState() == LEDConstants::Purple){
  //   m_Intake->setState(IntakeConstants::cubeIntaking);
  // } else {
  //   m_Intake->setState(IntakeConstants::coneIntaking);
  // }
  // m_Timer.Restart();
}

// Called repeatedly when this Command is scheduled to run
void command_Intake::Execute() {
if (!(m_Intake->CheckGamepieceIntake())){
  m_Timer.Restart();
  frc::SmartDashboard::SmartDashboard::PutBoolean("Spike Detected: ", false);
} else {
  frc::SmartDashboard::SmartDashboard::PutBoolean("Spike Detected: ", true);
}
}

// Returns true when the command should end.
bool command_Intake::IsFinished() {
  return m_Timer.Get() > 0.05_s;
}

// Called once the command ends or is interrupted.
void command_Intake::End(bool interrupted) {
  if (m_Intake->getState() == IntakeConstants::cubeIntaking){
    m_Intake->setState(IntakeConstants::hasCube);
  } else {
    m_Intake->setState(IntakeConstants::hasCone);
  }
}