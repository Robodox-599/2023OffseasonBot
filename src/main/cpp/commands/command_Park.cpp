// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Park.h"
#include <frc/smartdashboard/SmartDashboard.h> 

command_Park::command_Park(subsystem_DriveTrain* DriveTrain): m_DriveTrain{DriveTrain} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
}

// Called when the command is initially scheduled.
void command_Park::Initialize() {
  m_DriveTrain->SetPark();
}

// Called repeatedly when this Command is scheduled to run
void command_Park::Execute() {
  frc::SmartDashboard::SmartDashboard::PutBoolean("Is Parked", m_DriveTrain->IsPark());
}

// Called once the command ends or is interrupted.
void command_Park::End(bool interrupted) {}

// Returns true when the command should end.
bool command_Park::IsFinished() {
  return m_DriveTrain->IsPark();
}
