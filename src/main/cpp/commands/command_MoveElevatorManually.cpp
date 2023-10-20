// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveElevatorManually.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

command_MoveElevatorManually::command_MoveElevatorManually(subsystem_Elevator* Elevator, std::function<double()> triggers) :  m_Elevator{Elevator}, m_TriggerInput{triggers} {

  AddRequirements({m_Elevator});
  // Use addRequirements() here to declare subsystem dependencies.

  //Object of subsystem in robotcontainer values assign to use them locally
}

// Called when the command is initially scheduled.
void command_MoveElevatorManually::Initialize() {

  

}

// Called repeatedly when this Command is scheduled to runsdfsdfsdfsdfsdfsdfdf
void command_MoveElevatorManually::Execute() {

  frc::SmartDashboard::SmartDashboard::PutNumber("Manual joystick entered exceute",true);
  frc::SmartDashboard::SmartDashboard::PutNumber("Trigger input val", m_TriggerInput());
  if(fabs(m_TriggerInput() > ControllerConstants::Deadband))
  {
    m_Elevator->MoveElevatorManually(m_TriggerInput());
  }
}

// Called once the command ends or is interrupted.
void command_MoveElevatorManually::End(bool interrupted) {}

// Returns true when the command should end.
bool command_MoveElevatorManually::IsFinished() {
  return false;
}
