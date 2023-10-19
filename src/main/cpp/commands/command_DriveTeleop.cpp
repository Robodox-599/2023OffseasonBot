// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveTeleop.h"

command_DriveTeleop::command_DriveTeleop(subsystem_DriveTank* Drive, 
                                        std::function<double()> TurnStick,
                                        std::function<double()> DriveStick):
                                         m_Drive{Drive},
                                         m_TurnInput{TurnStick},
                                         m_DriveInput{DriveStick} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Drive});
}

// Called when the command is initially scheduled.
void command_DriveTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_DriveTeleop::Execute() {
  if (fabs(m_TurnInput()) > MotorConstants::Deadband){
    m_Drive->Turn(m_TurnInput()*60*MotorConstants::TurnGearRatio);
  }
  if (fabs(m_DriveInput()) > MotorConstants::Deadband){
    m_Drive->Drive(m_DriveInput());
  }
  

}

// Called once the command ends or is interrupted.
void command_DriveTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool command_DriveTeleop::IsFinished() {
  return false;
}
