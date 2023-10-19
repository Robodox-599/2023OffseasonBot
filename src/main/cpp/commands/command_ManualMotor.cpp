// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ManualMotor.h"

command_ManualMotor::command_ManualMotor(subsystem_WristMotor* Wrist, std::function<double()> StickPos):
                                         m_Wrist{Wrist},
                                        m_StickPos{StickPos} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Wrist});
}

// Called when the command is initially scheduled.
void command_ManualMotor::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void command_ManualMotor::Execute() {
  if(fabs(m_StickPos()) > OperatorConstants::ManualThreshold){
    m_Wrist->ManualWrist(m_StickPos());
  }
}

// Called once the command ends or is interrupted.
void command_ManualMotor::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ManualMotor::IsFinished() {
  return false;
}
