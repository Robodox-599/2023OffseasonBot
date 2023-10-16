// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Outtake.h"

command_Outtake::command_Outtake(subsystem_Intake* intake, subsystem_LED* LED) : m_Intake{intake}, m_LED{LED} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
  AddRequirements({m_LED});
}

// Called when the command is initially scheduled.
void command_Outtake::Initialize() {
  if(m_Intake->getState() == IntakeConstants::hasCube){
    m_Intake->setState(IntakeConstants::cubeOuttaking);
  } else {
    m_Intake->setState(IntakeConstants::coneOuttaking);
  }
  m_Timer.Restart();
}

// Called repeatedly when this Command is scheduled to run
void command_Outtake::Execute() {}

// Returns true when the command should end.
bool command_Outtake::IsFinished() {
  return m_Timer.Get() > 0.5_s;
}

// Called once the command ends or is interrupted.
void command_Outtake::End(bool interrupted) {
  m_Intake->setState(IntakeConstants::noGamepiece);
}