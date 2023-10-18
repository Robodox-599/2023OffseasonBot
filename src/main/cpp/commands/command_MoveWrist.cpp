// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveWrist.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_MoveWrist::command_MoveWrist(subsystem_Wrist* Wrist, 
                                  std::function<double()> EncPosition, 
                                  std::function<bool()> IsWait, 
                                  std::function<double()> Threshold):
                                  m_Wrist{Wrist},
                                  m_EncPosition{EncPosition},
                                  m_IsWait{IsWait},
                                  m_Threshold{Threshold}
                                  {
    AddRequirements({m_Wrist});
  }

// Called when the command is initially scheduled.
void command_MoveWrist::Initialize() {  
  frc::SmartDashboard::SmartDashboard::PutBoolean("initializing", true);
  m_Timer.Start();
  m_Wrist->SetWristByPosition(m_EncPosition());
  
  frc::SmartDashboard::SmartDashboard::PutNumber("Enc Pos", m_EncPosition());
}

// Called repeatedly when this Command is scheduled to run
void command_MoveWrist::Execute() {
  if (!m_Wrist->IsWristAtDesiredPosition()){
    m_Timer.Reset();
  }
}

// Called once the command ends or is interrupted.
void command_MoveWrist::End(bool interrupted) {}

// Returns true when the command should end.
bool command_MoveWrist::IsFinished() {
  // if (m_wrist->IsWristAtDesiredPosition()){
  //   return true;
  // } else {
  //   return false;
  // }
  if( m_IsWait() && m_Timer.Get() < WristConstants::TargetTime){
    if(m_Wrist->WristThreshold(m_Threshold())){
      return true;
    }
    return false;
  }
  else{
    return true;
  }

  // return m_IsWait() && m_Timer.Get() < WristConstants::TargetTime ? m_Wrist->WristThreshold(m_Threshold()) : true;
  // return m_Wrist->IsWristAtDesiredPosition();
  }
