// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>


command_MoveElevator::command_MoveElevator(subsystem_Elevator* Elevator, 
                                            std::function<double()> EncPosition,
                                            std::function<bool()> IsWait, 
                                            std::function<double()> Threshold) : 
                                            m_Elevator{Elevator},
                                            m_EncPosition{EncPosition},
                                            m_IsWait{IsWait},
                                            m_Threshold{Threshold}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Elevator});
}

// Called when the command is initially scheduled.
void command_MoveElevator::Initialize() 
{
  frc::SmartDashboard::SmartDashboard::PutBoolean("Initalize", true);
  m_Timer.Start();
  m_Elevator->SetElevatorPosition(m_EncPosition());
  frc::SmartDashboard::SmartDashboard::PutNumber("Enc Pos", m_EncPosition());
 
}

// Called repeatedly when this Command is scheduled to run
void command_MoveElevator::Execute() 
{
  
  if(!m_Elevator->IsElevatorAtDesiredPosition())
  {
    m_Timer.Reset();
    //m_Elevator->SetElevatorMotorSpeed(0);
  }
  // if(m_Timer.Get().value() > ElevatorConstants::MaxTimeAllowed)
  // {
  //   //m_Elevator->SetElevatorMotorSpeed(0);
  // }
  //frc::SmartDashboard::SmartDashboard::PutBoolean("At Desired Position", m_Elevator->IsElevatorAtDesiredPosition());

  // if(fabs(m_Elevator->GetElevatorPosition() - m_Elevator->DesiredElevatorPosition) >= ElevatorConstants::kBufferZone)
  //   {
  //     m_Elevator->SetElevatorMotorSpeed(0);
  //   }
  // if(fabs(m_Elevator->GetElevatorsMotorPosition() - m_Elevator->GetElevatorPosition()) <= m_Elevator->GetElevatorsMotorPosition()*0.05)
  // {
    
  // }
  // put this back in if it dont work m_EncPosition()
  // if(!m_Elevator->IsElevatorAtDesiredPosition())
  // { 
  //   m_Timer.Reset();
  // }
}

// Called once the command ends or is interrupted.
void command_MoveElevator::End(bool interrupted) {}

// Returns true when the command should end.
bool command_MoveElevator::IsFinished()
{
  
  if( m_IsWait() && m_Timer.Get() < ElevatorConstants::TargetTime){
    if(m_Elevator->ElevatorThreshold(m_Threshold())){
      return true;
    }
    return false;
  }else{
    return true;
  }
}
