// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

subsystem_Intake::subsystem_Intake(): 
                                      m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMax::MotorType::kBrushless} 
{
  m_IntakeMotor.SetSmartCurrentLimit(IntakeConstants::IntakeMotorCurrentLimit);
}

bool subsystem_Intake::CheckGamepieceIntake(){
  frc::SmartDashboard::SmartDashboard::PutNumber("Output Current", m_IntakeMotor.GetOutputCurrent());
   return (m_intakeState == IntakeConstants::cubeIntaking || m_intakeState == IntakeConstants::coneIntaking)
  && m_IntakeMotor.GetOutputCurrent() > IntakeConstants::normalOutputCurrent;
}

IntakeConstants::intakeState subsystem_Intake::getState(){
  return m_intakeState;
}

void subsystem_Intake::setState(IntakeConstants::intakeState state){
  m_intakeState = state;
}



// This method will be called once per scheduler run
void subsystem_Intake::Periodic() {
  switch(m_intakeState){
    case (IntakeConstants::coneIntaking):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "coneIntaking");
    m_IntakeMotor.Set(0.5);
      break;
    case (IntakeConstants::cubeOuttaking):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "cubeOuttaking");
      m_IntakeMotor.Set(0.5);
      break;
    case (IntakeConstants::cubeIntaking):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "cubeIntaking");
    m_IntakeMotor.Set(-0.5);
      break;
    case (IntakeConstants::coneOuttaking):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "coneOuttaking");
      m_IntakeMotor.Set(-0.5);
      break;
    case (IntakeConstants::hasCone):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "hasCone");
    m_IntakeMotor.Set(0);
    break;
    case (IntakeConstants::hasCube):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "hasCube");
    m_IntakeMotor.Set(0);
    break;
    case (IntakeConstants::noGamepiece):
    frc::SmartDashboard::SmartDashboard::PutString("Intake State", "noGamepiece");
    m_IntakeMotor.Set(0);
    break;
  }
}
