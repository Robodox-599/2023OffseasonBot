// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class subsystem_Intake extends SubsystemBase {
  /** Creates a new subsystem_Intake. */
  private intakeState m_IntakeState = intakeState.noGamepiece;
  private CANSparkMax m_IntakeMotor;

  public subsystem_Intake() {
    m_IntakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    m_IntakeMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit);
  }

  public boolean checkGamepieceIntake(){
    return (m_IntakeState == intakeState.cubeIntaking || m_IntakeState == intakeState.coneIntaking)
    && m_IntakeMotor.getOutputCurrent() > IntakeConstants.normalOutputCurrent;
  }

  public intakeState getState(){
    return m_IntakeState;
  }

  public void setState(intakeState state){
    m_IntakeState = state;
    switch (m_IntakeState){
      case coneIntaking:
      case cubeOuttaking:
        m_IntakeMotor.set(-0.5);
        break;
      case coneOuttaking:
      case cubeIntaking:
        m_IntakeMotor.set(0.5);
        break;
      case hasCone:
      case hasCube:
      case noGamepiece:
        m_IntakeMotor.set(0);
        break;
      default:
        m_IntakeMotor.set(0);
    }
  }
  
  public void setBrakeMode(){
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode(){
    m_IntakeMotor.setIdleMode(IdleMode.kCoast);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase toggleIntake(){
    // implicitly require `this`

    return m_IntakeMotor.get()> 0.1 ? this.runOnce(() -> m_IntakeMotor.set(0)) : 
    this.runOnce(() -> m_IntakeMotor.set(0.5));
}

}
