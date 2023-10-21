// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.WristConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class subsystem_Wrist extends SubsystemBase {
  private CANSparkMax m_Motor;
  private SparkMaxPIDController m_WristPID;
  private RelativeEncoder m_WristEncoder;
  // private double m_Angle;

  private double wristEnc = 0.0;
  private double desiredWristPosition = 0.0;
  /** Creates a new subsystem_WristMotor. */
  public subsystem_Wrist(){

    m_Motor = new CANSparkMax(WristConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_WristPID = m_Motor.getPIDController();
    m_WristEncoder = m_Motor.getEncoder();

    //actual constructor now
    m_WristPID.setP(WristConstants.kWristP);
    m_WristPID.setI(WristConstants.kWristI);
    m_WristPID.setD(WristConstants.kWristD);
    m_WristEncoder.setPosition(WristConstants.start_pos);
    m_Motor.setVoltage(12);
    m_Motor.setSmartCurrentLimit(5, 7, 2650/9);
  }

public CommandBase resetWrist(){
  return this.runOnce(() -> m_WristEncoder.setPosition(WristConstants.start_pos));
}

public CommandBase moveWristCommand(double desiredPos){
  return this.runOnce(() -> setWristByPosition(desiredPos));
}

public CommandBase zeroWrist(){
  return this.runOnce(() -> setWristByPosition(0));
}


//call Reset Wrist and zero wrist hand in hand !!!!!

public void manualWrist(double StickPos){
  desiredWristPosition += StickPos * WristConstants.stepperConstant;
  m_WristPID.setReference(desiredWristPosition, CANSparkMax.ControlType.kPosition);
 } 


public boolean isWristAtDesiredPosition(){
  return Math.abs(desiredWristPosition - wristEnc) < WristConstants.bufferZone;
}

public boolean wristThreshold(double Threshold){
  return desiredWristPosition >= wristEnc ? wristEnc > Threshold : wristEnc < Threshold;
}

public void setWristByPosition(double tiltPos){
  desiredWristPosition = tiltPos;
  SmartDashboard.putNumber("tilt pos", tiltPos);
}
  @Override
  public void periodic() {
    wristEnc = m_WristEncoder.getPosition();
    if (!isWristAtDesiredPosition()){
        SmartDashboard.putNumber("desiredwristposition", desiredWristPosition);
        m_WristPID.setReference(desiredWristPosition, CANSparkMax.ControlType.kPosition, 0);
        SmartDashboard.putBoolean("went into if statement", true);
    } else {         
        SmartDashboard.putBoolean("went into if statement", false);
    }
    SmartDashboard.putNumber("current encoder val", wristEnc);
    }
}
