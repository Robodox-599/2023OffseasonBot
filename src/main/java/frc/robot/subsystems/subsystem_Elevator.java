// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class subsystem_Elevator extends SubsystemBase {
  /** Creates a new subsystem_Elevator. */
  private CANSparkMax m_ElevatorMotor;
  private CANSparkMax m_ElevatorFollower;
  private DutyCycleEncoder m_ElevatorEncoder;
  private SparkMaxPIDController m_ElevatorMotorPID;
  private SparkMaxPIDController m_ElevatorFollowerPID;
  private RelativeEncoder m_ElevatorMotorEncoder;
  private RelativeEncoder m_ElevatorFollowerEncoder;

  private double desiredElevatorPosition = 0.0;
  private double elevatorEnc = 0.0;
  private int desiredElevatorSlot = 0;

  public subsystem_Elevator() {
    m_ElevatorMotor = new CANSparkMax(Constants.ElevatorConstants.kElevatorID, MotorType.kBrushless);
    m_ElevatorFollower = new CANSparkMax(Constants.ElevatorConstants.kElevatorFollowerID, MotorType.kBrushless);
    m_ElevatorEncoder = new DutyCycleEncoder(Constants.ElevatorConstants.kEncoderID);
    m_ElevatorMotorPID = m_ElevatorMotor.getPIDController();
    m_ElevatorFollowerPID = m_ElevatorFollower.getPIDController();
    m_ElevatorMotorEncoder = m_ElevatorMotor.getEncoder();
    m_ElevatorFollowerEncoder = m_ElevatorFollower.getEncoder();

    m_ElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_ElevatorFollower.setIdleMode(IdleMode.kBrake);

    m_ElevatorMotor.setIdleMode(IdleMode.kBrake);
    m_ElevatorFollower.setIdleMode(IdleMode.kBrake);

    m_ElevatorMotor.setInverted(true);
    m_ElevatorFollower.setInverted(true);

    m_ElevatorMotor.setSmartCurrentLimit(5, 35, 117);
    m_ElevatorFollower.setSmartCurrentLimit(5, 35, 117);

    m_ElevatorFollower.follow(m_ElevatorMotor);
    
    m_ElevatorMotorPID.setP(Constants.ElevatorConstants.kUpElevatorP, Constants.ElevatorConstants.kUpElevatorSlot);
    m_ElevatorMotorPID.setI(Constants.ElevatorConstants.kUpElevatorI, Constants.ElevatorConstants.kUpElevatorSlot);
    m_ElevatorMotorPID.setD(Constants.ElevatorConstants.kUpElevatorD, Constants.ElevatorConstants.kUpElevatorSlot);

    m_ElevatorFollowerPID.setP(Constants.ElevatorConstants.kUpElevatorP, Constants.ElevatorConstants.kUpElevatorSlot);
    m_ElevatorFollowerPID.setI(Constants.ElevatorConstants.kUpElevatorI, Constants.ElevatorConstants.kUpElevatorSlot);
    m_ElevatorFollowerPID.setD(Constants.ElevatorConstants.kUpElevatorD, Constants.ElevatorConstants.kUpElevatorSlot);

    m_ElevatorMotorPID.setP(Constants.ElevatorConstants.kDownElevatorP, Constants.ElevatorConstants.kDownElevatorSlot);
    m_ElevatorMotorPID.setI(Constants.ElevatorConstants.kDownElevatorI, Constants.ElevatorConstants.kDownElevatorSlot);
    m_ElevatorMotorPID.setD(Constants.ElevatorConstants.kDownElevatorD, Constants.ElevatorConstants.kDownElevatorSlot);

    m_ElevatorFollowerPID.setP(Constants.ElevatorConstants.kDownElevatorP, Constants.ElevatorConstants.kDownElevatorSlot);
    m_ElevatorFollowerPID.setI(Constants.ElevatorConstants.kDownElevatorI, Constants.ElevatorConstants.kDownElevatorSlot);
    m_ElevatorFollowerPID.setD(Constants.ElevatorConstants.kDownElevatorD, Constants.ElevatorConstants.kDownElevatorSlot);

    setElevatorPosition(Constants.ElevatorConstants.kElevatorStow);
    SmartDashboard.putNumber("motor current", m_ElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("follower motor current", m_ElevatorFollower.getOutputCurrent());
  }

  public void resetElevatorEncoder(){
    m_ElevatorEncoder.reset();
  }

  public void setElevatorPosition(double elevatorPosition){
    
    desiredElevatorPosition = elevatorPosition;

    if(elevatorPosition == Constants.ElevatorConstants.kElevatorStow){
      desiredElevatorSlot = Constants.ElevatorConstants.kDownElevatorSlot;
      m_ElevatorMotorPID.setReference(desiredElevatorPosition, ControlType.kPosition, desiredElevatorSlot);
    }
    else {
      desiredElevatorSlot = Constants.ElevatorConstants.kUpElevatorSlot;
      m_ElevatorMotorPID.setReference(desiredElevatorPosition, ControlType.kPosition, desiredElevatorSlot);
    }     

    SmartDashboard.putNumber("Elevator Position Passed In", elevatorPosition);
  }

  public void moveElevatorManually(double axis){
    
    desiredElevatorPosition += axis * Constants.ElevatorConstants.kManualSpeed;
    
    if(axis < 0){
      desiredElevatorSlot = Constants.ElevatorConstants.kDownElevatorSlot;
      m_ElevatorMotorPID.setReference(desiredElevatorPosition, ControlType.kPosition, desiredElevatorSlot);
    }
    if(axis > 0){
      desiredElevatorSlot = Constants.ElevatorConstants.kUpElevatorSlot;
      m_ElevatorMotorPID.setReference(desiredElevatorPosition, ControlType.kPosition, desiredElevatorSlot);
    }
  }

  public double getElevatorPosition(){
    return m_ElevatorEncoder.getDistance();
  }

  public double getElevatorMotorPosition(){
    return m_ElevatorMotorEncoder.getPosition();
  }

  public void setElevatorMotorSpeed(double speed){
    m_ElevatorMotor.set(speed);
    m_ElevatorFollower.set(speed);
  }

  public boolean isElevatorAtDesiredPosition(){
    return Math.abs(desiredElevatorPosition - elevatorEnc) < Constants.ElevatorConstants.kBufferZone;
  }

  public boolean elevatorThreshold(double threshold){
    return desiredElevatorPosition >= 0.0 ? elevatorEnc > threshold : elevatorEnc < threshold;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEnc = m_ElevatorMotorEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Pos", elevatorEnc);

    if(!isElevatorAtDesiredPosition()){
      SmartDashboard.putNumber("Desired Positin", desiredElevatorPosition);
      m_ElevatorMotorPID.setReference(desiredElevatorPosition, ControlType.kPosition, desiredElevatorSlot);
      SmartDashboard.putBoolean("Went Into If-Statement", true);
    }
    else {
      SmartDashboard.putBoolean("Went Into If-Statement", false);
    }

    SmartDashboard.putNumber("Current Motor Encoder Value", elevatorEnc);
  }
}
