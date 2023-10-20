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

import edu.wpi.first.wpilibj2.command.CommandBase;

public class subsystem_WristMotor extends SubsystemBase {
  private final CANSparkMax m_Motor;
  private final SparkMaxPIDController m_WristPID;
  private final RelativeEncoder m_WristEncoder;
  private double m_Angle;
  private double WristEnc = 0.0;
  private double DesiredWristPosition;
  /** Creates a new subsystem_WristMotor. */
  public subsystem_WristMotor(){
    // what would've been () : {}
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
  DesiredWristPosition += StickPos * WristConstants.stepperConstant;
  m_WristPID.setReference(DesiredWristPosition, CANSparkMax.ControlType.kPosition);
 // return RunOnce([this,StickPos]{return SetWristByPosition(WristEnc+(StickPos*WristConstants.stepperConstant));});
} 


public boolean isWristAtDesiredPosition(){
  if(Math.abs(DesiredWristPosition - WristEnc) < WristConstants.bufferZone){
      return true;
  }else{
      return false;
    }
}

public boolean wristThreshold(double Threshold){
  if(DesiredWristPosition >= WristEnc){
      return WristEnc > Threshold;
  } else{
      return WristEnc < Threshold;
  }
}

public void setWristByPosition(double tiltPos){
  DesiredWristPosition = tiltPos;
  //frc.SmartDashboard.SmartDashboard.PutNumber("tilt pos", tiltPos);
}
  @Override
  public void periodic() {
    WristEnc = m_WristEncoder.getPosition();
    m_Angle = (m_WristEncoder.getPosition() / 0.2325  );
    if (!isWristAtDesiredPosition()){
        //frc.SmartDashboard.SmartDashboard.PutNumber("desiredwristposition", DesiredWristPosition);
        m_WristPID.setReference(DesiredWristPosition, CANSparkMax.ControlType.kPosition, 0);
        //frc.SmartDashboard.SmartDashboard.PutBoolean("went into if statement", true);
    } else {         
        //frc.SmartDashboard.SmartDashboard.PutBoolean("went into if statement", false);
    }
    //frc.SmartDashboard.SmartDashboard.PutNumber("current encoder val", WristEnc);
    }
}
