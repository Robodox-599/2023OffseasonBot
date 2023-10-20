// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.subsystem_WristMotor;

public class command_RunMotor extends CommandBase {
  /** Creates a new command_RunMotor. */
  private final subsystem_WristMotor m_Wrist;
  private final DoubleSupplier m_EncPosition;
  private final BooleanSupplier m_IsWait;
  private final DoubleSupplier m_Threshold;
  private final DoubleSupplier m_StickPos;
  private final Timer m_Timer = new Timer();

  public command_RunMotor(subsystem_WristMotor Wrist, DoubleSupplier EncPosition, BooleanSupplier IsWait, DoubleSupplier Threshold, DoubleSupplier StickPos) {
    m_Wrist = Wrist;
    m_EncPosition = EncPosition;
    m_IsWait = IsWait;
    m_Threshold = Threshold;
    m_StickPos = StickPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //frc::SmartDashboard::SmartDashboard::PutBoolean("initializing", true);
    m_Timer.start();
    m_Wrist.setWristByPosition(m_EncPosition.getAsDouble());
  
    //frc::SmartDashboard::SmartDashboard::PutNumber("Enc Pos", m_EncPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Wrist.isWristAtDesiredPosition()){
      m_Timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_IsWait.getAsBoolean() && m_Timer.get() < WristConstants.TargetTime){
      if(m_Wrist.wristThreshold(m_Threshold.getAsDouble())){
        return true;
      }
      return false;
    }
    else{
      return true;
    }
  
  }
}
