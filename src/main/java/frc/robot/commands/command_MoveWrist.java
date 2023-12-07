// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.subsystem_Wrist;

public class command_MoveWrist extends CommandBase {
  /** Creates a new command_RunMotor. */
  private final subsystem_Wrist m_Wrist;
  private final DoubleSupplier m_EncPosition;
  private final BooleanSupplier m_IsWait;
  private final DoubleSupplier m_Threshold;
  private final Timer m_Timer = new Timer();

  public command_MoveWrist(subsystem_Wrist Wrist, DoubleSupplier EncPosition, BooleanSupplier IsWait, DoubleSupplier Threshold) {
    m_Wrist = Wrist;
    m_EncPosition = EncPosition;
    m_IsWait = IsWait;
    m_Threshold = Threshold;
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
    SmartDashboard.putBoolean("cool1", m_IsWait.getAsBoolean() && m_Timer.get() < WristConstants.targetTime);
    if(m_IsWait.getAsBoolean() && m_Timer.get() < WristConstants.targetTime){
      SmartDashboard.putNumber("gad", m_Threshold.getAsDouble());
      return m_Wrist.wristThreshold(m_Threshold.getAsDouble());
    }
    else{
      return true;
    }
  }
}
