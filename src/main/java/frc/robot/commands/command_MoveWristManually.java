// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subsystem_Wrist;

import frc.robot.Constants;

public class command_MoveWristManually extends CommandBase {
  /** Creates a new command_MoveWristManually. */
  private subsystem_Wrist m_Wrist;
  private DoubleSupplier m_StickPos;

  public command_MoveWristManually(subsystem_Wrist wrist, DoubleSupplier stickPos) {
    m_Wrist = wrist;
    m_StickPos = stickPos;

    addRequirements(m_Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_StickPos.getAsDouble()) > Constants.WristConstants.manualThreshold){
      m_Wrist.manualWrist(m_StickPos.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
