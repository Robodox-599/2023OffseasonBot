// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.subsystem_WristMotor;

public class command_ManualMotor extends CommandBase {
  /** Creates a new command_ManualMotor. */
  private final DoubleSupplier m_StickPos;
  private final subsystem_WristMotor m_Wrist;

  public command_ManualMotor(subsystem_WristMotor Wrist, DoubleSupplier StickPos) {
    m_Wrist = Wrist;
    m_StickPos = StickPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_StickPos.getAsDouble()) > OperatorConstants.ManualThreshold){
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
