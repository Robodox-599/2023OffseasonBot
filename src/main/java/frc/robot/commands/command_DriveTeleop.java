// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subsystem_DriveTrain;

public class command_DriveTeleop extends CommandBase {
  /** Creates a new command_DriveTeleop. */
  private subsystem_DriveTrain m_DriveTrain;
  private DoubleSupplier m_XSpeed;
  private DoubleSupplier m_YSpeed;
  private DoubleSupplier m_ZRot;

  public command_DriveTeleop(subsystem_DriveTrain driveTrain, DoubleSupplier xSpeed, 
                            DoubleSupplier ySpeed, DoubleSupplier zRot) {
    
    m_DriveTrain = driveTrain;
    m_XSpeed = xSpeed;
    m_YSpeed = ySpeed;
    m_ZRot = zRot;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.stopModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_XSpeed.getAsDouble();
    double ySpeed = m_YSpeed.getAsDouble();
    double zRot = m_ZRot.getAsDouble();

    xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadband ? ySpeed : 0.0;
    zRot = Math.abs(zRot) > OperatorConstants.kDeadband ? zRot : 0.0;

    m_DriveTrain.drive(
      xSpeed * SwerveConstants.maxSpeed, 
      ySpeed * SwerveConstants.maxSpeed,
      zRot * SwerveConstants.maxAngularVelocity);
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
