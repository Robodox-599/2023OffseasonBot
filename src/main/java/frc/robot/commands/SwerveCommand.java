// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** An example command that uses an example subsystem. */
public class SwerveCommand extends CommandBase {
  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier m_xSpdFunction, m_ySpdFunction, m_turningSpdFunction;
  private final BooleanSupplier m_fieldOrientedFunction;
  

  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public SwerveCommand(SwerveSubsystem swerveSubsystem, 
                      DoubleSupplier xSpdFunction, 
                      DoubleSupplier ySpdFunction, 
                      DoubleSupplier turningSpdFunction,
                      BooleanSupplier fieldOrientedFunction) {
    
        m_swerveSubsystem = swerveSubsystem;
        m_xSpdFunction = xSpdFunction;
        m_ySpdFunction = ySpdFunction;
        m_turningSpdFunction = turningSpdFunction;
        m_fieldOrientedFunction = fieldOrientedFunction;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double xSpeed = m_xSpdFunction.getAsDouble();
        double ySpeed = m_ySpdFunction.getAsDouble();
        double turningSpeed = m_turningSpdFunction.getAsDouble();
         xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OperatorConstants.kDeadband ? turningSpeed : 0.0;

      ChassisSpeeds chassisSpeeds;
        if (m_fieldOrientedFunction.getAsBoolean()) {
            // Relative to field
            chassisSpeeds = new ChassisSpeeds(
                    xSpeed, ySpeed, turningSpeed);
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }
        SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called e the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
