// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subsystem_DriveTrain;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.robot.Constants.Swerve;

public class command_DriveAuton extends CommandBase {
  /** Creates a new command_DriveAuton. */
  //https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Installing
  //https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
  subsystem_DriveTrain m_DriveTrain;
  PathPlannerTrajectory m_Trajectory;
  BooleanSupplier m_ToReset;
  HolonomicDriveController m_DriveController;
  Timer m_Timer;
  
  public command_DriveAuton(subsystem_DriveTrain driveTrain, String trajFilePath, BooleanSupplier toReset) {
    m_DriveTrain = driveTrain;
    m_ToReset = toReset;
    m_DriveController = new HolonomicDriveController(
      AutoConstants.xPIDController, 
      AutoConstants.yPIDController, 
      AutoConstants.thetaPIDController
    );
    m_Timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.stop();
    m_Timer.reset();
    m_Timer.start();

    if(m_ToReset.getAsBoolean()){
      m_DriveTrain.resetOdometry(m_Trajectory.getInitialTargetHolonomicPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerTrajectory.State state = m_Trajectory.sample(m_Timer.get());
    ChassisSpeeds chassisSpeeds= m_DriveController.calculate(
      m_DriveTrain.getPose(),
      state.getTargetHolonomicPose(),
      state.velocityMps,
      state.targetHolonomicRotation
    );
    SwerveModuleState[] moduleStates = Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    // desaturateWheelSpeeds(moduleStates, AutoConstants.kMaxSpeedMetersPerSecond);
    Swerve.swerveKinematics.desaturateWheelSpeeds(moduleStates, Swerve.maxSpeed);
    m_DriveTrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.drive(new Translation2d(0, 0), 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() >= (m_Trajectory.getTotalTimeSeconds());
  }
}
