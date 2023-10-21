// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subsystem_DriveTrain;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;

/*import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;*/


import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class command_DriveAuton extends CommandBase {
  /** Creates a new command_DriveAuton. */
  //https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Installing
  //https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
  subsystem_DriveTrain m_DriveTrain;
  //PathPlannerTrajectory m_Trajectory;
  BooleanSupplier m_ToReset;
  HolonomicDriveController m_DriveController;
  Timer m_Timer;
  
  public command_DriveAuton(subsystem_DriveTrain driveTrain, String trajFilePath, BooleanSupplier toReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    // m_Trajectory = PathPlannerPath.fromPathFile(trajFilePath)
    // m_Trajectory = 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
