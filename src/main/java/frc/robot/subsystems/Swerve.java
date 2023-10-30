package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  // AutoBuilder m_AutoBuilder;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID, "BarryDriveCANivore");
    Timer.delay(0.2);
    gyro.configFactoryDefault();
    Timer.delay(0.2);
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for(int i = 0; i < 4; i++){
      modulePositions[i] = mSwerveMods[i].getPosition();
      // SmartDashboard.putNumber("Angle...?" + mSwerveMods[i].moduleNumber, mSwerveMods[i].getPosition().angle.getDegrees());
    }

    return modulePositions;
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    
    double vel_y = translation.getX();
    double vel_x = translation.getY();
    double omega_x = rotation * Constants.Swerve.wheelBase / 2.0;
    double omega_y = rotation * Constants.Swerve.trackWidth / 2.0;
    
    /* FL Module */
    double vel_x_FL = vel_x + omega_x;
    double vel_y_FL = vel_y + omega_y;
    double vel_FL = Math.hypot(vel_x_FL, vel_y_FL);
    double theta_FL = Math.atan2(vel_x_FL, vel_y_FL) * 180.0 / Math.PI;
    swerveModuleStates[0] = new SwerveModuleState(vel_FL, Rotation2d.fromDegrees(theta_FL));
    
    /* FR Module */
    double vel_x_FR = vel_x + omega_x;
    double vel_y_FR = vel_y - omega_y;
    double vel_FR = Math.hypot(vel_x_FR, vel_y_FR);
    double theta_FR = Math.atan2(vel_x_FR, vel_y_FR) * 180.0 / Math.PI;
    swerveModuleStates[1] = new SwerveModuleState(vel_FR, Rotation2d.fromDegrees(theta_FR));

    /* BL Module */
    double vel_x_BL = vel_x - omega_x;
    double vel_y_BL = vel_y + omega_y;
    double vel_BL = Math.hypot(vel_x_BL, vel_y_BL);
    double theta_BL = Math.atan2(vel_x_BL, vel_y_BL) * 180.0 / Math.PI;
    swerveModuleStates[2] = new SwerveModuleState(vel_BL, Rotation2d.fromDegrees(theta_BL));

    /* BR Module */
    double vel_x_BR = vel_x - omega_x;
    double vel_y_BR = vel_y - omega_y;
    double vel_BR = Math.hypot(vel_x_BR, vel_y_BR);
    double theta_BR = Math.atan2(vel_x_BR, vel_y_BR) * 180.0 / Math.PI;
    swerveModuleStates[3] = new SwerveModuleState(vel_BR, Rotation2d.fromDegrees(theta_BR));

        // SwerveModuleState[] swerveModuleStates =
        // Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        //     fieldRelative
        //         ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //             translation.getX(), translation.getY(), rotation, getYaw())
        //         : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public CommandBase zeroModuleAngles(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0.0));
    states[1] = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0.0));
    states[2] = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0.0));
    states[3] = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0.0));
    return this.runOnce(() -> setModuleStates(states));
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
