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
    gyro.configFactoryDefault();
    zeroGyro();

    Timer.delay(5);
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    // m_AutoBuilder.configureHolonomic(null, null, null, null, null, null);

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
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    mSwerveMods[0].setDesiredState(swerveModuleStates[mSwerveMods[0].moduleNumber], false);
    mSwerveMods[1].setDesiredState(swerveModuleStates[mSwerveMods[1].moduleNumber], false);
    mSwerveMods[2].setDesiredState(swerveModuleStates[mSwerveMods[2].moduleNumber], false);
    mSwerveMods[3].setDesiredState(swerveModuleStates[mSwerveMods[3].moduleNumber], false);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    mSwerveMods[0].setDesiredState(desiredStates[mSwerveMods[0].moduleNumber], false);
    mSwerveMods[1].setDesiredState(desiredStates[mSwerveMods[1].moduleNumber], false);
    mSwerveMods[2].setDesiredState(desiredStates[mSwerveMods[2].moduleNumber], false);
    mSwerveMods[3].setDesiredState(desiredStates[mSwerveMods[3].moduleNumber], false);
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
    states[0] = mSwerveMods[0].getState();
    states[1] = mSwerveMods[1].getState();
    states[2] = mSwerveMods[2].getState();
    states[3] = mSwerveMods[3].getState();
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

    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber(
          "Mod " + mSwerveMods[i].moduleNumber + " Cancoder", mSwerveMods[i].getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mSwerveMods[i].moduleNumber + " Integrated", mSwerveMods[i].getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mSwerveMods[i].moduleNumber + " Velocity", mSwerveMods[i].getState().speedMetersPerSecond);
      mSwerveMods[i].periodic();
    }
  }
}
