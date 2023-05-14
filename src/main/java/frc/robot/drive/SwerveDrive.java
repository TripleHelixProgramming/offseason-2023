// Copyright (c) Triple Helix Robotics

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElectricalConstants;
import frc.robot.Constants.ModuleConstants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ModuleConstants.*;

@SuppressWarnings("PMD.ExcessiveImports")
public class SwerveDrive extends SubsystemBase {

  private static final int kNumSwerveModules = 4;

  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          ElectricalConstants.kDriveMotorPorts[kFrontLeftIndex],
          ElectricalConstants.kSteerMotorPorts[kFrontLeftIndex],
          ElectricalConstants.kSteerEncoderPorts[kFrontLeftIndex],
          kSteerEncoderOffsets[kFrontLeftIndex]
          );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          ElectricalConstants.kDriveMotorPorts[kFrontRightIndex],
          ElectricalConstants.kSteerMotorPorts[kFrontRightIndex],
          ElectricalConstants.kSteerEncoderPorts[kFrontRightIndex],
          kSteerEncoderOffsets[kFrontRightIndex]
          );

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          ElectricalConstants.kDriveMotorPorts[kRearLeftIndex],
          ElectricalConstants.kSteerMotorPorts[kRearLeftIndex],
          ElectricalConstants.kSteerEncoderPorts[kRearLeftIndex],
          kSteerEncoderOffsets[kRearLeftIndex]
          );

  private final SwerveModule m_rearRight =
      new SwerveModule(
          ElectricalConstants.kDriveMotorPorts[kRearRightIndex],
          ElectricalConstants.kSteerMotorPorts[kRearRightIndex],
          ElectricalConstants.kSteerEncoderPorts[kRearRightIndex],
          kSteerEncoderOffsets[kRearRightIndex]
          );

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

  // The gyro sensor
  private final AHRS m_ahrs = new AHRS();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
    
  public SwerveDrive() {

    // Zero the gyro.
    m_ahrs.zeroYaw();
    m_odometry = new SwerveDriveOdometry(kDriveKinematics, getHeading(), getModulePositions());

    for (SwerveModule module: modules) {
      module.resetDistance();
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeading(),
        getModulePositions());
    
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    
    // SmartDashboard.putNumber("FrontLeft State Velocity", modules[0].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("FrontLeft State Angle", modules[0].getState().angle.getDegrees());

    // SmartDashboard.putNumber("FrontRight Velocity", modules[1].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("FrontRight Angle", modules[1].getState().angle.getDegrees());

    // SmartDashboard.putNumber("RearLeft Velocity", modules[2].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("RearLeft Angle", modules[2].getState().angle.getDegrees());

    // SmartDashboard.putNumber("RearRight Velocity", modules[3].getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("RearRight Angle", modules[3].getState().angle.getDegrees());
    
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("currentAngle", getPose().getRotation().getRadians());
    // SmartDashboard.putNumber("targetPoseAngle", m_targetPose.getRotation().getRadians());

    // SmartDashboard.putNumber("FronLeft Turning CANcoder Mag Offset", modules[0].getTurnCANcoder().configGetMagnetOffset());
    // SmartDashboard.putNumber("FronLeft Turning CANcoder Abs Position", modules[0].getTurnCANcoder().getAbsolutePosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot with given velocities.
   *
   * @param speeds ChassisSpeeds object with the desired chassis speeds [m/s and rad/s].
   */
  public void joystickDrive(double joystickDrive, 
                            double joystickStrafe, 
                            double joystickRotate,
                            boolean fieldRelative) {
    ChassisSpeeds speeds = fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(
                          joystickDrive * kMaxTranslationalVelocity,
                          joystickStrafe * kMaxTranslationalVelocity,
                          joystickRotate * kMaxRotationalVelocity,
                          getHeading()
        ) :
        new ChassisSpeeds(joystickDrive * kMaxTranslationalVelocity,
                          joystickStrafe * kMaxTranslationalVelocity,
                          joystickRotate * kMaxRotationalVelocity);
    SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(speeds);
    
    double scale = Math.max(Math.hypot(joystickDrive, joystickStrafe), joystickRotate);

    // Identify fastest motor's speed.                                      
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : states) {
      double speed = Math.abs(moduleState.speedMetersPerSecond);
      realMaxSpeed = Math.max(realMaxSpeed, speed);
    }
    
    if (realMaxSpeed != 0.0) {
        for (SwerveModuleState moduleState : states) {
          moduleState.speedMetersPerSecond *= (scale * kMaxVelocity / realMaxSpeed);
        }
    }
           
    for (int i = 0; i < 4; ++i) {
      modules[i].setDesiredState(states[i]);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds);
    for (int i = 0; i < 4; ++i) {
      modules[i].setDesiredState(moduleStates[i]);
    }
  }

  public void brake() {
    for (SwerveModule module : modules) {
      module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[kNumSwerveModules];

    for (int i = 0; i < kNumSwerveModules; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[kNumSwerveModules];

    for (int i = 0; i < kNumSwerveModules; i++) {
      positions[i] = modules[i].getPosition();
    }

    return positions;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (SwerveModule module: modules) {
      module.resetDistance();
    }
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_ahrs.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return m_ahrs.getRotation2d();
  }

  public void syncEncoders() {
    for (SwerveModule module : modules) {
      module.syncEncoders();
    }
  }
}