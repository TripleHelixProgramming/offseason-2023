// Copyright (c) Triple Helix Robotics

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  public static final class ElectricalConstants {
      public static final ModuleType pdpType = ModuleType.kCTRE;
      public static final int pdpPort = 0;

      // -----------------------------------------   {fl  fr  rl  rr}
      public static final int[] kSteerEncoderPorts = { 0,  1,  2,  3};
      public static final int[] kDriveMotorPorts   = {10, 12, 22, 24};
      public static final int[] kSteerMotorPorts   = {11, 13, 23, 25};

      // CAN IDs for intake and arm 
      public static final int kIntakeLeaderPort = 15;
      public static final int kIntakeFollowerPort = 16;
      public static final int kArmWristPort = 17;

      public static final int kWristEncoderPort = 5; // TODO this
  }

  public static final class DriveConstants {

    // Define the conventional order of our modules when putting them into arrays
    public static final int kFrontLeftIndex  = 0;
    public static final int kFrontRightIndex = 1;
    public static final int kRearLeftIndex   = 2;
    public static final int kRearRightIndex  = 3;
    
    // Rotational transformation between absolute encoder "north" and wheel "forward"
    public static final Rotation2d[] kSteerEncoderOffsets = {Rotation2d.fromDegrees(-64.2679), // fl
                                                             Rotation2d.fromDegrees(44.23),    // fr
                                                             Rotation2d.fromDegrees(15.19),    // rl
                                                             Rotation2d.fromDegrees(99.36)};   // rr

    // Distance between centers of right and left wheels on robot
    // Units: meters
    public static final double kTrackWidth = 0.5715; // 22.5 in
    
    // Distance between front and back wheels on robot
    // Units: meters
    public static final double kWheelBase = 0.6223; // 24.5 in

    // Units: meters per second
    public static final double kMaxTranslationalVelocity = 1.0; //max 4.5

    // Units: radians per second
    public static final double kMaxRotationalVelocity = 2.5; //max 5.0

    //The locations for the modules must be relative to the center of the robot. 
    // Positive x values represent moving toward the front of the robot 
    // Positive y values represent moving toward the left of the robot.
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d( kWheelBase / 2.0,  kTrackWidth / 2.0),   // front left
            new Translation2d( kWheelBase / 2.0, -kTrackWidth / 2.0),  // front right
            new Translation2d(-kWheelBase / 2.0,  kTrackWidth / 2.0),  // rear left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)  // rear right
            );

    public static final boolean kGyroReversed = false;

    // Is the joystick drive in field relative mode.
    public static final boolean kFieldRelative = true;
  }

  public static final class ModuleConstants {

    public static final double kDriveP  = 0.025;
    public static final double kDriveI  = 0.0;
    public static final double kDriveD  = 0.0;
    public static final double kDriveFF = 0.25;

    public static final double kSteerP  = 0.0075;
    public static final double kSteerI  = 0.0;
    public static final double kSteerD  = 0.0;
    public static final double kSteerFF = 0.0;

    // Unit: meters per second
    public static final double kMaxVelocity = 3.0;

    // Unit: meters
    public static final double kWheelDiameter = 0.0762; // 3 in

    // Gear reduction (unitless) between the drive motor and the wheel
    public static final double kDriveGearRatio = 4.71;

    // The drive encoder reports in RPM by default. Calculate the conversion factor
    // to make it report in meters per second.
    public static final double kDriveConversionFactor = (kWheelDiameter * Math.PI) / kDriveGearRatio;

    // Gear reduction (unitless) between the steering motor and the module azimuth
    // Stage 1 - REV Ultraplanetary nominal "4:1" stage, actual ratio 29:84
    // Stage 2 - REV Ultraplanetary nominal "3:1" stage, actual ratio 21:76
    // Stage 3 - 14:62
    public static final double kTurnPositionConversionFactor = 46.42;

    // Unit: volts
    public static final int kNominalVoltage = 12;

    // Unit: amps
    public static final int kDriveCurrentLimit = 60;
    public static final int kSteerCurrentLimit = 25;
  }

  public static final class ArmConstants {

    public static final double kWristP  = 0;
    public static final double kWristI  = 0;
    public static final double kWristD  = 0;
    public static final double kWristFF = 0;
    
    public static final double kIntakeP  = 0;
    public static final double kIntakeI  = 0;
    public static final double kIntakeD  = 0;
    public static final double kIntakeFF = 0;
    
    public static final int kIntakeNominalVoltage = 12;
    public static final int kWristNominalVoltage  = 12;

    public static final int kIntakeCurrentLimit = 20;
    public static final int kWristCurrentLimit  = 20;

    public static final double kArmIntakingPosition = 1;
    public static final double kIntakingVoltage     = 1;

    public enum Setpoint {
      // TODO: change these values to reflect actual setpoints/make robot work
      LOW(1, 2),
      MID(2, 3),
      HIGH(3, 3);

      private final double armPosition;
      private final double intakeSpeed;
      
      Setpoint(double armPosition, double intakeSpeed) {
        this.armPosition = armPosition;
        this.intakeSpeed = intakeSpeed;
      }
      
      public double getArmPosition() {
        return armPosition;
      }
      
      public double getIntakeVoltage() {
        return intakeSpeed;
      }
    }
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Xbox Joystick Axis
    public static final int kXboxLeftXAxis = 0;
    public static final int kXboxLeftYAxis = 1;
    public static final int kXboxLeftTrigger = 2;
    public static final int kXboxRightTripper = 3;
    public static final int kXboxRightXAxis = 4;
    public static final int kXboxRightYAxis = 5;

    // Xbox Button Bindings
    public static final int kXboxA = 1;
    public static final int kXboxB = 2;
    public static final int kXboxX = 3;
    public static final int kXboxY = 4;
    public static final int kXboxLB = 5;
    public static final int kXboxRB = 6;
    public static final int kXboxView = 7;
    public static final int kXboxMenuT = 8;
    public static final int kXboxLeftStickButton = 9;
    public static final int kXboxRightStickButton = 10;

    // RadioMaster Zorro Joystick Axis
    public static final int kZorroLeftXAxis = 0;
    public static final int kZorroLeftYAxis = 1;
    public static final int kZorroLeftDial = 2;
    public static final int kZorroRightDial = 3;
    public static final int kZorroRightXAxis = 4;
    public static final int kZorroRightYAxis = 5;

    // RadioMaster Zorro Button Bindings
    public static final int kZorroBDown = 1;
    public static final int kZorroBMid = 2;
    public static final int kZorroBUp = 3;
    public static final int kZorroEDown = 4;
    public static final int kZorroEUp = 5;
    public static final int kZorroAIn = 6;
    public static final int kZorroGIn = 7;
    public static final int kZorroCDown = 8;
    public static final int kZorroCMid = 9;
    public static final int kZorroCUp = 10;
    public static final int kZorroFDown = 11;
    public static final int kZorroFUp = 12;
    public static final int kZorroDIn = 13;
    public static final int kZorroHIn = 14;
  }
}