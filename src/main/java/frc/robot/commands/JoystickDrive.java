// Copyright (c) Triple Helix Robotics

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import java.lang.Math;
public class JoystickDrive extends CommandBase {
    private final SwerveDrive drivetrain;
    private DoubleSupplier x, y, theta;

    public JoystickDrive(SwerveDrive drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta){
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.theta = theta;
        addRequirements(drivetrain);
    }
 
    @Override
    public void initialize() {
    }
            
    @Override
    public void execute() {
        double doubleX = Math.abs(x.getAsDouble()) < 0.05 ? 0 : x.getAsDouble();
        double doubleY = Math.abs(y.getAsDouble()) < 0.05 ? 0 : y.getAsDouble();
        double doubleTheta = Math.abs(theta.getAsDouble()) < 0.05 ? 0 : theta.getAsDouble();

        double vx = doubleX * DriveConstants.kMaxTranslationalVelocity;
        double vy = doubleY * DriveConstants.kMaxTranslationalVelocity;
        double omega = doubleTheta * DriveConstants.kMaxRotationalVelocity;

        SmartDashboard.putNumber("vx", vx);
        SmartDashboard.putNumber("vy", vy);
        SmartDashboard.putNumber("omega", omega);

        ChassisSpeeds velocity = DriveConstants.kFieldRelative
                                 ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading())
                                 : new ChassisSpeeds(vx, vy, omega);

        ChassisSpeeds percent = new ChassisSpeeds(doubleX, doubleY, doubleTheta);

        drivetrain.drive(velocity, percent);
    }
}