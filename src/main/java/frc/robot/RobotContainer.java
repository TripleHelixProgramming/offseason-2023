// Copyright (c) Triple Helix Robotics

package frc.robot;

import java.util.function.DoubleSupplier;

import static com.team2363.utilities.ControllerMap.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
  private final SwerveDrive drive = new SwerveDrive();
  private Joystick driver = new Joystick(0);

  public RobotContainer() {
    configureButtonBindings();

    DoubleSupplier xAxis = () -> driver.getRawAxis(X_BOX_RIGHT_STICK_Y);
    DoubleSupplier yAxis = () -> driver.getRawAxis(X_BOX_RIGHT_STICK_X);
    DoubleSupplier thetaAxis = () -> driver.getRawAxis(X_BOX_LEFT_STICK_X);

    drive.setDefaultCommand(new JoystickDrive(drive, xAxis, yAxis, thetaAxis));
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void syncEncoders() {
    drive.syncEncoders();
  }

  public void configureButtonBindings() {
    CommandScheduler.getInstance().clearButtons();

    new JoystickButton(driver, X_BOX_A).whenPressed(new InstantCommand(drive::zeroHeading));
  }
}
