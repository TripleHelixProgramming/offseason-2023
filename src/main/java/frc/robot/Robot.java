// Copyright (c) Triple Helix Robotics

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.ArmInterface;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.JoystickDrive;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Robot extends TimedRobot {

  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //           IO, global sensors, subsystems, and global commands           //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  private Joystick driver = new Joystick(0);
  
  private SwerveDrive swerve;
  private ArmInterface arm;

  private Command autonomousCommand = new PrintCommand("default auto command! please override me!");
  
  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //                      Robot Scheduling and Stuff                         //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  @Override
  public void robotInit() {
    configureButtonBindings();

    // TODO: change these to actually match real axis ports
    DoubleSupplier xAxis = () -> driver.getRawAxis(0);
    DoubleSupplier yAxis = () -> driver.getRawAxis(1);
    DoubleSupplier thetaAxis = () -> driver.getRawAxis(2);

    swerve.setDefaultCommand(new JoystickDrive(swerve, xAxis, yAxis, thetaAxis));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    arm.syncEncoders();
    swerve.syncEncoders();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //                      Old Robot Container Stuffs                         //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  private void configureButtonBindings() {
    Trigger leftBumper = new JoystickButton(driver, OIConstants.kXboxLB);
    Trigger rightBumper = new JoystickButton(driver, OIConstants.kXboxRB);

    leftBumper.onTrue(arm.score()).onFalse(arm.stow());

    rightBumper.onTrue(arm.intake().andThen(waitUntil(arm::hasCube)).andThen(arm.stow()))
               .onFalse(arm.stow());
  }
}