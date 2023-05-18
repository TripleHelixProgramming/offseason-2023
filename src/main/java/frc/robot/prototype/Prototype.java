// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.prototype;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Prototype extends SubsystemBase {
  private CANSparkMax motor;

  public Prototype() {
    motor = new CANSparkMax(14, MotorType.kBrushless);
  }

  public Command shoot() {
    return Commands.run(() -> motor.setVoltage(12), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> motor.setVoltage(0), this);
  }
}
