
package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricalConstants;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase implements ArmInterface {
    private Setpoint setpoint;

    private DigitalInput limitSwitch;

    private CANSparkMax intakeLeader, intakeFollower, wrist;
    private DutyCycleEncoder absoluteEncoder;

    private SparkMaxPIDController intakeController, wristController;

    public Arm() {
        this.limitSwitch = new DigitalInput(0);
        
        this.intakeLeader = new CANSparkMax(ElectricalConstants.kIntakeLeaderPort, kBrushless);
        this.intakeFollower = new CANSparkMax(ElectricalConstants.kIntakeFollowerPort, kBrushless);

        this.wrist = new CANSparkMax(ElectricalConstants.kArmWristPort, kBrushless);
        this.absoluteEncoder = new DutyCycleEncoder(ElectricalConstants.kWristEncoderPort);

        this.wrist.enableVoltageCompensation(kWristNominalVoltage);
        this.intakeLeader.enableVoltageCompensation(kIntakeNominalVoltage);
        this.intakeFollower.enableVoltageCompensation(kIntakeNominalVoltage);

        this.wrist.setSmartCurrentLimit(kWristCurrentLimit);
        this.intakeLeader.setSmartCurrentLimit(kIntakeCurrentLimit);
        this.intakeFollower.setSmartCurrentLimit(kIntakeCurrentLimit);

        this.intakeFollower.follow(this.intakeLeader);

        this.wristController = wrist.getPIDController();
        this.intakeController = intakeLeader.getPIDController();

        this.wristController.setP(kWristP);
        this.wristController.setI(kWristI);
        this.wristController.setD(kWristD);
        this.wristController.setFF(kWristFF);

        this.intakeController.setP(kIntakeP);
        this.intakeController.setI(kIntakeI);
        this.intakeController.setD(kIntakeD);
        this.intakeController.setFF(kIntakeFF);

        syncEncoders();

        putPIDFSmartDash();
    }

    @Override
    public boolean hasCube() {
        return limitSwitch.get();
    }

    private Command waitUntilCube() {
        return waitUntil(this::hasCube);
    }

    @Override
    public void setTargetSetpoint(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public Command intake() {
        return this.setArmState(kArmIntakingPosition, kIntakingVoltage);
    }

    @Override
    public Command intakeUntilCube() {
        return this.intake().andThen(this.waitUntilCube(), this.stow());
    }

    @Override
    public Command stow() {
        return this.setArmState(setpoint.getArmPosition(), 0.0);
    }

    @Override
    public Command score() {
        return this.setArmState(setpoint.getArmPosition(), setpoint.getIntakeVoltage());
    }

    private Command setArmState(double armPosition, double intakeVoltage) {
        return this.runOnce(() -> {
            intakeController.setReference(intakeVoltage, ControlType.kVoltage);
            wristController.setReference(armPosition, ControlType.kPosition);
        });
    }

    private void putPIDFSmartDash() {
        SmartDashboard.putNumber("Arm/WristP", wristController.getP());
        SmartDashboard.putNumber("Arm/WristI", wristController.getI());
        SmartDashboard.putNumber("Arm/WristD", wristController.getD());
        SmartDashboard.putNumber("Arm/WristFF", wristController.getFF());

        SmartDashboard.putNumber("Arm/IntakeP", intakeController.getP());
        SmartDashboard.putNumber("Arm/IntakeI", intakeController.getI());
        SmartDashboard.putNumber("Arm/IntakeD", intakeController.getD());
        SmartDashboard.putNumber("Arm/IntakeFF", intakeController.getFF());
    }

    public void syncEncoders() {
        // TODO make sure we don't break the robot by syncing absolute and wrist encoders
        this.wrist.getEncoder().setPosition(this.absoluteEncoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        // Display telemetry here
        // TODO: add logging
        SmartDashboard.putBoolean("Arm/Beam break", hasCube());
        SmartDashboard.putNumber("Arm/Absolute encoder", absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm/Wrist encoder", wrist.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm/Wrist setpoint", setpoint.getArmPosition());
        SmartDashboard.putNumber("Arm/Intake setpoint", setpoint.getIntakeVoltage());

        double newWristP = SmartDashboard.getNumber("Arm/WristP", wristController.getP());
        double newWristI = SmartDashboard.getNumber("Arm/WristI", wristController.getI());
        double newWristD = SmartDashboard.getNumber("Arm/WristD", wristController.getD());
        double newWristFF = SmartDashboard.getNumber("Arm/WristFF", wristController.getFF());

        double newIntakeP = SmartDashboard.getNumber("Arm/IntakeP", intakeController.getP());
        double newIntakeI = SmartDashboard.getNumber("Arm/IntakeI", intakeController.getI());
        double newIntakeD = SmartDashboard.getNumber("Arm/IntakeD", intakeController.getD());
        double newIntakeFF = SmartDashboard.getNumber("Arm/IntakeFF", intakeController.getFF());

        if (newWristP != wristController.getP()) wristController.setP(newWristP);
        if (newWristI != wristController.getI()) wristController.setI(newWristI);
        if (newWristD != wristController.getD()) wristController.setD(newWristD);
        if (newWristFF != wristController.getFF()) wristController.setFF(newWristFF);

        if (newIntakeP != intakeController.getP()) intakeController.setP(newIntakeP);
        if (newIntakeI != intakeController.getI()) intakeController.setI(newIntakeI);
        if (newIntakeD != intakeController.getD()) intakeController.setD(newIntakeD);
        if (newIntakeFF != intakeController.getFF()) intakeController.setFF(newIntakeFF);
    }
}
