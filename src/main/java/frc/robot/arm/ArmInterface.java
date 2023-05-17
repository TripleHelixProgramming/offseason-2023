package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants.Setpoint;

public interface ArmInterface extends Subsystem {
    /**
     * @return whether the intake has a cube or not
     */
    public boolean hasCube();
    
    /**
     * Command to intake cubes
     */
    public Command intake();

    /**
     * Command to intake cubes until the limit switch is hit
     */
    public Command intakeUntilCube();

    /**
     * 
     * @return
     */
    public Command stow();

    /**
     * Command to score a cube/bloop if in intake position
     */
    public Command score();

    /**
     * Sets target arm position
     * @param position
     */
    public void setTargetSetpoint(Setpoint setpoint);

    public void syncEncoders();

}
