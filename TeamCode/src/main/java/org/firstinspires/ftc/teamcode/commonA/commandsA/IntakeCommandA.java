package org.firstinspires.ftc.teamcode.commonA.commandsA;// IntakeCommand.java
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
// Import the correct subsystem file name


public class IntakeCommandA extends CommandBase {
    private final IntakeA intake; // Renamed variable
    private final IntakeA.IntakeState targetState; // Renamed variable and adjusted type import

    // Constructor requires the subsystem instance and the desired state
    public IntakeCommandA(IntakeA subsystem, IntakeA.IntakeState targetState) {
        this.intake = subsystem; // Use 'this' to distinguish from parameter
        this.targetState = targetState; // Use 'this' to distinguish from parameter
        // This command requires the intake subsystem to run
        addRequirements(intake); // Renamed variable usage
    }

    @Override
    public void initialize() {
        // When the command starts, update the subsystem's state
        intake.setIntakeState(targetState); // Renamed variable usage
    }

    @Override
    public boolean isFinished() {
        // The command finishes instantly; the 'periodic' method handles continuous power
        return true;
    }
}
