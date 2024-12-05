package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;

public class ExtendSlideCommand extends CommandBase {

    // Storage
    private LinearSlides linearSlides;
    private double targetExtensionLengthInches;

    /**
     * Creates a new ExtendSlideCommand command with the given information.
     *
     * @param linearSlides An instance of the robot's linear sides..
     * @param targetExtensionLengthInches How far the linear slides wil be extended, in inches.
     */
    public ExtendSlideCommand(LinearSlides linearSlides, double targetExtensionLengthInches) {
        this.linearSlides = linearSlides;
        this.targetExtensionLengthInches = targetExtensionLengthInches;
    }

    @Override
    public void init() {

        // Tell the linear slides to target the specified location.
        this.linearSlides.setTargetLengthInches(this.targetExtensionLengthInches);

        // Let the linear slides preform autonomous actions.
        this.linearSlides.toggleAutonomousControl(true);
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

    }

    @Override
    public boolean isFinished() {
        return this.linearSlides.atTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
