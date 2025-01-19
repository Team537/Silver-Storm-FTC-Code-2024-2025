package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;

public class AdjustManipulatorCommand extends CommandBase {

    // Storage
    private Manipulator manipulator;
    private double targetWristPosition;
    private double targetClawPosition = -1;

    /**
     * Creates a new AdjustManipulatorCommand with the given information.
     *
     * @param manipulator An instance of the robot's manipulator.
     * @param wristPosition The position the robot's wrist will be moved to (0-1 range)
     */
    public AdjustManipulatorCommand(Manipulator manipulator, double wristPosition) {
        this.manipulator = manipulator;
        this.targetWristPosition = wristPosition;
    }

    /**
     * Creates a new AdjustManipulatorCommand with the given information.
     *
     * @param manipulator An instance of the robot's manipulator.
     * @param wristPosition The position the robot's wrist will be moved to (0-1 range)
     * @param clawPosition The position the claw will be moved to.
     */
    public AdjustManipulatorCommand(Manipulator manipulator, double wristPosition, double clawPosition) {
        this.manipulator = manipulator;
        this.targetWristPosition = wristPosition;
        this.targetClawPosition = clawPosition;
    }
    @Override
    public void init() {

        // Move the wrist to the specified location.
        this.manipulator.setWristPosition(targetWristPosition);

        // If told to do so, open the claw to the given position.
        if (this.targetClawPosition != -1) {
            this.manipulator.setClawPosition(this.targetClawPosition);
        }
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

    }

    // TODO: Check if position changes instantly.
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
