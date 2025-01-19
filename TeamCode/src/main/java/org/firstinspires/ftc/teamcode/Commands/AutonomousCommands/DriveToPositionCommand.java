package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class DriveToPositionCommand extends CommandBase {

    // Storage
    private Drivetrain drivetrain;
    private Pose2d targetPosition;

    /**
     * Creates a new DriveToPositionCommand with the given parameters.
     *
     * @param drivetrain The drivetrain subsystem that will be driven to a position.
     * @param targetPosition The position the given drivetrain will be driven to.
     */
    public DriveToPositionCommand(Drivetrain drivetrain, Pose2d targetPosition) {
        this.drivetrain = drivetrain;
        this.targetPosition = targetPosition;
    }

    @Override
    public void init() {

        // Set the robot's target position.
        this.drivetrain.setTargetPosition(this.targetPosition);

        // Make it possible to autonomously drive the robot.
        this.drivetrain.toggleAutonomousControl(true);
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

    }

    @Override
    public boolean isFinished() {
        return drivetrain.atTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {

        // Stop the robot from autonomously moving.
        drivetrain.toggleAutonomousControl(false);
    }
}
