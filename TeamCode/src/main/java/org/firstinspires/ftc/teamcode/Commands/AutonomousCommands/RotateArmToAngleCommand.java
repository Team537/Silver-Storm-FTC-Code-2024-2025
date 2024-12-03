package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmPositions;

public class RotateArmToAngleCommand extends CommandBase {

    // Storage
    private Arm arm;
    private double armTargetAngle;

    /**
     * Creates a new RotateArmToAngleCommand command to rotate the robot's arm to the given angle.
     *
     * @param arm An instance of the robot's arm.
     * @param angleDegrees The angle you wish to rotate the arm to, in degrees.
     */
    public RotateArmToAngleCommand(Arm arm,double angleDegrees) {
        this.arm = arm;
        this.armTargetAngle = Math.toRadians(angleDegrees);
    }

    /**
     * Creates a new RotateArmToAngleCommand command to rotate the robot's arm to the given angle.
     *
     * @param arm An instance of the robot's arm.
     * @param armPosition The position you wish to rotate the arm to, as an ArmPosition..
     */
    public RotateArmToAngleCommand(Arm arm, ArmPositions armPosition) {
        this.arm = arm;
        this.armTargetAngle = armPosition.getArmAngleRadians();
    }

    @Override
    public void init() {

        // Set the target position of the arm.
        this.arm.setTargetArmPositionRadians(this.armTargetAngle);

        // Allow for the arm to be autonomously controlled.
        this.arm.toggleAutonomousControl(true);
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

    }

    @Override
    public boolean isFinished() {
        return this.arm.atTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
