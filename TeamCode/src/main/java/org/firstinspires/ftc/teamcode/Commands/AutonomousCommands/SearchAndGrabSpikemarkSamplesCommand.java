package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class SearchAndGrabSpikemarkSamplesCommand extends CommandBase {

    // Settings
    private Pose2d redTwoSearchPosition;
    private Pose2d redOneSearchPosition;

    private Pose2d blueTwoSearchPosition;
    private Pose2d blueOneSearchPosition;

    // Subsystems
    private Drivetrain drivetrain;
    private ComputerVision computerVision;
    private Arm arm;
    private LinearSlides linearSlides;
    private Manipulator manipulator;

    // Storage
    private AutonomousRoutine autonomousRoutine;
    private Pose2d searchLocation;
    // TODO: Set sample finder stuff.
    private Pose2d targetSamplePosition;

    // Flags
    private boolean isFinished = false;
    private boolean slidesFullyRetracted = false;
    private boolean armAtGroundLevel = false;
    private boolean atSearchLocation = false;
    private boolean foundSampleTarget = false;
    private boolean atSampleGrabLocation = false;

    /**
     * Create a new SearchAndGrabSpikemarkSamplesCommand instance using the given subsystems for the given auto.
     *
     * @param drivetrain An instance of the robot's drivetrain.
     * @param computerVision An instance of the robot's computer vision.
     * @param manipulator An instance of the robot's manipulator.
     * @param alliance The alliance this robot is on.
     * @param autonomousRoutine The autonomous routine this robot is running.
     */
    public SearchAndGrabSpikemarkSamplesCommand(Drivetrain drivetrain, ComputerVision computerVision, Arm arm, LinearSlides linearSLides, Manipulator manipulator, AutonomousRoutine autonomousRoutine) {

        // Store the necessary subsystems for this command's usage.
        this.drivetrain = drivetrain;
        this.computerVision = computerVision;
        this.arm = arm;
        this.linearSlides = linearSLides;
        this.manipulator = manipulator;

        // Store the autonomous information to determine how this command is run.
        this.autonomousRoutine = autonomousRoutine;

        // Configure autonomous specific values.
        switch (this.autonomousRoutine) {
            case RED_TWO:
                this.searchLocation = redTwoSearchPosition;
                break;
            case RED_ONE:
                this.searchLocation = redOneSearchPosition;
                break;
            case BLUE_TWO:
                this.searchLocation = blueTwoSearchPosition;
                break;
            case BLUE_ONE:
                this.searchLocation = blueOneSearchPosition;
                break;
        }
    }

    @Override
    public void init() {

        // Set the target position for all mechanisms.
        this.drivetrain.setTargetPosition(this.searchLocation);
        this.arm.setTargetArmPositionRadians(this.arm.getStartingAngleRadians());
        this.linearSlides.setTargetLengthInches(0);

        // Toggle autonomous control of all mechanisms.
        this.drivetrain.toggleAutonomousControl(true);
        this.arm.toggleAutonomousControl(true);
        this.linearSlides.toggleAutonomousControl(true);

        // Open the claw and lower the wrist to be level with the ground.
        this.manipulator.setWristPosition(0); // TODO: TUNE wrist angle to be level with ground at base.
        this.manipulator.openClaw();

        // Clear old data.
        this.targetSamplePosition = null;

        // Toggle all flags.
        this.isFinished = false;
        this.slidesFullyRetracted = false;
        this.armAtGroundLevel = false;
        this.atSearchLocation = false;
        this.foundSampleTarget = false;
        this.atSampleGrabLocation = false;
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // If we aren't at our desired state yet, update the flags and wait to be at the desired state.
        if (!(atSearchLocation && armAtGroundLevel && slidesFullyRetracted)) {
            updateSearchLocationFlags();
            return;
        }

        // If a sample target hasn't bee found, then get the closest sample. If there aren't any samples that can be see, wait until we can see one.
        if (!foundSampleTarget) {
            Pose2d samplePosition = getSampleTarget(); // TODO: Switch to sample object. Preform necessary actions to improve performance.
            if (samplePosition == null) {
                return;
            } else {
                //TODO: Do magic to translate to robot origin position.
                this.targetSamplePosition = samplePosition;
            }

            this.drivetrain.setTargetPosition(this.targetSamplePosition); // TODO: Adjust based on in-match performance.
        }

        // If the robot isn't yet at the location where it needs to grab samples, then update necessary flags
        // and wait until it is at the grabbing location.
        if (!atSampleGrabLocation) {
            updateGrabLocationFlags();
            return;
        }

        // Close the manipulator claw around the sample.
        this.manipulator.closeClaw();

        // TODO: Determine whether or not we can wait for it to be fully closed or if we can set a timer/delay.

        this.isFinished = true;
    }

    /**
     * Updates the flags determining whether or not we are at the target position.
     */
    private void updateSearchLocationFlags() {
        atSearchLocation = this.drivetrain.atTargetPosition();
        armAtGroundLevel = this.arm.atTargetPosition();
        slidesFullyRetracted = this.linearSlides.atTargetPosition();
    }

    /**
     * Find and return the position of the desired colored sample to the robot.
     *
     * @return Return the position of the desired colored sample to the robot.
     */
    private Pose2d getSampleTarget() {
        // TODO: Write code to find the closest, most recent sample. Maybe remove this sample from the list of seen samples.
        return null;
    }

    /**
     * Updates all flags related to whether or not the robot is at the correct position to grab the located sample.
     */
    private void updateGrabLocationFlags() {
        this.atSampleGrabLocation = this.drivetrain.atTargetPosition();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
