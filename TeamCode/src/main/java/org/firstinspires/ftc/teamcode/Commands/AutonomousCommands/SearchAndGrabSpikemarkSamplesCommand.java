package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Sample;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Constants;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

import java.util.List;

public class SearchAndGrabSpikemarkSamplesCommand extends CommandBase {

    // Settings
    private Pose2d redOneSearchPosition = new Pose2d(1.202903, 0.739140, new Rotation2d(2.35619, 0));
    private Pose2d blueOneSearchPosition = new Pose2d(-1.202903, 0.739140);

    private Pose2d redThirdSpikemarkPose = new Pose2d(0.660832, 1.779);
    private Pose2d blueThirdSpikemarkPose = new Pose2d(-0.660832, 1.779);

    private double manipulatorCloseTimeSeconds = 1.25;

    // Subsystems
    private Drivetrain drivetrain;
    private ComputerVision computerVision;
    private Arm arm;
    private LinearSlides linearSlides;
    private Manipulator manipulator;

    // Storage
    private AutonomousRoutine autonomousRoutine;
    private Pose2d searchLocation;
    private Pose2d thirdSpikemarkPose;
    private Pose2d targetSamplePosition;
    private ElapsedTime manipulatorTimer = new ElapsedTime();
    private double rotationalOffsetSampleGrab = 0;

    // Flags
    private boolean isFinished = false;
    private boolean slidesFullyRetracted = false;
    private boolean armAtGroundLevel = false;
    private boolean atSearchLocation = false;
    private boolean foundSampleTarget = false;
    private boolean atSampleGrabLocation = false;
    private boolean timerRestFlag = false;

    /**
     * Create a new SearchAndGrabSpikemarkSamplesCommand instance using the given subsystems for the given auto.
     *
     * @param drivetrain An instance of the robot's drivetrain.
     * @param computerVision An instance of the robot's computer vision.
     * @param autonomousRoutine The autonomous routine this robot is running.
     */
    public SearchAndGrabSpikemarkSamplesCommand(Drivetrain drivetrain, ComputerVision computerVision, Arm arm, AutonomousRoutine autonomousRoutine) {

        // Store the necessary subsystems for this command's usage.
        this.drivetrain = drivetrain;
        this.computerVision = computerVision;
        this.arm = arm;
        this.linearSlides = this.arm.getLinearSlides();
        this.manipulator = this.arm.getManipulator();

        // Store the autonomous information to determine how this command is run.
        this.autonomousRoutine = autonomousRoutine;

        // Configure autonomous specific values.
        switch (this.autonomousRoutine) {
            case RED_ONE:
                this.searchLocation = redOneSearchPosition;
                this.thirdSpikemarkPose = redThirdSpikemarkPose;
                this.rotationalOffsetSampleGrab = -Math.PI;
                break;
            case BLUE_ONE:
                this.searchLocation = blueOneSearchPosition;
                this.thirdSpikemarkPose = blueThirdSpikemarkPose;
                this.rotationalOffsetSampleGrab = 0;
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
        this.manipulator.setWristPosition(1);
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
        this.timerRestFlag = false;
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

            // Get the closest sample target.
            Pose2d samplePosition = getSampleTarget();

            // If there isn't a closest sample, then try again.
            if (samplePosition == null) {
                return;
            }

            // Go to the target position.
            this.targetSamplePosition = samplePosition;
            this.foundSampleTarget = true;
            this.drivetrain.setTargetPosition(this.targetSamplePosition);
        }

        // If the robot isn't yet at the location where it needs to grab samples, then update necessary flags
        // and wait until it is at the grabbing location.
        if (!atSampleGrabLocation) {
            updateGrabLocationFlags();
            return;
        }

        // Close the manipulator claw around the sample.
        this.manipulator.closeClaw();

        // If the close timer hasn't bee reset, then reset it so that we can stop the command once the manipulator has fully closed.
        if (!timerRestFlag) {
            timerRestFlag = true;
            this.manipulatorTimer.reset();
        }

        // If the manipulator hasn't finished closing, return.
        if (this.manipulatorTimer.getElapsedTime(TimeUnit.SECOND) < manipulatorCloseTimeSeconds) {
            return;
        }

        // The command has finished running.
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
        List<Sample> detectedObjects = this.computerVision.getDetectedObjects();

        // Get the robot's current position.
        Pose2d robotPosition = this.drivetrain.getRobotPosition();

        // Storage
        double closestObjectDistance = 5;
        Sample closestObject = null;

        // Loop through all of the detected objects and store the closest one.
        for (Sample detectedObject : detectedObjects) {
            double distanceToObject = Pose2d.getAbsolutePositionalDistanceTo(robotPosition, detectedObject.getFieldPosition());

            // If the object is closer than the current closest object, set this object as the closest object.
            if (distanceToObject < closestObjectDistance) {
                closestObject = detectedObject;
                closestObjectDistance = distanceToObject;
            }
        }

        if (closestObject == null) {
            return null;
        }

        // Get the detected object's position.
        Pose2d detectedObjectFieldPosition = closestObject.getFieldPosition().clone();

        // If the objet is the last one on the last spike mark, then grab it differently.
        if (Pose2d.getAbsolutePositionalDistanceTo(thirdSpikemarkPose, detectedObjectFieldPosition) <= 0.1778) {
            Pose2d grabPositionOffset = Constants.AutoConstants.GRAB_OFFSET_ROBOT_CENTRIC_ROTATED.clone();
            grabPositionOffset = this.drivetrain.getCoordinateSystem().robotSpaceToFieldSpace(grabPositionOffset);
            Pose2d robotTargetPosition = new Pose2d(detectedObjectFieldPosition.getX(), detectedObjectFieldPosition.getZ(), new Rotation2d(1.5708, 0));
            grabPositionOffset.addValues(grabPositionOffset);
            detectedObjectFieldPosition.setYaw(this.rotationalOffsetSampleGrab);
            return robotTargetPosition;
        }

        // Offset the sample grab position such that the robot can grab it.
        Pose2d grabPositionOffset = Constants.AutoConstants.GRAB_OFFSET_ROBOT_CENTRIC.clone();
        grabPositionOffset = this.drivetrain.getCoordinateSystem().robotSpaceToFieldSpace(grabPositionOffset);
        detectedObjectFieldPosition.addValues(grabPositionOffset);
        detectedObjectFieldPosition.setYaw(this.rotationalOffsetSampleGrab);

        // Return the detected object's position.
        return detectedObjectFieldPosition;
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
