package org.firstinspires.ftc.teamcode.Commands.AutonomousCommands;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandResult;
import org.firstinspires.ftc.teamcode.Commands.Directives.NextCommandDirective;
import org.firstinspires.ftc.teamcode.Commands.Directives.TerminateDirective;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Sample;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Constants;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

import java.util.List;

public class CreateSpecimenCommand extends CommandBase {

    // Settings
    private Pose2d redDropoffLocation = new Pose2d(1.038032, 1.409193);
    private Pose2d redWaitLocation = new Pose2d(1.143, 1.409193);

    private Pose2d blueDropoffLocation = new Pose2d(-1.038032, 1.409193, new Rotation2d(Math.toRadians(-180), 0));
    private Pose2d blueWaitLocation = new Pose2d(-1.143, 1.409193, new Rotation2d(Math.toRadians(-180), 0));

    private double specimenCreationTime = 5;
    private double manipulatorOpenTime = .75;
    private double manipulatorCloseTime = 0;

    // Storage
    private ElapsedTime manipulatorOpenTimer = new ElapsedTime();
    private ElapsedTime specimenCreationTimer = new ElapsedTime();
    private ElapsedTime manipulatorCloseTimer = new ElapsedTime();

    private AutonomousRoutine autonomousRoutine;
    private Pose2d dropoffLocation;
    private Pose2d waitLocation;

    private double timesCommandUsed = 0;
    private double timeCuttoff;

    private String scoreCommandID;

    // Subsystems
    private Drivetrain drivetrain;
    private ComputerVision computerVision;
    private Arm arm;
    private LinearSlides linearSlides;
    private Manipulator manipulator;

    // Flags
    private boolean isFinished = false;
    private boolean atDropoffLocation = false;
    private boolean armAtGroundLevel = false;
    private boolean linearSlidesFullyRetracted = false;
    private boolean manipulatorOpenTimerReset = false;
    private boolean waitTimerReset = false;
    private boolean reenabledCameraData = false;
    private boolean hasSampleTarget = false;
    private boolean atGrabLocation = false;
    private boolean manipulatorClosedTimerReset = false;

    /**
     * Create a new CreateSpecimenCommand instance using the given subsystems for the given auto.
     *
     * @param drivetrain An instance of the robot's drivetrain.
     * @param computerVision An instance of the robot's computer vision.
     * @param autonomousRoutine The autonomous routine this robot is running.
     */
    public CreateSpecimenCommand(Drivetrain drivetrain, ComputerVision computerVision, Arm arm, AutonomousRoutine autonomousRoutine) {

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
                this.dropoffLocation = redDropoffLocation;
                this.waitLocation = redWaitLocation;
                break;
            case BLUE_ONE:
                this.dropoffLocation = blueDropoffLocation;
                this.waitLocation = blueWaitLocation;
                break;
        }
    }

    @Override
    public void init() {

        // Configure all subsystems for optimal control.
        this.drivetrain.setTargetPosition(this.dropoffLocation);
        this.linearSlides.setTargetLengthInches(0);
        this.arm.setTargetPosition(this.arm.getStartingAngleRadians());
        this.manipulator.setWristPosition(1);

        // Toggle autonomous control of all mechanisms.
        this.drivetrain.toggleAutonomousControl(true);
        this.arm.toggleAutonomousControl(true);
        this.linearSlides.toggleAutonomousControl(true);

        // Reset all of the flags.
        this.isFinished = false;
        this.atDropoffLocation = false;
        this.armAtGroundLevel = false;
        this.linearSlidesFullyRetracted = false;
        this.manipulatorOpenTimerReset = false;
        this.waitTimerReset = false;
        this.hasSampleTarget = false;
        this.atGrabLocation = false;
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // If we aren't at our desired state yet, update the flags and wait to be at the desired state.
        if (!(atDropoffLocation && armAtGroundLevel && linearSlidesFullyRetracted)) {
            updateSearchLocationFlags();
            return;
        }

        // If the manipulator opening timer hasn't been reset yet, reset it.
        if (!manipulatorOpenTimerReset) {

            // Open the manipulator's claw.
            this.manipulator.openClaw();

            // Pause vision data to prevent erroneous data from harming results.
            this.computerVision.setPauseCapture(true);

            // Reset the timer and toggle the flag.
            this.manipulatorOpenTimer.reset();
            this.manipulatorOpenTimerReset = true;
        }

        // If the manipulator hasn't finished opening yet, then wait for it to open.
        if (manipulatorOpenTimer.getElapsedTime(TimeUnit.SECOND) < manipulatorOpenTime) {
            return;
        }

        // If the wait timer hasn't been reset, then reset it.
        if (!waitTimerReset) {

            // Drive to the waiting position.
            this.drivetrain.setTargetPosition(this.waitLocation);

            // Reset the specimen creation timer.
            this.specimenCreationTimer.reset();
            this.waitTimerReset = true;
        }

        // If we haven't waited long enough, then return.
        if (this.specimenCreationTimer.getElapsedTime(TimeUnit.SECOND) < specimenCreationTime) {
            return;
        }

        // If vision hasn't been enabled, enable it.
        if (!reenabledCameraData) {
            this.timeCuttoff = this.computerVision.getVisionTimeSeconds();
            this.computerVision.setPauseCapture(false);
            this.reenabledCameraData = true;
            return; // Return to give time to get new vision data.
        }

        // Attempt to find the specimen and grab it.
        if (!hasSampleTarget) {
            Pose2d targetSamplePosition = getSampleTarget();
            if (targetSamplePosition == null) {
                return;
            }

            // Drive to the sample.
            this.drivetrain.setTargetPosition(targetSamplePosition);
            this.hasSampleTarget = true;
        }

        // If we aren't at the grab location, wait until we are.
        if (!atGrabLocation) {
            updateSampleGrabFlag();
            return;
        }

        // If the manipulator closing timer hasn't been reset yet then reset it.
        if (!manipulatorClosedTimerReset) {

            // Close the claw.
            this.manipulator.closeClaw();

            // Reset the timer and toggle the flag.
            this.manipulatorCloseTimer.reset();
            this.manipulatorClosedTimerReset = true;
        }

        // If the claw hasn't closed yet, close it.
        if (this.manipulatorCloseTimer.getElapsedTime(TimeUnit.SECOND) < manipulatorCloseTime) {
            return;
        }

        // The command has finished running.
        this.isFinished = true;
    }


    /**
     * Updates the flags determining whether or not we are at the target position.
     */
    private void updateSearchLocationFlags() {
        this.atDropoffLocation = this.drivetrain.atTargetPosition();
        this.armAtGroundLevel = this.arm.atTargetPosition();
        this.linearSlidesFullyRetracted = this.linearSlides.atTargetPosition();
    }

    private void updateSampleGrabFlag() {
        this.drivetrain.atTargetPosition();
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

            // If the vision data is outdated, skip it.
            if (detectedObject.getDetectionTimeSeconds() < this.timeCuttoff) {
                continue;
            }

            // Get the distance to the object.
            double distanceToObject = Pose2d.getAbsolutePositionalDistanceTo(robotPosition, detectedObject.getFieldPosition());

            // If the object is closer than the current closest object, set this object as the closest object.
            if (distanceToObject < closestObjectDistance) {
                closestObject = detectedObject;
                closestObjectDistance = distanceToObject;
            }
        }

        // Return null if no sample was found.
        if (closestObject == null) {
            return null;
        }

        // Get the detected object's position.
        Pose2d detectedObjectFieldPosition = closestObject.getFieldPosition().clone();

        // Offset the sample grab position such that the robot can grab it.
        Pose2d grabPositionOffset = Constants.AutoConstants.GRAB_OFFSET_ROBOT_CENTRIC.clone();
        grabPositionOffset = this.drivetrain.getCoordinateSystem().robotSpaceToFieldSpace(grabPositionOffset);
        detectedObjectFieldPosition.addValues(grabPositionOffset);
        detectedObjectFieldPosition.setRotation(this.dropoffLocation.getRotation2d());

        // Return the detected object's position.
        return detectedObjectFieldPosition;
    }
    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    /**
     * Enables the robot to automatically score the specimen when acquired.
     *
     * @param scoreCommandID The ID of the scoring command.
     */
    public void setScoreCommandID(String scoreCommandID) {
        this.scoreCommandID = scoreCommandID;
    }

    @Override
    public CommandResult getCommandResult() {

        // If less than 3 specimens are scored,
        if (this.scoreCommandID == null) {
            return CommandResult.fromDirective(new NextCommandDirective(this.getID(), this.parentCommand));
        }

        // Run the scoring command.
        return CommandResult.fromCommandID(this.scoreCommandID);
    }
    @Override
    public void end(boolean interrupted) {

    }
}
