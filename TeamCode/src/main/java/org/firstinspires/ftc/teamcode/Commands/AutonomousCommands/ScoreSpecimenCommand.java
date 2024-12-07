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
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

public class ScoreSpecimenCommand extends CommandBase {

    // Settings
    private Pose2d redOneScoringLocation = new Pose2d(1.2944, 0.102998, new Rotation2d(Math.toRadians(-180), 0));
    private Pose2d blueOneScoringLocation = new Pose2d(1.2944, 0.102998, new Rotation2d(0, 0));
    private double subsequentUseOffsetMeters = 0.1016;

    private double armScoringAngleDegrees = 40;
    private double slideExtensionLengthInches = 10;
    private double wristPosition = .45;

    private double scoringWristPosition = 0;
    private double scoringSlideExtensionLength = 6;

    private double wristMoveTime = 1;
    private double slideMoveTime = 1;

    // Storage
    private ElapsedTime wristFlipTimer = new ElapsedTime();
    private ElapsedTime slideTimer = new ElapsedTime();

    private AutonomousRoutine autonomousRoutine;
    private Pose2d scoringLocation;
    private Pose2d actualScoringLocation;

    private double specimensScored = 0;

    // Subsystems
    private Drivetrain drivetrain;
    private Arm arm;
    private LinearSlides linearSlides;
    private Manipulator manipulator;

    // Flags
    private boolean isFinished = false;
    private boolean slidesFullyExtended = false;
    private boolean armAtPosition = false;
    private boolean atScoreingLocation = false;
    private boolean wristTimeReset = false;
    private boolean slideTimerReset = false;

    /**
     * Create a new ScoreSpecimenCommand instance using the given subsystems for the given auto.
     *
     * @param drivetrain An instance of the robot's drivetrain.
     * @param autonomousRoutine The autonomous routine this robot is running.
     */
    public ScoreSpecimenCommand(Drivetrain drivetrain, Arm arm, AutonomousRoutine autonomousRoutine) {

        // Store the necessary subsystems for this command's usage.
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.linearSlides = arm.getLinearSlides();
        this.manipulator = arm.getManipulator();

        // Store the autonomous information to determine how this command is run.
        this.autonomousRoutine = autonomousRoutine;

        // Configure autonomous specific values.
        switch (this.autonomousRoutine) {
            case RED_ONE:
                this.scoringLocation = redOneScoringLocation;
                break;
            case BLUE_ONE:
                this.scoringLocation = blueOneScoringLocation;
                break;
        }
    }

    @Override
    public void init() {

        // Update the scoring position to account for subsequent uses of this command.
        this.actualScoringLocation = this.scoringLocation.clone();

        this.actualScoringLocation.setZ(this.actualScoringLocation.getZ() - (specimensScored * subsequentUseOffsetMeters));

        // Set the target position for all mechanisms.
        this.drivetrain.setTargetPosition(this.actualScoringLocation);
        this.arm.setTargetArmPositionRadians(armScoringAngleDegrees);
        this.linearSlides.setTargetLengthInches(slideExtensionLengthInches);
        this.manipulator.setWristPosition(wristPosition);
        this.manipulator.closeClaw();

        // Toggle autonomous control of all mechanisms.
        this.drivetrain.toggleAutonomousControl(true);
        this.arm.toggleAutonomousControl(true);
        this.linearSlides.toggleAutonomousControl(true);

        // Reset flags.
        this.isFinished = false;
        this.slidesFullyExtended = false;
        this.armAtPosition = false;
        this.atScoreingLocation = false;
        this.wristTimeReset = false;
        this.slideTimerReset = false;
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // If we aren't at our desired state yet, update the flags and wait to be at the desired state.
        if (!(slidesFullyExtended && armAtPosition && atScoreingLocation)) {
            updateScoringLocationFlags();
            return;
        }

        // If the wrist timer hasn't been reset, then reset it.
        if (!wristTimeReset) {

            // Set the wrist to the desired position.
            this.manipulator.setWristPosition(scoringWristPosition);

            // Reset the timer and flip the flag.
            this.wristFlipTimer.reset();
            this.wristTimeReset = true;
        }

        // If the wrist isn't at its desired position yet, wait for such to be true.
        if (this.wristFlipTimer.getElapsedTime(TimeUnit.SECOND) < wristMoveTime) {
            return;
        }

        // If the slide timer hasn't been reset, then reset it.
        if (!slideTimerReset) {

            // Set the slide to the desired position.
            this.linearSlides.setTargetLengthInches(scoringSlideExtensionLength);

            // Reset the timer and flip the flag.
            this.slideTimer.reset();
            this.slideTimerReset = true;
        }

        // If the wrist isn't at its desired position yet, wait for such to be true.
        if (this.slideTimer.getElapsedTime(TimeUnit.SECOND) < slideMoveTime){
            return;
        }

        // Open the manipulator.
        this.manipulator.openClaw();

        // Increase the number of scored specimens by 1.
        this.specimensScored += 1;

        // The command has finished running.
        this.isFinished = true;
    }

    /**
     * Updates the flags determining whether or not we are at the target position.
     */
    private void updateScoringLocationFlags() {
        atScoreingLocation = this.drivetrain.atTargetPosition();
        armAtPosition = this.arm.atTargetPosition();
        slidesFullyExtended = this.linearSlides.atTargetPosition();
    }

    @Override
    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public CommandResult getCommandResult() {

        // If less than 3 specimens are scored,
        if (this.specimensScored < 3) {
            return CommandResult.fromDirective(new NextCommandDirective(this.getID(), this.parentCommand));
        }

        // Stop running the OpMode.
        return CommandResult.fromDirective(new TerminateDirective());
    }

    @Override
    public void end(boolean interrupted) {

    }
}
