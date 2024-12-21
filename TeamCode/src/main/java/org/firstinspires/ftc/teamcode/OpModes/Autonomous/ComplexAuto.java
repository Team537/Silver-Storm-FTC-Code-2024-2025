package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Commands.AutonomousCommands.CreateSpecimenCommand;
import org.firstinspires.ftc.teamcode.Commands.AutonomousCommands.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Commands.AutonomousCommands.SearchAndGrabSpikemarkSamplesCommand;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Storage.FileEx;

import java.util.HashSet;
import java.util.Set;

@Autonomous (name = "Complex Auto", group = "2024-2025")
public class ComplexAuto extends LinearOpMode {

    RobotHardware robotHardware;
    DataLogger dataLogger;
    FileEx robotPositionFile = new FileEx("RobotPose");
    @Override
    public void runOpMode() throws InterruptedException {

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Get the command scheduler.
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Autonomous Data.
        AutonomousRoutine autonomousRoutine = AutonomousRoutine.RED_ONE;

        // While the OpMode is initializing, allow for certain autos to be selected.
        while (opModeInInit()) {

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Red Alliance Autonomous Selections
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                autonomousRoutine = AutonomousRoutine.RED_TWO;
            }
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                autonomousRoutine = AutonomousRoutine.RED_ONE;
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                autonomousRoutine = AutonomousRoutine.BLUE_TWO;
            }
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                autonomousRoutine = AutonomousRoutine.BLUE_ONE;
            }

            // Break free from the loop early if the center button is pressed. This makes it possible to
            // still load hardware early.
            if (currentGamepad.guide) {
                break;
            }

            // Display Data to the user.
            telemetry.addLine("Autonomous Routine: " + autonomousRoutine);
            telemetry.update();
        }

        // Setup the robot's hardware. This is done late
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry, autonomousRoutine.getStartingPosition());

        // Preemptively get hardware.
        Drivetrain drivetrain = robotHardware.drivetrain;
        Arm arm = robotHardware.robotArm;

        ComputerVision computerVision = robotHardware.computerVision;
        SampleDetectionPipeline genericSamplePipeline = computerVision.getSampleDetectionPipeline();

        // Create the autonomous commands.
        ScoreSpecimenCommand scoreSpecimenCommand = new ScoreSpecimenCommand(drivetrain, arm, autonomousRoutine);
        SearchAndGrabSpikemarkSamplesCommand searchAndGrabSpikemarkSamplesCommand = new SearchAndGrabSpikemarkSamplesCommand(drivetrain, computerVision, arm, autonomousRoutine);
        CreateSpecimenCommand createSpecimenCommand = new CreateSpecimenCommand(drivetrain, computerVision, arm, autonomousRoutine);
        createSpecimenCommand.setScoreCommandID(scoreSpecimenCommand.getID());

        // Schedule the commands
        commandScheduler.scheduleCommand(scoreSpecimenCommand, 1, drivetrain, arm, arm.getLinearSlides(), arm.getManipulator());
        commandScheduler.scheduleCommand(searchAndGrabSpikemarkSamplesCommand, 1, drivetrain, computerVision, arm, arm.getLinearSlides(), arm.getManipulator());
        commandScheduler.scheduleCommand(createSpecimenCommand, 1, drivetrain, computerVision, arm, arm.getLinearSlides(), arm.getManipulator());

        // Prevent auto from running early if the guide button is pressed.
        waitForStart();

        // Activate the first command added.
        Set<String> commandsToActivate = new HashSet<>(List.of(scoreSpecimenCommand.getID()));
        try {
            commandScheduler.activateCommands(commandsToActivate);
        } catch (UnscheduledCommandException e) {
            throw new RuntimeException(e);
        }

        while (opModeIsActive()) {

            // Execute all active commands.
            try {
                commandScheduler.execute();
            } catch (UnscheduledCommandException | NullCommandException e) {
                throw new RuntimeException(e);
            }

            // Call the periodic function of all hardware.
            robotHardware.periodic();
            telemetry.update();
        }

        Pose2d robotPosition = robotHardware.drivetrain.getRobotPosition();
        robotPositionFile.addData("x",robotPosition.getX());
        robotPositionFile.addData("z",robotPosition.getZ());
        robotPositionFile.addData("theta",robotPosition.getYawInRadians());
        robotPositionFile.saveData();
    }
}
