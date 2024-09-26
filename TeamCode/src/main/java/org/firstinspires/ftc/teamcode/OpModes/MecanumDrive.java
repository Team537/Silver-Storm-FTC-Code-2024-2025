package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

@TeleOp(name = "Mecanum Drive", group = "2024-2025")
public class MecanumDrive extends LinearOpMode {

    // Hardware
    RobotHardware robotHardware;

    // Systems
    DataLogger dataLogger;
    ElapsedTime opModeRuntime;

    // Gamepads
    Gamepad previousGamepad;
    Gamepad currentGamepad;

    @Override
    public void runOpMode()  {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Setup other values.
        this.dataLogger = new DataLogger("Log-RobotState", List.of("time", "input_left_stick_x",
                "input_left_stick_y", "input_right_stick", "fr_motor_velocity", "fl_motor_velocity",
                "bl_motor_velocity", "br_motor_velocity"));

        // Store multiple gamepads to server as a sort of debounce.
        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();

        // Setup the elapsed time
        opModeRuntime = new ElapsedTime();

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Call all subsystem's periodic methods .
            robotHardware.periodic();

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Toggle whether or not the robot will drive in a field centric manner.
            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.drivetrain.toggleFieldCentricDrive();
            }

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.start && !previousGamepad.start) {
                robotHardware.drivetrain.toggleVelocityDrive();
            }

            // Drive the robot based on user inputs.
            robotHardware.drivetrain.driveRobotWithControllerInputs(currentGamepad.left_stick_x,
                    currentGamepad.left_stick_y, currentGamepad.right_stick_x);

            // Update Diagnostics
            updateDiagnostics();
        }
    }

    /**
     * Logs relevant data and displays system data using the telemetry.
     */
    private void updateDiagnostics() {

        // Get the robot's current state.
        double[] robotState = robotHardware.drivetrain.getVelocityState();
        dataLogger.logData(List.of(opModeRuntime.getElapsedTime(TimeUnit.SECOND), currentGamepad.right_stick_x,
                currentGamepad.left_stick_y, robotState[0], robotState[1], robotState[2], robotState[3]));

        // Display the current system state on the telemetry.
        telemetry.addLine("~~~~~~~~~~ Diagnostics ~~~~~~~~~~");
        telemetry.addLine("elapsed_time: " + opModeRuntime.getElapsedTime(TimeUnit.SECOND) + "s" );
        telemetry.addLine("runtime: " + this.getRuntime() + "s" );
        telemetry.addLine("~~~~~~~~~~ Settings ~~~~~~~~~~");
        telemetry.addLine("Field Centric Drive Status: " + robotHardware.drivetrain.getFieldCentricEnabled());
        telemetry.addLine("Velocity Drive Status: " + robotHardware.drivetrain.getVelocityDriveEnabled());
        telemetry.addLine("~~~~~~~~~~ Inputs ~~~~~~~~~~");
        telemetry.addLine("input_left_stick_x: " + currentGamepad.left_stick_x);
        telemetry.addLine("input_left_stick_y: " + currentGamepad.left_stick_y);
        telemetry.addLine("~~~~~~~~~~ Outputs ~~~~~~~~~~");
        telemetry.addLine("fr_motor_velocity: " + robotState[0]);
        telemetry.addLine("fl_motor_velocity: " + robotState[1]);
        telemetry.addLine("bl_motor_velocity: " + robotState[2]);
        telemetry.addLine("br_motor_velocity: " + robotState[3]);

        // Update telemetry.
        telemetry.update();
    }
}
