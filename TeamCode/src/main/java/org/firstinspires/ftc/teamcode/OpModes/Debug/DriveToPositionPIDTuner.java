package org.firstinspires.ftc.teamcode.OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

@TeleOp(name = "Drive to position PID Tuner", group = "2024-2025")
public class DriveToPositionPIDTuner extends LinearOpMode {

    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Setup target positions.
        Pose2d startingPosition = new Pose2d();
        Pose2d topRightCornerPosition = new Pose2d(0.508, 0);
        Pose2d topLeftCorner = new Pose2d(0.508, -0.508);
        Pose2d bottomLeftCorner = new Pose2d(0, -0.508);

        // Wait for the OpMode to be started.
        waitForStart();

        // Setup PID Coefficients
        double p = 0;
        double i = 0;
        double d = 0;

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————" );

            // Call all subsystem's periodic methods .
            robotHardware.periodic();

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Proportional term.
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                p += 0.25;
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                p -= 0.25;
            }

            // Derivative term.
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                d -= 0.001;
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                d += 0.001;
            }

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.guide && !previousGamepad.guide) {
                robotHardware.drivetrain.toggleAutonomousControl();
            }

            // Integral Term
            if (currentGamepad.x && !previousGamepad.x) {
                i += 0.001;
            }

            // Stop turning the intake if requested.
            if (currentGamepad.b && !previousGamepad.b) {
                i -= 0.001;
            }

            // Gauge how well the arm cna reach different targets.
            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.drivetrain.setTargetPosition(topLeftCorner);
                robotHardware.drivetrain.setDriveToPositionPID(p, i, d);
            }

            // Gauge how well the arm cna reach different targets.
            if (currentGamepad.y && !previousGamepad.y) {
                robotHardware.drivetrain.setTargetPosition(topRightCornerPosition);
                robotHardware.drivetrain.setDriveToPositionPID(p, i, d);
            }

            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.drivetrain.setTargetPosition(bottomLeftCorner);
                robotHardware.drivetrain.setDriveToPositionPID(p, i, d);
            }

            if (currentGamepad.start && !previousGamepad.start) {
                robotHardware.drivetrain.setTargetPosition(startingPosition);
                robotHardware.drivetrain.setDriveToPositionPID(p, i, d);
            }

            // Update Telemetry.
            telemetry.addLine("P: " + p);
            telemetry.addLine("I: " + i);
            telemetry.addLine("D: " + d);

            telemetry.update();
        }
    }
}
