package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp(name = "Linear Slide PID Tuner", group = "2024-2025")
public class LinearSlidePIDTuner extends LinearOpMode {
    RobotHardware robotHardware;
    DataLogger dataLogger;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        dataLogger = new DataLogger("System Status", List.of("Time", "Used Memory", "Free Memory", "Total Processors"));
        elapsedTime = new ElapsedTime();

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();
        // Store the linear slides as a variable.
        LinearSlides linearSlides = robotHardware.robotArm.getLinearSlides();

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
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————");

            // Call all subsystem's periodic methods .
            robotHardware.periodic();

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.start && !previousGamepad.start) {
                linearSlides.setTargetLength(5.5);
                //linearSlides.setP(p);
                //linearSlides.setI(i);
                //linearSlides.setD(d);
            }

            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                robotHardware.robotArm.setTargetPosition(60);
            }

            if (currentGamepad.back && !previousGamepad.back) {
                linearSlides.setTargetLength(0);
            }

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
                linearSlides.toggleActive();
                robotHardware.robotArm.toggleActive();
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
                linearSlides.setTargetLength(9);
            }

            // Gauge how well the arm cna reach different targets.
            if (currentGamepad.y && !previousGamepad.y) {
                linearSlides.setTargetLength(0);
            }
            
            if (gamepad1.a) {
                // robotHardware.claw.open();
            }

            // Drive the robot based on user inputs.
            robotHardware.drivetrain.driveRobotWithControllerInputs(currentGamepad.left_stick_x,
                    -currentGamepad.left_stick_y, currentGamepad.right_stick_x);

            //robotHardware.robotArm.setArmPower(power);

            // Update Telemetry.
            telemetry.addLine("P: " + p);
            telemetry.addLine("I: " + i);
            telemetry.addLine("D: " + d);

            telemetry.update();
        }
    }
}
