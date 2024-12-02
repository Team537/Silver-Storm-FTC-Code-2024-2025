package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

@TeleOp(name = "Arm Tuner", group = "2024-2025")

public class ArmTuner extends LinearOpMode {
    private RobotHardware robotHardware;
    private ElapsedTime loopTimer;

    @Override
    public void runOpMode() {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();
        double powerStep = 0;
        double power = 0;
        loopTimer = new ElapsedTime();

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————");

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Adjust arm motor power step.
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                power -= 0.01;
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                power += 0.01;
            }
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                power += 0.001;
            }
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                power -= 0.001;
            }

            // Reset motor power step.
            if (currentGamepad.guide && !previousGamepad.guide) {
                power = 0;
            }

            // Adjust arm motor power.
            if (currentGamepad.x && !previousGamepad.x) {
                power -= 0.001;
            }

            if (currentGamepad.b && !previousGamepad.b) {
                power += 0.001;
            }

            // Reset arm motor power.
            if (currentGamepad.a && !previousGamepad.a) {
                power -= 0.0001;
            }

            if (currentGamepad.y && !previousGamepad.y) {
                power += 0.0001;
            }

            // Call the periodic function for all subsystems.
            robotHardware.periodic();
            robotHardware.robotArm.setPower(power);

            // Display Tuning Parameters
            telemetry.addLine("----- Arm Tuning Parameters -----");
            telemetry.addLine("Arm Power Step: " + powerStep);
            telemetry.addLine("Arm Power: " + power);

            telemetry.addLine("----- OpMode Status -----");
            telemetry.addLine("Loop Time (s): " + loopTimer.getElapsedTime(TimeUnit.SECOND));

            // Update Telemetry.
            telemetry.update();

            loopTimer.reset();
        }
    }
}
