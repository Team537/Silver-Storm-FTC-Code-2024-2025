package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name = "Slide Tuner / Tensioner", group = "2024-2025")
public class SlideTuner extends LinearOpMode {
    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {


        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();

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

            // Fast Adjustments
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                robotHardware.robotArm.getLinearSlides().setMotorPower(0.1);
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                robotHardware.robotArm.getLinearSlides().setMotorPower(-0.1);
            }

            // Slow Adjustments
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                robotHardware.robotArm.getLinearSlides().setMotorPower(0.05);
            }

            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                robotHardware.robotArm.getLinearSlides().setMotorPower(-0.05);
            }

            // Stop slides.
            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.robotArm.getLinearSlides().setMotorPower(0);
            }

            robotHardware.periodic();
            telemetry.update();
        }
    }
}
