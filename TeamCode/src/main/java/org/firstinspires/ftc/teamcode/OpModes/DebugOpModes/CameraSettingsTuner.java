package org.firstinspires.ftc.teamcode.OpModes.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp(name = "Camera Settings Tuner", group = "2024-2025 - Debug")

public class CameraSettingsTuner extends LinearOpMode {

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

        // Store the gain and exposure of the camera so that they can be changed later.
        long exposureTimeMilliseconds = robotHardware.computerVision.getExposure();
        int gain = robotHardware.computerVision.getGain();

        while (opModeInInit() || opModeIsActive()) {

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If A is pressed on the controller, take a photo using the camera.
            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.computerVision.captureFrame();
            }

            // If the right DPad button is pushed, increase exposure by 2.5ms.
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                exposureTimeMilliseconds += 2.5;
                robotHardware.computerVision.setExposure(exposureTimeMilliseconds);
            }

            // If the left DPad button is pushed, decrease exposure by 2.5ms.
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                exposureTimeMilliseconds -= 2.5;
                robotHardware.computerVision.setExposure(exposureTimeMilliseconds);
            }

            // If the up DPad button is pushed, increase gain by 5.
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                gain += 5;
                robotHardware.computerVision.setGain(gain);
            }

            // If the down DPad button is pushed, decrease gain by 5.
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                gain -= 5;
                robotHardware.computerVision.setGain(gain);
            }

            // Display the controls so that other people can more easily figure out how to use this program.
            telemetry.addLine("~~~~~~~~ Controls ~~~~~~~~");
            telemetry.addLine("Dpad Up: Increase gain by 5");
            telemetry.addLine("Dpad Down: Decrease gain by 5");
            telemetry.addLine("Dpad Right: Increase exposure time by 2.5ms");
            telemetry.addLine("Dpad Left: Decrease exposure time by 2.5ms");
            telemetry.addLine("A: Save a photo of what the camera can currently see.");

            // Display current vision values so that the programmer can visualize what is happening.
            telemetry.addLine("~~~~~~~~ Camera Parameters ~~~~~~~~");
            telemetry.addData("Exposure Time (ms)", exposureTimeMilliseconds);
            telemetry.addData("Gain", gain);

            // Update telemetry
            telemetry.update();
        }

        // Save the camera settings to the file.
        robotHardware.computerVision.saveCameraSettings();
    }
}
