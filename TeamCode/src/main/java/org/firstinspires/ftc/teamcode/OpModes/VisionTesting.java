package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp(name = "Vision Test", group = "2024-2025 - Debug")

public class VisionTesting extends LinearOpMode {

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

        while (opModeInInit()) {

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // If A is pressed on the controller, take a photo using the camera.
            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.computerVision.captureFrame();
            }
        }
    }
}
