package org.firstinspires.ftc.teamcode.OpModes.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Sample;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp(name = "Drive To Sample Tuner", group = "2024-2025 - Debug")
public class VisionPositionTuner extends LinearOpMode {

    RobotHardware robotHardware;
    DataLogger dataLogger;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        elapsedTime = new ElapsedTime();

        Pose2d grabOffset = new Pose2d(-0.1905, -0.0127);
        boolean active = false;
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

            if (currentGamepad.guide && !previousGamepad.guide) {
                robotHardware.drivetrain.toggleAutonomousControl(true);

                // Toggle Active
                active = true;
            }

            if (currentGamepad.start && !previousGamepad.start) {
                Pose2d grabOffsetInFieldSpace = robotHardware.drivetrain.getCoordinateSystem().robotSpaceToFieldSpace(grabOffset.clone());
                List<Sample> detectedObjects = robotHardware.computerVision.getDetectedObjects();
                Pose2d samplePosition = detectedObjects.get(0).getFieldPosition().clone();
                samplePosition.addValues(grabOffsetInFieldSpace);
                robotHardware.drivetrain.setTargetPosition(samplePosition);

                // Toggle Active
                active = true;
            }


            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.robotArm.getManipulator().openClaw();
                robotHardware.robotArm.getManipulator().setWristPosition(1);
            }
            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.robotArm.getManipulator().closeClaw();
            }
            // Call the periodic methods of all of the subsystems
            robotHardware.periodic();

            // Update telemetry
            telemetry.update();
        }

        // Save the camera settings to the file.
        robotHardware.computerVision.saveCameraSettings();
    }
}
