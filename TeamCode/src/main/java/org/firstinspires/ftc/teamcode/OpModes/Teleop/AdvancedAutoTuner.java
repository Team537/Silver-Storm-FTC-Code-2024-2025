package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp (name = "Advanced Auto Tuner - RED")
public class AdvancedAutoTuner extends LinearOpMode {

    RobotHardware robotHardware;
    @Override
    public void runOpMode() throws InterruptedException {

        // Set the starting position.
        Pose2d startPose = new Pose2d(1.614148, 0.413513, new Rotation2d(-Math.PI, 0));
        Pose2d scorePose = new Pose2d(1.2944, 0.102998 , new Rotation2d(-Math.PI, 0));
        Pose2d searchPoseOne = new Pose2d(1.202903, 0.739140, new Rotation2d(-Math.toRadians(140), 0));
        Pose2d searchPoseTwo = new Pose2d(1.214120, -0.600075, new Rotation2d(-Math.toRadians(118), 0));
        Pose2d red2Scoring = new Pose2d(1.518128, -1.492113, new Rotation2d(-Math.toRadians(45), 0));

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry, startPose);
        ElapsedTime elapsedTime = new ElapsedTime();

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepadOne = new Gamepad();
        Gamepad currentGamepadOne = new Gamepad();

        Gamepad previousGamepadTwo = new Gamepad();
        Gamepad currentGamepadTwo = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();
        double wristPose = 0;
        double slideExtensionDistance = 6.5;
        double armAngle = 0;
        Pose2d positionalOffset = new Pose2d();

        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————" );
            telemetry.addLine("~~~~~~~~~~ Controls ~~~~~~~~~~" );
            telemetry.addLine("———— Gamepad 1 —————" );
            telemetry.addLine("Start: Go to the scoring position");
            telemetry.addLine("Back: Go to the start position");
            telemetry.addLine("Guide: Enable autonomous control");
            telemetry.addLine("Dpad Up: Increase arm angle by 2 Degrees");
            telemetry.addLine("Dpad Down: Decrease arm angle by 2 Degrees");
            telemetry.addLine("Dpad Left: Decrease wrist angle by 0.05");
            telemetry.addLine("Dpad Right: Increase wrist angle by 0.05");
            telemetry.addLine("X: Set arm angle to the specified angle");
            telemetry.addLine("A: Set the wrist position to the set value");
            telemetry.addLine("B: Open the claw");
            telemetry.addLine("Y: Close the claw");
            telemetry.addLine("———— Gamepad 2 —————" );
            telemetry.addLine("Start: E-stop the robot");
            telemetry.addLine("Back: Go to the search position");
            telemetry.addLine("Dpad Up: Increase the robot's X position by 0.5in (0.0127m)");
            telemetry.addLine("Dpad Down: Decrease the robot's X position by 0.5in (0.0127m)");
            telemetry.addLine("Dpad Left: Decrease the robot's Z position by 0.5in (0.0127m)");
            telemetry.addLine("Dpad Right: Increase the robot's Z position by 0.5in (0.0127m)");
            telemetry.addLine("X: Drive the robot to the new starting location.");
            telemetry.addLine("B: Decrease the linear slide extension distance 1/4 in");
            telemetry.addLine("Y: Increase the linear slide extension distance 1/4 in");
            telemetry.addLine("A: Set the linear slide position to the set value");
            telemetry.addLine("~~~~~~~~~~ Diagnostics ~~~~~~~~~~" );

            // Update controllers.
            previousGamepadOne.copy(currentGamepadOne);
            currentGamepadOne.copy(gamepad1);
            previousGamepadTwo.copy(currentGamepadTwo);
            currentGamepadTwo.copy(gamepad2);

            // Enable autonomous control.
            if (currentGamepadOne.guide && !previousGamepadOne.guide) {
                robotHardware.drivetrain.toggleAutonomousControl(true);
                robotHardware.robotArm.toggleAutonomousControl(true);
                robotHardware.robotArm.getLinearSlides().toggleAutonomousControl(true);
            }

            // Base positions
            if (currentGamepadOne.back && !previousGamepadOne.back) {
                robotHardware.drivetrain.setTargetPosition(startPose);
                robotHardware.robotArm.setTargetArmPositionRadians(robotHardware.robotArm.getStartingAngleRadians());
                robotHardware.robotArm.getManipulator().setWristPosition(.95);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(0);
            }

            if (currentGamepadOne.start && !previousGamepadOne.start) {
                robotHardware.drivetrain.setTargetPosition(scorePose);
                robotHardware.robotArm.getManipulator().setWristPosition(.45);
                robotHardware.robotArm.setTargetPosition(40);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(10);
            }

            // Arm Angle
            if (currentGamepadOne.dpad_up && !previousGamepadOne.dpad_up) {
                armAngle += 2;
            }
            if (currentGamepadOne.dpad_down && !previousGamepadOne.dpad_down) {
                armAngle -= 2;
            }

            if (currentGamepadOne.x && !previousGamepadOne.x) {
                robotHardware.robotArm.setTargetPosition(armAngle);
            }

            // Wrist Position
            if (currentGamepadOne.dpad_left && !previousGamepadOne.dpad_left) {
                wristPose -= 0.05;
            }
            if (currentGamepadOne.dpad_right && !previousGamepadOne.dpad_right) {
                wristPose += 0.05;
            }

            if (currentGamepadOne.a && !previousGamepadOne.a) {
                robotHardware.robotArm.getManipulator().setWristPosition(wristPose);
            }

            // Claw Stuff
            if (currentGamepadOne.y && !previousGamepadOne.y) {
                robotHardware.robotArm.getManipulator().closeClaw();
            }
            if (currentGamepadOne.b && !previousGamepadOne.b) {
                robotHardware.robotArm.getManipulator().openClaw();
            }

            // Drivetrain Stuff
            if (currentGamepadTwo.dpad_up && !currentGamepadTwo.dpad_up) {
                positionalOffset.setX(positionalOffset.getX() + 0.0127);
            }
            if (currentGamepadTwo.dpad_down && !currentGamepadTwo.dpad_down) {
                positionalOffset.setX(positionalOffset.getX() - 0.0127);
            }

            if (currentGamepadTwo.dpad_left && !currentGamepadTwo.dpad_left) {
                positionalOffset.setZ(positionalOffset.getZ() - 0.0127);
            }
            if (currentGamepadTwo.dpad_down && !currentGamepadTwo.dpad_right) {
                positionalOffset.setZ(positionalOffset.getZ() + 0.0127);
            }

            // Set the enw drive to position target.
            if (currentGamepadTwo.x && !currentGamepadTwo.x) {
                Pose2d offsetClone = positionalOffset.clone();
                offsetClone = robotHardware.drivetrain.getCoordinateSystem().robotSpaceToFieldSpace(offsetClone);

                Pose2d targetPosition = robotHardware.drivetrain.getTargetPosition().clone();
                targetPosition.addValues(offsetClone);

                robotHardware.drivetrain.setTargetPosition(targetPosition);
            }

            // Linear Slides
            if (currentGamepadTwo.y && !previousGamepadTwo.y) {
                slideExtensionDistance += 0.25;
            }
            if (currentGamepadTwo.a && !previousGamepadTwo.a) {
                slideExtensionDistance -= 0.25;
            }
            if (currentGamepadTwo.a && !previousGamepadTwo.a) {
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(slideExtensionDistance);
            }

            // Other positional Information:
            if (currentGamepadTwo.back && !currentGamepadTwo.back) {
                robotHardware.drivetrain.setTargetPosition(searchPoseOne);
                robotHardware.robotArm.setTargetArmPositionRadians(robotHardware.robotArm.getStartingAngleRadians());
                robotHardware.robotArm.getManipulator().setWristPosition(0.7);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(0);
                robotHardware.robotArm.getManipulator().closeClaw();
            }

            if (currentGamepadTwo.start && !currentGamepadTwo.start) {
                robotHardware.robotArm.eStop();
            }

            // Update Subsystems
            robotHardware.periodic();

            // Display Debug Data
            telemetry.addLine("———— Debug Data —————" );
            telemetry.addLine("Manuel Wrist Pose:" + wristPose);
            telemetry.addLine("Manuel Slide Extension:" + slideExtensionDistance);
            telemetry.addLine("Manuel Arm Angle:" + armAngle);
            telemetry.addLine("Manuel Pose Offset: (" + positionalOffset.getX() + ", " + positionalOffset.getZ() + ")");
            telemetry.update();
        }
    }
}
