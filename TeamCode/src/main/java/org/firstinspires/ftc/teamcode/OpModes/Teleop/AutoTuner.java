package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

@TeleOp
public class AutoTuner extends LinearOpMode {

    RobotHardware robotHardware;

    @Override
    public void runOpMode()  {

        // Set the starting position.
        Pose2d startPose = new Pose2d(1.614148, 0.413513, new Rotation2d(-Math.PI, 0));
        Pose2d scorePose = new Pose2d(1.1928, 0.255398, new Rotation2d(-Math.PI, 0));
        Pose2d searchPoseOne = new Pose2d(1.202903, 0.739140, new Rotation2d(-Math.toRadians(140), 0));
        Pose2d searchPoseTwo = new Pose2d(1.214120, -0.600075, new Rotation2d(-Math.toRadians(118), 0));
        Pose2d red2Scoring = new Pose2d(1.518128, -1.492113, new Rotation2d(-Math.toRadians(45), 0));

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry, startPose);
        ElapsedTime elapsedTime = new ElapsedTime();

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();
        double wristPos = 0;

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————" );

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            // Arm position and movement.
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                wristPos -= 0.05;
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                wristPos += 0.05;
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                robotHardware.robotArm.getManipulator().setWristPosition(wristPos);
            }

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.guide && !previousGamepad.guide) {
                robotHardware.drivetrain.toggleAutonomousControl(true);
                robotHardware.robotArm.toggleAutonomousControl(true);
                robotHardware.robotArm.getLinearSlides().toggleAutonomousControl(true);
            }

            // Toggle whether or not the robot will drive in a field centric manner.
            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.drivetrain.setTargetPosition(startPose);
                robotHardware.robotArm.setTargetArmPositionRadians(robotHardware.robotArm.getStartingAngleRadians());
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(0);
                robotHardware.robotArm.getManipulator().closeClaw();
            }

            // Reset the robot's IMU.
            if (currentGamepad.start && !previousGamepad.start) {
                robotHardware.drivetrain.setTargetPosition(scorePose);
                robotHardware.robotArm.getManipulator().closeClaw();
                robotHardware.robotArm.getManipulator().setWristPosition(0.7);
            }


            // Manipulator Control
            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.robotArm.setTargetArmPositionRadians(robotHardware.robotArm.getStartingAngleRadians());
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(0);
            }

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.robotArm.setTargetPosition(35);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(10);
                robotHardware.robotArm.getManipulator().setWristPosition(0.36);
            }

            if (currentGamepad.y && !previousGamepad.y) {
                robotHardware.drivetrain.setTargetPosition(searchPoseOne);
            }

            if (currentGamepad.b && !previousGamepad.b) {
                robotHardware.robotArm.getManipulator().setWristPosition(0.2);
            }


            // Drive the robot based on user inputs.
            double xInput = currentGamepad.left_stick_x;
            double zInput = currentGamepad.left_stick_y;
            double rotationalInput = currentGamepad.right_stick_x;

            // Drive the robot.

            // Call the periodic function for all subsystems.
            robotHardware.periodic();

            // Update Telemetry.
            telemetry.addLine("WristPosition: " + wristPos);
            telemetry.addLine("LoopTIme: " + elapsedTime.getElapsedTime(TimeUnit.SECOND));
            elapsedTime.reset();
            telemetry.update();
        }
    }
}
