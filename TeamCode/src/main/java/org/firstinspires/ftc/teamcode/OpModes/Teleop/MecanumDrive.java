package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmPositions;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name = "Mecanum Drive", group = "2024-2025")
public class MecanumDrive extends LinearOpMode {

    RobotHardware robotHardware;

    @Override
    public void runOpMode()  {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();
        double power = 0;

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

            // Arm position and movement.
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                robotHardware.robotArm.setTargetPosition(ArmPositions.FLOOR_POSITION);
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                robotHardware.robotArm.setTargetPosition(ArmPositions.HIGH_BASKET);
            }
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                robotHardware.robotArm.setTargetPosition(ArmPositions.CENTER_CROSS);
            }
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                robotHardware.robotArm.setTargetPosition(ArmPositions.LOW_BASKET);
            }

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.guide && !previousGamepad.guide) {
                robotHardware.drivetrain.toggleVelocityDrive();
            }

            // Toggle whether or not the robot will drive in a field centric manner.
            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.drivetrain.toggleFieldCentricDrive();
            }

            // Reset the robot's IMU.
            if (currentGamepad.start && !previousGamepad.start) {
                robotHardware.drivetrain.resetIMU();
            }

            // Manipulator Control
            if (currentGamepad.x && !previousGamepad.x) {
                robotHardware.robotArm.getManipulator().intake();
            }

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.robotArm.getManipulator().outtake();
            }

            if (currentGamepad.b && !previousGamepad.b) {
                robotHardware.robotArm.getManipulator().stopIntake();
            }

            // Drive the robot based on user inputs.
            double xInput = currentGamepad.left_stick_x;
            double zInput = currentGamepad.left_stick_y;
            double rotationalInput = currentGamepad.right_stick_x;

            // Drive the robot.
            robotHardware.drivetrain.driveRobotWithControllerInputs(xInput,
                    -zInput, rotationalInput);

            // Call the periodic function for all subsystems.
            robotHardware.periodic();

            // Update Telemetry.
            telemetry.update();
        }
    }
}
