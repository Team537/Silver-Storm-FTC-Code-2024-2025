package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Controllers.RotationalPIDController;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utility.Storage.FileEx;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

@TeleOp(name = "Mecanum Drive", group = "2024-2025")
public class MecanumDrive extends LinearOpMode {

    private Pose2d redOneScoringLocation = new Pose2d(1.2944, 0.102998, new Rotation2d(Math.toRadians(-180), 0));
    private Pose2d blueOneScoringLocation = new Pose2d(1.2944, 0.102998, new Rotation2d(0, 0));
    private RobotHardware robotHardware;
    private RotationalPIDController rotationalPIDController = new RotationalPIDController(0.5, 0.005, 0);
    private PIDController xPIDController = new PIDController(1.25, 0.005, 0);
    private PIDController zPIDController = new PIDController(1.25, 0.005, 0);

    @Override
    public void runOpMode()  {

        // Get the robot's position.
        FileEx robotPositionFile = new FileEx("RobotPose");
        double x = 0, z = 0, theta = 0;
        if (robotPositionFile.getValue("x", Double.class) != null) {
            x = robotPositionFile.getValue("x", Double.class);
        }
        if (robotPositionFile.getValue("z", Double.class) != null) {
            z = robotPositionFile.getValue("z", Double.class);
        }
        if (robotPositionFile.getValue("theta", Double.class) != null) {
            theta = robotPositionFile.getValue("theta", Double.class);
        }
        Pose2d robotPosition = new Pose2d(x, z, new Rotation2d(theta, 0));

        // Store the alliance
        Alliance alliance = Alliance.RED;

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry, robotPosition);

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepadTwo = new Gamepad();
        Gamepad currentGamepadTwo = new Gamepad();

        // Wait for the OpMode to be started.
        waitForStart();
        double slideExtensionLength = 0;
        boolean autoDrive = false;

        // Rotational Correction
        ElapsedTime rotationalFreeTime = new ElapsedTime();
        ElapsedTime xFreeTime = new ElapsedTime();
        ElapsedTime zFreeTime = new ElapsedTime();

        double lockedAngle = theta;
        double lockedX = theta;
        double lockedZ = theta;

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————" );

            // Update controllers.
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            previousGamepadTwo.copy(currentGamepadTwo);
            currentGamepadTwo.copy(gamepad2);

            // Arm position and movement.
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                robotHardware.robotArm.setTargetArmPositionRadians(robotHardware.robotArm.getStartingAngleRadians());
                robotHardware.robotArm.getManipulator().setWristPosition(1);
                robotHardware.robotArm.toggleAutonomousControl(true);
            }
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                robotHardware.robotArm.setTargetPosition(80);
                robotHardware.robotArm.getManipulator().setWristPosition(0.1);
                robotHardware.robotArm.toggleAutonomousControl(true);
            }
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                robotHardware.robotArm.setTargetPosition(43);
                robotHardware.robotArm.getManipulator().setWristPosition(.45);
                robotHardware.robotArm.toggleAutonomousControl(true);
            }
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                robotHardware.robotArm.setTargetPosition(-20);
                robotHardware.robotArm.getManipulator().setWristPosition(0.7);
                robotHardware.robotArm.toggleAutonomousControl(true);
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
                robotHardware.robotArm.getManipulator().openClaw();
            }

            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.robotArm.getManipulator().closeClaw();
            }

            if (currentGamepad.b && !previousGamepad.b) {
                robotHardware.robotArm.getManipulator().setWristPosition(0);
            }

            if (currentGamepad.y && !previousGamepad.y) {
                robotHardware.robotArm.getManipulator().setWristPosition(1);
            }

            // Gamepad2
            if (currentGamepadTwo.dpad_up && !previousGamepadTwo.dpad_up) {
                slideExtensionLength += 1;
                slideExtensionLength = Math.min(11, slideExtensionLength);
                slideExtensionLength = Math.max(0, slideExtensionLength);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(slideExtensionLength);
                robotHardware.robotArm.getLinearSlides().toggleAutonomousControl(true);
            }

            if (currentGamepadTwo.dpad_down && !previousGamepadTwo.dpad_down) {
                slideExtensionLength -= 1;
                slideExtensionLength = Math.min(11, slideExtensionLength);
                slideExtensionLength = Math.max(0, slideExtensionLength);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(slideExtensionLength);
                robotHardware.robotArm.getLinearSlides().toggleAutonomousControl(true);
            }

            // Switch Alliance
            if (currentGamepadTwo.left_trigger == 1) {
                alliance = Alliance.BLUE;
            }
            if (currentGamepadTwo.right_trigger == 1) {
                alliance = Alliance.RED;
            }

            // Toggle autonomous control.
            if (currentGamepadTwo.guide && !previousGamepadTwo.guide) {
                autoDrive = !autoDrive;
                robotHardware.drivetrain.toggleAutonomousControl(autoDrive);
                robotHardware.robotArm.getLinearSlides().toggleAutonomousControl(true);
                robotHardware.robotArm.toggleAutonomousControl(true);
            }

            if (currentGamepadTwo.x && !previousGamepadTwo.x) {
                switch (alliance) {
                    case RED:
                        robotHardware.drivetrain.setTargetPosition(redOneScoringLocation);
                        break;
                    case BLUE:
                        robotHardware.drivetrain.setTargetPosition(blueOneScoringLocation);
                        break;
                }
                robotHardware.robotArm.setTargetPosition(40);
                robotHardware.robotArm.getManipulator().setWristPosition(0.45);
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(10);
            }

            if (currentGamepadTwo.a && !previousGamepadTwo.a) {
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(6);
            }
            if (currentGamepadTwo.b && !previousGamepadTwo.b) {
                robotHardware.robotArm.getLinearSlides().setTargetLengthInches(0);
                robotHardware.robotArm.getManipulator().setWristPosition(1);
                robotHardware.robotArm.getManipulator().openClaw();
            }

            // Get joystick inputs from the gamepad for x (strafe), z (forward/backward), and rotation
            double xInput = currentGamepad.left_stick_x; // Strafe input (left/right movement)
            double zInput = currentGamepad.left_stick_y; // Forward/backward input
            double rotationalInput = currentGamepad.right_stick_x; // Rotational input (turning)

            // Apply a power curve to smooth joystick inputs while preserving their signs
            double curvedXInput = Math.pow(Math.abs(xInput), 1.7) * Math.signum(xInput); // Curve x input
            double curvedZInput = Math.pow(Math.abs(zInput), 1.7) * Math.signum(zInput); // Curve z input
            double curvedRotationalInput = Math.pow(Math.abs(rotationalInput), 1.7) * Math.signum(rotationalInput); // Curve rotational input

            // Get the robot's current angle in radians (from the drivetrain's sensors or odometry)
            Pose2d currentRobotPosition = robotHardware.drivetrain.getRobotPosition();
            double robotX = currentRobotPosition.getX();
            double robotZ = currentRobotPosition.getZ();
            double robotAngle = robotHardware.drivetrain.getRobotPosition().getYawInRadians();

            // If rotational input is very small (indicating the driver is not actively turning)
            if (Math.abs(curvedRotationalInput) < 0.01 && rotationalFreeTime.getElapsedTime(TimeUnit.SECOND) > 0.5) {
                // Check if the robot's current angle deviates significantly from the locked angle
                if (Math.abs(lockedAngle - robotAngle) > Math.toRadians(0.75)) {
                    // Use a PID controller to correct the robot's angle back to the locked angle
                    curvedRotationalInput = -rotationalPIDController.update(lockedAngle, robotAngle);
                }
            } else {
                // If the driver is actively turning, update the locked angle to the current robot angle
                lockedAngle = robotAngle;
                if (Math.abs(curvedRotationalInput) > 0.01) {
                    rotationalFreeTime.reset();
                }
            }

            // If autonomous driving is not enabled
            if (!autoDrive) {
                // Use the calculated curved inputs to control the robot's drivetrain
                robotHardware.drivetrain.driveRobotWithControllerInputs(
                        curvedXInput, // Adjusted strafe input
                        -curvedZInput, // Adjusted forward/backward input (negated for correct direction)
                        curvedRotationalInput // Adjusted rotational input
                );
            }

            // Call the periodic function for all subsystems.
            robotHardware.periodic();

            // Update Telemetry.
            telemetry.update();
        }
    }
}
