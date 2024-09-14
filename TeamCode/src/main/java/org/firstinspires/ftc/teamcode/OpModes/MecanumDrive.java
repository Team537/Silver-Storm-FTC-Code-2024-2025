package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

            // Toggle whether or not the robot will drive in a field centric manner.
            if (currentGamepad.back && !previousGamepad.back) {
                robotHardware.drivetrain.toggleFieldCentricDrive();
            }

            // Drive the robot based on user inputs.
            robotHardware.drivetrain.driveRobotWithControllerInputs(currentGamepad.left_stick_x,
                    currentGamepad.left_stick_y, currentGamepad.right_stick_x);

            // Update Telemetry.
            telemetry.update();
        }
    }
}
