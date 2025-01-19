package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm.ArmPositions;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@Autonomous(name = "Drive Left Auto", group = "2024-2025")
public class LeftAuto extends LinearOpMode {

    RobotHardware robotHardware;

    @Override
    public void runOpMode()  {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        // Wait for the OpMode to be started.
        waitForStart();

        // While the opmode is active, allow the robot to be controlled by the driver and display
        // useful diagnostic information.
        while (opModeIsActive()) {

            // Print out the opMode's current runtime.
            telemetry.addLine("———— Runtime: " + this.getRuntime() + "s —————" );

            // Call all subsystem's periodic methods .
            robotHardware.periodic();

            // Drive the robot.
            robotHardware.drivetrain.driveRobotWithControllerInputs(-0.25, 0, 0);

            // Call the periodic function for all subsystems.
            robotHardware.periodic();

            // Update Telemetry.
            telemetry.update();
        }
    }
}
