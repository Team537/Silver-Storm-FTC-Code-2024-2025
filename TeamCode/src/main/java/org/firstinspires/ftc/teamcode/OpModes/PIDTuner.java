package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;

@TeleOp(name = "PID Tuner", group = "2024-2025")
public class PIDTuner extends LinearOpMode {

    RobotHardware robotHardware;
    DataLogger dataLogger;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode()  {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        dataLogger = new DataLogger("System Status", List.of("Time", "Used Memory", "Free Memory", "Total Processors"));
        elapsedTime = new ElapsedTime();

        // Store multiple gamepads to server as a sort of debounce.
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        waitForStart();

        // Setup PID Coefficients
        double p = 0;
        double i = 0;
        double d = 0;

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
            if (currentGamepad.back) {
                robotHardware.periodic();
            }

            // Toggle whether or not the robot will drive using velocity.
            if (currentGamepad.start && !previousGamepad.start) {
                robotHardware.robotArm.setTargetPosition(60);
                robotHardware.robotArm.setP(p);
                robotHardware.robotArm.setI(i);
                robotHardware.robotArm.setD(d);
            }

            // Proportional term.
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                p+= 0.01;
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                p-= 0.01;
            }

            // Derivative term.
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
                d-=0.01;
            }

            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
                d+=0.001;
            }

            // Integral Term
            if (currentGamepad.x && !previousGamepad.x) {
                i+= 0.001;
            }

            // Stop turning the intake if requested.
            if (currentGamepad.b && !previousGamepad.b) {
                i-=0.001;
            }

            // Gauge how well the arm cna reach different targets.
            if (currentGamepad.a && !previousGamepad.a) {
                robotHardware.robotArm.setTargetPosition(135);
            }
            
            if (gamepad1.a) {
                // robotHardware.claw.open();
            }

            // Drive the robot based on user inputs.
            robotHardware.drivetrain.driveRobotWithControllerInputs(currentGamepad.left_stick_x,
                    -currentGamepad.left_stick_y, currentGamepad.right_stick_x);

            //robotHardware.robotArm.setArmPower(power);

            // Update Telemetry.
            telemetry.addLine("P: " + p);
            telemetry.addLine("I: " + i);
            telemetry.addLine("D: " + d);

            telemetry.update();
        }
    }
}
