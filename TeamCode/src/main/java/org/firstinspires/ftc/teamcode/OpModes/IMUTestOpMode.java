package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@TeleOp(name = "IMU Test", group = "Debug-2024-2025")
public class IMUTestOpMode extends LinearOpMode {
    RobotHardware robotHardware;

    @Override
    public void runOpMode() throws InterruptedException {

        // Setup the robot's hardware.
        robotHardware = new RobotHardware();
        robotHardware.init(this.hardwareMap, this.telemetry);

        waitForStart();

        while (opModeIsActive()) {
            robotHardware.imu.performKalmanFiltering();
        }

        robotHardware.imu.save();
    }
}
