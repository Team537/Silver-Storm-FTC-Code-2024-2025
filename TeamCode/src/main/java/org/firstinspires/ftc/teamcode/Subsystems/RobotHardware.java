package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware implements Subsystem {

    // Subsystems
    public Drivetrain drivetrain;

    // Storage
    private Telemetry telemetry;

    @Override
    public void init(HardwareMap opModeHardwareMap, Telemetry opModeTelemetry) {

        // Setup subsystems
        this.drivetrain = new Drivetrain();
        this.drivetrain.init(opModeHardwareMap, opModeTelemetry);

        // Save the telemetry so that diagnostic data can be output.
        this.telemetry = opModeTelemetry;
    }

    @Override
    public void periodic() {
        this.drivetrain.periodic();
    }
}
