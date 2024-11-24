package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;

public class RobotHardware implements Subsystem {

    // Subsystems
    public Drivetrain drivetrain;
    public ComputerVision computerVision;
    public Arm robotArm;

    // Storage
    private Telemetry telemetry;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup subsystems
        this.drivetrain = new Drivetrain();
        this.drivetrain.init(hardwareMap, telemetry);

        this.computerVision = new ComputerVision();
        computerVision.init(hardwareMap, telemetry);

        this.robotArm = new Arm();
        this.robotArm.init(hardwareMap, telemetry);

        // Save the telemetry so that diagnostic data can be output.
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        // this.drivetrain.periodic();
        this.robotArm.periodic();
    }
}
