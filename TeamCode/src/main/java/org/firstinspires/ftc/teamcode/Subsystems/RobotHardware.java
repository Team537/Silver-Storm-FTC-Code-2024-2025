package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Arm.Manipulator;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.ComputerVision;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousRoutine;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

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

        this.computerVision = new ComputerVision(this.drivetrain.getCoordinateSystem()::robotSpaceToFieldSpace);
        computerVision.init(hardwareMap, telemetry);

        this.robotArm = new Arm();
        this.robotArm.init(hardwareMap, telemetry);

        // Save the telemetry so that diagnostic data can be output.
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startingPosition) {

        // Setup subsystems
        this.drivetrain = new Drivetrain(startingPosition);
        this.drivetrain.init(hardwareMap, telemetry);

        this.computerVision = new ComputerVision(this.drivetrain.getCoordinateSystem()::robotSpaceToFieldSpace);
        computerVision.init(hardwareMap, telemetry);

        this.robotArm = new Arm();
        this.robotArm.init(hardwareMap, telemetry);

        // Save the telemetry so that diagnostic data can be output.
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        this.drivetrain.periodic();
        this.computerVision.periodic();
        this.robotArm.periodic();
    }
}
