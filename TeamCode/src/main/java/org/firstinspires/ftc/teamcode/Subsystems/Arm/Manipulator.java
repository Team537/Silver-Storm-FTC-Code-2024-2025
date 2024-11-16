package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Manipulator implements Subsystem {

    CRServo crServo;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup Hardware
        setupHardware(hardwareMap);
    }

    private void setupHardware(HardwareMap hardwareMap) {
        crServo = hardwareMap.get(CRServo.class, "manipulatorServo");
        crServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake() {
        crServo.setPower(1);
    }

    public void outtake() {
        crServo.setPower(-1);
    }

    public void stopIntake() {
        crServo.setPower(0);
    }

    @Override
    public void periodic() {

    }
}
