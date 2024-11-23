package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Manipulator implements Subsystem {

    // Hardware Storage
    private CRServo crServo;

    // Flags
    private boolean eStopped = false;

    /**
     * Sets up this subsystem so that it can function.
     *
     * @param hardwareMap The opMode's hardware map. This is required in order to gian access
     *                    to the robot's hardware.
     * @param telemetry The opMode's telemetry. This is required in order to output
     *                  diagnostic / feedback information.
     */
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup Hardware.
        setupHardware(hardwareMap);
    }

    /**
     * Sets up the hardware for this subsystem.
     *
     * @param hardwareMap The hardware map so that hardware cna be accessed.
     */
    private void setupHardware(HardwareMap hardwareMap) {
        crServo = hardwareMap.get(CRServo.class, "intakeServo");
        crServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Stops all hardware present within this subsystem.
     */
    public void eStop() {
        this.crServo.setPower(0);
        this.eStopped = true;
    }

    /**
     * Adjusts the hardware such that it begins intaking samples.
     */
    public void intake() {
        crServo.setPower(1);
    }

    public void setMotorPower(double motorPower) {
        this.crServo.setPower(motorPower);
    }

    /**
     * Adjusts the hardware such that it begins outtaking samples.
     */
    public void outtake() {
        crServo.setPower(-1);
    }

    /**
     * Stops all running hardware.
     */
    public void stopIntake() {
        crServo.setPower(0);
    }

    @Override
    public void periodic() {

        // If the subsystem is emergency stopped, don't run any more code.
        if (this.eStopped) {
            return;
        }
    }
}
