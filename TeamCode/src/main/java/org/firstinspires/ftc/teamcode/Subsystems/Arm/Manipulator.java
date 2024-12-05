package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Manipulator implements Subsystem {

    // Hardware Storage
    private CRServo intakeServo;
    private CRServo wristServo;
    private ServoController servoController;

    // Storage
    private Telemetry telemetry;

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

        // Store the telemetry for future use.
        this.telemetry = telemetry;
    }

    /**
     * Sets up the hardware for this subsystem.
     *
     * @param hardwareMap The hardware map so that hardware cna be accessed.
     */
    private void setupHardware(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        servoController = wristServo.getController();
    }

    /**
     * Stops all hardware present within this subsystem.
     */
    public void eStop() {
        this.intakeServo.setPower(0);
        this.eStopped = true;
    }

    /**
     * Sets the claw servo to the given position.
     *
     * @param clawPosition The position that the claw will travel to.
     */
    public void setClawPosition(double clawPosition) {
        servoController.setServoPosition(4, clawPosition);
    }

    /**
     * Sets the wrist servo to the given position.
     *
     * @param wristPosition The position that the wrist will travel to.
     */
    public void setWristPosition(double wristPosition) {
        servoController.setServoPosition(3, wristPosition);
    }

    /**
     * Adjusts the hardware such that it begins intaking samples.
     */
    public void openClaw() {
        servoController.setServoPosition(4, 1);
    }

    /**
     * Adjusts the hardware such that it begins outtaking samples.
     */
    public void closeClaw() {
        servoController.setServoPosition(4, 0);
    }

    @Override
    public void periodic() {

        // If the subsystem is emergency stopped, don't run any more code.
        if (this.eStopped) {
            return;
        }
    }
}
