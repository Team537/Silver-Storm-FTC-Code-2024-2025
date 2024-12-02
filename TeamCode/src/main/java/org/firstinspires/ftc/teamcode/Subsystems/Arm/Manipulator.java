package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Manipulator implements Subsystem {

    // Hardware Storage
    private CRServo intakeServo;
    private CRServo wristServo;
    private ServoController servoController;

    // Storage
    private Telemetry telemetry;
    private WristPosition targetPosition = WristPosition.START_POSITION;

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
     * Sets the wrist's position.
     *
     * @param wristPosition The position the wrist will travel to.
     */
    public void setWristPosition(WristPosition wristPosition) {

        // Store the target wrist position.
        this.targetPosition = wristPosition;

        // Set the target angle for the wrist.
        this.servoController.setServoPosition(3, this.targetPosition.getWristAnglePosition());
    }

    public void setWristPosition(double theta) {
        double position = 0.5 + ((theta / 90) * 0.5);
        servoController.setServoPosition(3, position);
    }

    /**
     * Adjusts the hardware such that it begins intaking samples.
     */
    public void intake() {
        servoController.setServoPosition(4, 1);
    }

    /**
     * Adjusts the hardware such that it begins outtaking samples.
     */
    public void outtake() {
        servoController.setServoPosition(4, 0);
    }

    /**
     * Stops all running hardware.
     */
    public void stopIntake() {
        intakeServo.setPower(0);
    }

    @Override
    public void periodic() {

        // If the subsystem is emergency stopped, don't run any more code.
        if (this.eStopped) {
            return;
        }
    }
}
