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
    private ServoController wristServoController;

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
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        wristServo = hardwareMap.get(CRServo.class, "wristServo");
        wristServoController = wristServo.getController();
        wristServoController.pwmDisable(); // Enable position mode.
        // wristServo.setDirection(DcMotorSimple.Direction.REVERSE); TODO: Check if needed
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
        this.wristServoController.setServoPosition(0, this.targetPosition.getWristAnglePosition());
    }
    /**
     * Adjusts the hardware such that it begins intaking samples.
     */
    public void intake() {
        //intakeServo.setPower(1);
        wristServo.setPower(0.5);
    }

    /**
     * Adjusts the hardware such that it begins outtaking samples.
     */
    public void outtake() {
       // intakeServo.setPower(-1);
        wristServo.setPower(-0.5);
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

        // Output the wrist's current position.
        telemetry.addLine("Wrist Position: " + wristServoController.getServoPosition(wristServo.getPortNumber()));
        telemetry.addLine("Wrist Position: " + wristServoController.getServoPosition(wristServo.getPortNumber()));
    }
}
