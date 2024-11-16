package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;

public class Arm implements Subsystem {

    // Settings
    private final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.6135681656;

    // Storage
    private DcMotorEx armMotor;
    private ArmPositions currentTargetArmPosition;
    private Telemetry telemetry;
    private PIDController pidController;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup the hardware.
        setupHardware(hardwareMap);

        // Set the current arm position.
        currentTargetArmPosition = ArmPositions.LOW_BASKET;

        // Store the telemetry for later use.
        this.telemetry = telemetry;
    }

    private void setupHardware(HardwareMap hardwareMap)  {

        // Create te code object for the motor.
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run the arm motor using the encoder so that we can use the encoders to preform various tasks.
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(ArmPositions armPositions) {
        currentTargetArmPosition = armPositions;
    }

    public void eStop() {
        armMotor.setPower(0);
        armMotor.setTargetPosition(0);
    }

    @Override
    public void periodic() {
        telemetry.addLine("~~~~~ Arm ~~~~~");
        telemetry.addData("ArmPosition", armMotor.getCurrentPosition());
        telemetry.addData("ArmVelocity", armMotor.getVelocity());

        double pidValue = pidController.update(currentTargetArmPosition.ENCODER_POSITION, armMotor.getCurrentPosition());
        armMotor.setVelocity(pidValue * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }
}
