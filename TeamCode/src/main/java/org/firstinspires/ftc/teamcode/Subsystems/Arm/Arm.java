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
    private final double MAX_ANGULAR_SPEED_TICKS_PER_SECOND = 0; // TODO: DERIVE VALUE

    // Subsystem Storage
    private LinearSlides linearSlides;
    private Manipulator manipulator;

    // Storage
    private DcMotorEx armMotor;
    private Telemetry telemetry;

    // Controller Storage
    private PIDController pidController;
    private final double PROPORTIONAL_COEFFICIENT = 0;
    private final double INTEGRAL_COEFFICIENT = 0;
    private final double DERIVATIVE_COEFFICIENT = 0;
    private final double GRAVITATIONAL_COMPENSATION_CONSTANT = 0; // TODO: Tune value
    private final double SLIDE_EXTENSION_CONSTANT = 0; // TODO: Tune value

    // General Storage
    private ArmPositions currentTargetArmPosition;
    private double targetPosition = 0;

    // FLags
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

        // Setup Subsystems.
        this.linearSlides = new LinearSlides(this::getArmAngle);
        this.linearSlides.init(hardwareMap, telemetry);

        this.manipulator = new Manipulator();
        this.manipulator.init(hardwareMap, telemetry);

        // Setup the hardware.
        setupHardware(hardwareMap);

        // Set the current arm position.
        this.currentTargetArmPosition = ArmPositions.LOW_BASKET;

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

    /**
     * Set the current position that the arm should target.
     *
     * @param armPositions The current position that the arm should target.
     */
    public void setTargetPosition(ArmPositions armPositions) {
        this.currentTargetArmPosition = armPositions;
        this.targetPosition = armPositions.encoderPosition();
    }

    /**
     * Recalculate the motor's power based on the arm's distance form its target position.
     */
    public void calculateNewMotorPower() {

        // Calculate feedback based on the distance towards the target position.
        double calculatedFeedbackMotorPower = this.pidController.update(this.targetPosition, this.armMotor.getCurrentPosition());

        // Acquire necessary values to counteract both gravity and the slide extension length.
        double armAngle = getArmAngle();
        double slideExtensionLength = this.linearSlides.getExtensionDistance();

        // Calculate the feedforwards value to compensate for slide extension length and gravity.
        double feedforwards = (GRAVITATIONAL_COMPENSATION_CONSTANT * Math.cos(armAngle)) +
                (SLIDE_EXTENSION_CONSTANT * slideExtensionLength);

        // Calculate the final motor power.
        double motorPower = feedforwards + calculatedFeedbackMotorPower;

        // Set the motor power of the arm motor.
        this.armMotor.setPower(motorPower);
    }

    /**
     * Stops all hardware present within this subsystem.
     */
    public void eStop() {
        this.armMotor.setPower(0);
        this.linearSlides.eStop();
        this.manipulator.eStop();
    }

    /**
     * Returns the angle between the ground and the arm.
     *
     * @return The angle between the ground and the arm.
     */
    public double getArmAngle() {
        return 0;
    }

    /**
     * Return this arm's linear slides.
     *
     * @return This arm's linear slides.
     */
    public LinearSlides getLinearSlides() {
        return this.linearSlides;
    }

    /**
     * Return this arm's manipulator.
     *
     * @return This arm's manipulator.
     */
    public Manipulator getManipulator() {
        return manipulator;
    }

    @Override
    public void periodic() {

        // If emergency stopped, do not run any additional code.
        if (eStopped) {
            return;
        }

        // Update this subsystem's subsystems.
        this.linearSlides.periodic();
        this.manipulator.periodic();;

        // Update motor velocity based on current system state.
        calculateNewMotorPower();
    }
}
