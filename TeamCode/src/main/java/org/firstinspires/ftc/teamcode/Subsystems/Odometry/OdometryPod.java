package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

public class OdometryPod implements Subsystem {

    // Hardware Storage
    private DcMotor throughBoreEncoder; // Encoders are hooked up to motor ports, so we have to represent each odometry pod as a motor.

    // Settings
    private ElapsedTime elapsedTime;
    private Telemetry telemetry;
    private final int TICKS_PER_REVOLUTION = 8192;
    private final double WHEEL_DIAMETER_METERS = 0.035;
    private final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2;

    // Storage
    private final String ODOMETRY_POD_ENCODER_NAME;
    private double lastEncoderPosition = 0;
    private double currentVelocity = 0;

    /**
     * Create a new OdometryPod object. The odometryPodEncoderName passed through should match the
     * "motor" name on the driver hub.
     *
     * @param odometryPodEncoderName The name of the motor on the driver hub.
     */
    public OdometryPod(String odometryPodEncoderName) {
        this.ODOMETRY_POD_ENCODER_NAME = odometryPodEncoderName;
        this.elapsedTime = new ElapsedTime();
    }
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Store the telemetry for later use.
        this.telemetry = telemetry;

        // Setup this subsystem's hardware.
        setupHardware(hardwareMap);
    }

    /**
     * Setup this odometry pod's hardware.
     *
     * @param hardwareMap The hardware map of the op mode. This is required in order to access the hardware.
     */
    private void setupHardware(HardwareMap hardwareMap) {

        // Get the "motor" for this odometry pod.
        this.throughBoreEncoder = hardwareMap.get(DcMotor.class, this.ODOMETRY_POD_ENCODER_NAME);
    }

    /**
     * Returns the change in position of this odometry pod, in meters.
     *
     * @return The change in position of this odometry pod, in meters.
     */
    public double getPositionalChangeMeters() {

        // Get the current position of the odometry pod's through bore encoder.
        double currentEncoderPosition = this.throughBoreEncoder.getCurrentPosition();

        // Get how far the odometry pod wheel turned.
        double encoderTickChange = currentEncoderPosition - this.lastEncoderPosition;
        double rotationalChangeRadians = (encoderTickChange / TICKS_PER_REVOLUTION) * (2 * Math.PI);

        // Calculate the odometry pod's wheel's positional change.
        double positionalChange = rotationalChangeRadians * WHEEL_RADIUS_METERS;

        // Calculate the average velocity for the heck of it.
        this.currentVelocity = calculateCurrentVelocity(positionalChange);

        // Update the last encoder position value.
        this.lastEncoderPosition = currentEncoderPosition;

        // Return the positional change.
        return positionalChange;
    }

    /**
     * Estimates the robot's velocity given the change in position and time since this method was last called.
     *
     * @param positionalChangeMeters The change in the encoder's position, in meters.
     *
     * @return The average velocity of the meter.
     */
    private double calculateCurrentVelocity(double positionalChangeMeters) {

        // Calculate the average positional change. This value will be more accurate as the change in time decreases.
        double averageVelocity = positionalChangeMeters / elapsedTime.getElapsedTime(TimeUnit.SECOND);

        // Reset the elapsed time.
        this.elapsedTime.reset();

        // Return the velocity.
        return averageVelocity;
    }

    @Override
    public void periodic() {
        telemetry.addLine(this.ODOMETRY_POD_ENCODER_NAME + " Velocity (m/s):" + this.currentVelocity);
    }
}
