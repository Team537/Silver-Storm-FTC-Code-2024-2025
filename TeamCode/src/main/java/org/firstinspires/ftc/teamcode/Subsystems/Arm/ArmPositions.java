package org.firstinspires.ftc.teamcode.Subsystems.Arm;

public enum ArmPositions {
    FLOOR_POSITION(-1.0472),
    CENTER_CROSS (-0.663225),
    LOW_BASKET(0.959931),
    HIGH_BASKET(1.309);

    final double ARM_ANGLE_RADIANS;
    ArmPositions(double armAngleRadians) {
        this.ARM_ANGLE_RADIANS = armAngleRadians;
    }

    /**
     * Returns the arm angle associated with this arm position.
     *
     * @return The arm angle associated with this arm position.
     */
    public double getArmAngleRadians() {
        return this.ARM_ANGLE_RADIANS;
    }
}