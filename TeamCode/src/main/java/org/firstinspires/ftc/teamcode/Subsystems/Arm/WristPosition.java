package org.firstinspires.ftc.teamcode.Subsystems.Arm;

public enum WristPosition {
    START_POSITION (0),
    INTAKE (0),
    SPECIMEN_SCORING (0),
    HIGH_BASKET_SCORING (0),
    LOW_BASKET_SCORING (0);

    final double WRIST_ANGLE_POSITION;
    WristPosition(double wristAngle) {
        this.WRIST_ANGLE_POSITION = wristAngle;
    }

    /**
     * Returns the wrist angle associated with this wrist position.
     *
     * @return The wrist angle associated with this wrist position.
     */
    public double getWristAnglePosition() {
        return this.WRIST_ANGLE_POSITION;
    }
}
