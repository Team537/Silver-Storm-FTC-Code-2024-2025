package org.firstinspires.ftc.teamcode.Subsystems.Arm;

public enum ArmPositions {
    FLOOR_POSITION(0),
    LOW_BASKET(0),
    HIGH_BASKET(2000);

    final int ENCODER_POSITION;
    ArmPositions(int encoderPosition) {
        this.ENCODER_POSITION = encoderPosition;
    }

    /**
     * Returns the encoder position associated with this arm position.
     *
     * @return The encoder position associated with this arm position.
     */
    public int encoderPosition() {
        return this.ENCODER_POSITION;
    }
}