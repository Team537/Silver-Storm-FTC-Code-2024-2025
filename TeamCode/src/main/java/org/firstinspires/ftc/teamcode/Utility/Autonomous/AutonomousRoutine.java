package org.firstinspires.ftc.teamcode.Utility.Autonomous;

import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;

public enum AutonomousRoutine {
    RED_ONE (Alliance.RED, new Pose2d(1.614148, 0.413513, new Rotation2d(-Math.PI, 0))),
    RED_TWO (Alliance.RED, new Pose2d()),
    BLUE_ONE (Alliance.BLUE, new Pose2d(-1.614148, 0.413513, new Rotation2d(0, 0))),
    BLUE_TWO (Alliance.BLUE, new Pose2d());

    private final Alliance ALLIANCE;
    private final Pose2d STARTING_POSITION;
    AutonomousRoutine(Alliance alliance, Pose2d startingPosition) {
        this.ALLIANCE = alliance;
        this.STARTING_POSITION = startingPosition;
    }

    /**
     * Return the alliance that this autonomous routine belongs to.
     *
     * @return The alliance that this autonomous routine belongs to.
     */
    public Alliance getAlliance() {
        return this.ALLIANCE;
    }

    /**
     * Returns the starting position of this autonomous routine.
     *
     * @return The starting position of this autonomous routine.
     */
    public Pose2d getStartingPosition() {
        return this.STARTING_POSITION;
    }
}
