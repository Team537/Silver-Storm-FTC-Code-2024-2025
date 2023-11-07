package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Testing")
public class TestingStuff extends RobotOpMode {
    @Override
    public void init() {
        super.init();

        final int NEW_SERVO_POSITION = 180;

        //moveServo(armCatcherServo, NEW_SERVO_POSITION);
    }
    @Override
    public void robotLoop() {
        //gamePadMoveRobot();
        float ArmServoPositionModifier = 0;
        if (gamepad1.dpad_left) {
            ArmServoPositionModifier += 0.1;
        }
        if (gamepad1.dpad_right) {
            ArmServoPositionModifier -= 0.1;
        }
        double WristServoPositionModifier = 0;
        if (gamepad1.dpad_up) {
            WristServoPositionModifier += 0.1;
        }
        if (gamepad1.dpad_down) {
            WristServoPositionModifier -= 0.1;
        }

        elapsedTime.reset();
        while(elapsedTime.seconds() < 0.1) {
            dbp.put("robotLoop", "Waiting");
        }

        //moveServo(armCatcherServo, (armCatcherServoPosition + ArmServoPositionModifier));
        moveServo(wristServo, (int) (wristServoPosition + WristServoPositionModifier));

        if (gamepad1.a) {
            armExtensionMotor.setPower(1);
        } else if (gamepad1.b) {
            armExtensionMotor.setPower(-1);
        } else {
            armExtensionMotor.setPower(0);
        }
        armMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        /*
        if(gamepad1.left_bumper) {
            linearMoveArm(1f, ArmServoPosition.CLOSED);
        } else if(gamepad1.right_bumper) {
            linearMoveArm(1f, ArmServoPosition.OPEN);
        }

        if(!armMotor.isBusy()) {
            armMotor.setPower(0);
        }
         */
    }
}
