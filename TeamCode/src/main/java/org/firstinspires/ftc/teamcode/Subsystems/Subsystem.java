package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Subsystem {

    void init(HardwareMap hardwareMap, Telemetry telemetry);
    void periodic();
}
