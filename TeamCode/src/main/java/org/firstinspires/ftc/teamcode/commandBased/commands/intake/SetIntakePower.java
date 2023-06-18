package org.firstinspires.ftc.teamcode.commandBased.commands.intake;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;

public class SetIntakePower extends TriggerCommandBase {

    private final IntakeSubsystem m_intakeSubsystem;
    private final double power;

    public SetIntakePower(IntakeSubsystem intakeSubsystem, double power) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
        this.power = power;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean isTriggered() {
        return true;
    }
}