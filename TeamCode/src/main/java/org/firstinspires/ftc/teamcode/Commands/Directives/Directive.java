package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public interface Directive {
    CommandBase execute() throws UnscheduledCommandException;
}
