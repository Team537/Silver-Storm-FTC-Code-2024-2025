package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

/** TODOLIST:
 * SkipForwards
 * Repeat
 * Terminate
 * E_Terminate
 */
public interface Directive {
    CommandBase execute() throws UnscheduledCommandException;
}
