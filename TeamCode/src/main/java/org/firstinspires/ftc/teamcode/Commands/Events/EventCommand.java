package org.firstinspires.ftc.teamcode.Commands.Events;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandResult;
import org.firstinspires.ftc.teamcode.Commands.Directives.TerminateDirective;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;


public class EventCommand extends CommandBase {
    @Override
    public void init() {

    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

    /**
     * Event commands shouldn't ent to begin with, so the getCommandResult() will only stop running commands.
     *
     * @return A new CommandResult containing a terminate directive.
     */
    @Override
    final public CommandResult getCommandResult() {
        return CommandResult.fromDirective(new TerminateDirective());
    }
}
