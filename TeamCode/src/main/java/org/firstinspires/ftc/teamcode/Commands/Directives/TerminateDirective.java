package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;

public class TerminateDirective implements Directive {

    /**
     * Return a new stop command, ending the current chain of commands.
     *
     * @return A new stop command, ending the current chain of commands.
     */
    @Override
    public CommandBase execute() {
        return new StopCommand();
    }
}
