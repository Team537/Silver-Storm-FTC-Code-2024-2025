package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public class PreviousCommandDirective implements Directive {
    private String commandID;

    // TODO: Allow logic that makes it possible for command groups to be used instead of the CommandScheduler.
    /**
     * Create a new PreviousCommandDirective for the specified command ID.
     *
     * @param commandID The ID of the command that is creating this directive.
     */
    public PreviousCommandDirective(String commandID) {
        this.commandID = commandID;
    }

    /**
     * Sets this directive's command ID.
     *
     * @param commandID The ID of the command that this directive is contained within.
     */
    public void setCommandID(String commandID) {
        this.commandID = commandID;
    }

    /**
     * Returns the command that was scheduled before this command, or the "Previous Command". If this is the
     * first command, a stop command will be returned instead.
     *
     * @return The command that was scheduled before this command, or the "Previous Command".
     * @throws UnscheduledCommandException If this directive's listed command ID is outdated or hasn't been scheduled.
     */
    @Override
    public CommandBase execute() throws UnscheduledCommandException {

        // Get the command scheduler so that necessary actions can be reformed.
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Get the index of the currently running command.
        int activeCommandIndex = commandScheduler.getCommandIndex(commandID);

        // Get the command at the active command's index minus 1. This is the previous command.
        CommandBase nextCommand = commandScheduler.getCommandAtIndex(activeCommandIndex - 1);

        // If no command exists at the previous command's index, create a new StopCommand and return that instead,
        // since there isn't any other command to run.
        if (nextCommand == null) {
            nextCommand = new StopCommand();
        }

        // Return the next command.
        return nextCommand;
    }
}
