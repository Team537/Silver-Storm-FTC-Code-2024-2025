package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandManager;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public class NextCommandDirective implements Directive {
    private String commandID;
    private CommandManager commandManager;

    /**
     * Create a new NextCommandDirective for the specified command ID.
     *
     * @param commandID The ID of the command that is creating this directive.
     * @param parentCommand The parent command that is running this command. If the command scheduler is running this command,
     *                      use the null keyword.
     */
    public NextCommandDirective(String commandID, CommandManager parentCommand) {
        this.commandID = commandID;

        // If the parent command is null, use the command scheduler.
        // Otherwise, use the given parent command for the following operations.
        if (parentCommand == null) {
            this.commandManager = CommandScheduler.getInstance();
        } else {
            this.commandManager = parentCommand;
        }
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
     * Returns the command that was scheduled after this command, or the "Next Command". If this is the
     * last command, a stop command will be returned instead.
     *
     * @return The command that was scheduled after this command, or the "Next Command".
     * @throws UnscheduledCommandException If this directive's listed command ID is outdated or hasn't been scheduled.
     */
    @Override
    public CommandBase execute() throws UnscheduledCommandException {

        // Get the index of the currently running command.
        int activeCommandIndex = commandManager.getCommandIndex(commandID);

        // Get the command at the active command's index plus 1. This is the next command.
        CommandBase nextCommand = commandManager.getCommandAtIndex(activeCommandIndex + 1);

        // If no command exists at the next command's index, create a new StopCommand and return that instead,
        // since the commands have finished running.
        if (nextCommand == null) {
            nextCommand = new StopCommand();
        }

        // Return the next command.
        return nextCommand;
    }
}
