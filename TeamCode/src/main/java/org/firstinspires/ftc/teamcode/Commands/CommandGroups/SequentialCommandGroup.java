package org.firstinspires.ftc.teamcode.Commands.CommandGroups;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandResult;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public class SequentialCommandGroup extends CommandGroupBase {

    // Storage
    private CommandBase activeCommand; // Only one command can run at a time.

    // Flags
    private boolean executionComplete = false;

    @Override
    public void init() {

        // The command isn't finished running.
        executionComplete = true;

        // Get the first scheduled command.
        CommandBase firstScheduledCommand = this.getCommandAtIndex(0);

        // Add the first scheduled command to the list of active commands.
        activeCommand = firstScheduledCommand;
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // If all the commands have finished running or doesn't exist, then return.
        if (executionComplete || activeCommand == null) {
            return;
        }

        // Check whether or not the command has finished running.
        if (activeCommand.isFinished()) {

            // Get the next command that should be run.
            CommandBase nextCommand = getNextCommand();

            // If the nextCommand isn't a stop command, call the nextCommand's init method and
            // add it to the command activation queue.
            if (!(nextCommand instanceof StopCommand)) {
                nextCommand.init();
            } else {
                executionComplete = true; // This command group has finished running all of its commands.
            }

            // Call this command's "end" method and add it to the command removal queue.
            activeCommand.end(false);
        } else {

            // Run the active command.
            activeCommand.execute();
        }
    }

    /**
     * Returns which command should be run next according to the active command.
     *
     * @return The command that should run next.
     * @throws UnscheduledCommandException If the a command with the returned command ID that should be run next hasn't been scheduled.
     * @throws NullCommandException If the directive the terminating command gives returns null.
     */
    private CommandBase getNextCommand() throws UnscheduledCommandException, NullCommandException {

        // Get the command result from the terminating command,
        CommandResult commandResult = activeCommand.getCommandResult();
        CommandBase nextCommand;

        // If the CommandResult contains a CommandID, attempt to find a command with the specified ID in
        // the SCHEDULED_COMMANDS LinkedHashMap. If no command can be found, throw an error.
        if (commandResult.isCommand()) {

            // Attempt to get the specified command.
            nextCommand = SCHEDULED_COMMANDS.get(commandResult.getCommandID());

            // If the command with the specified ID hasn't been scheduled, throw an error.
            // Otherwise, return the located command.
            if (nextCommand == null) {
                throw new UnscheduledCommandException(commandResult.getCommandID());
            }
        } else { // The command result contains a directive.
            nextCommand = commandResult.getDirective().execute();

            // If the command with the specified ID hasn't been scheduled, throw an error.
            // Otherwise, return the located command.
            if (nextCommand == null) {
                throw new NullCommandException(activeCommand.getID());
            }
        }

        // Return the next command that should be run.
        return nextCommand;
    }

    @Override
    public boolean isFinished() {
        return executionComplete;
    }

    @Override
    public void end(boolean interrupted) {

        // Stop all of the currently running commands.
        stopAll();
    }

    /**
     * Schedules the given command.
     *
     * @param command The command that will be scheduled.
     */
    @Override
    public void scheduleCommand(CommandBase command) {

        // Set the parent of the added command.
        command.setParentID(this);

        // Schedule the command.
        this.SCHEDULED_COMMANDS.put(command.getID(), command);
    }

    /**
     * Unschedules the given command.
     *
     * @param command The command that will be unscheduled.
     */
    public void unscheduleCommand(CommandBase command) {

        // If the active command is being unscheduled, then run its end function and remove the active command.
        if (this.activeCommand == command) {
            this.activeCommand.end(true);
            this.activeCommand = null;
        }

        // If the command has been scheduled, unschedule it.
        if (this.SCHEDULED_COMMANDS.get(command.getID()) != null) {
            this.SCHEDULED_COMMANDS.remove(command.getID());
        }
    }

    /**
     * Stops the currently running command
     */
    @Override
    public void stopAll() {

        // Stop the currently running command.
        activeCommand.end(true);

        // No command is actively running.
        activeCommand = null;
    }
}
