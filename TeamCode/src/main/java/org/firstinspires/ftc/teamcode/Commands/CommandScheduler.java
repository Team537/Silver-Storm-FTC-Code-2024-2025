package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class CommandScheduler implements CommandManager {

    // Storage
    private static CommandScheduler instance;

    private final Map<String, CommandBase> SCHEDULED_COMMANDS = new LinkedHashMap<>();
    private final List<String> ACTIVE_COMMANDS = new LinkedList<>();

    // Flags
    private boolean emergencyStop = false;

    /**
     * Private constructor
     */
    private CommandScheduler() {}

    /**
     * This method returns the only existing instance of this class so that it may
     * be used.
     *
     * @return The singular instance of the {@code CommandScheduler} class.
     */
    public static CommandScheduler getInstance() {

        /*
         * Check whether or not an instance of command runtime has been created.
         * If an instance has not yet be created, then create one.
         * This is done to prevent this class from taking up memory when not in use.
         */
        if (instance == null) {
            instance = new CommandScheduler();
        }

        // Return the only instance of this class.
        return instance;
    }

    @Override
    public void stopAll() {

    }

    /**
     * Schedules the given command so that it can be run during autonomous operation.
     *
     * @param command The command to be scheduled.
     */
    @Override
    public void scheduleCommand(CommandBase command) {
        SCHEDULED_COMMANDS.put(command.getID(), command);
    }

    /**
     * Executes all active commands
     * @throws UnscheduledCommandException If a completed command throws the ID of an unscheduled command.
     * @throws NullCommandException If a completed command returns null as the next command.
     */
    public void execute() throws UnscheduledCommandException, NullCommandException {

        // Keep track of the ID's of commands that need to be removed or added from the list of active commands.
        List<String> commandRemovalQueue = new LinkedList<>();
        List<String> commandActivationQueue = new LinkedList<>();

        // Loop through all of the currently active commands and run or stop them if necessary.
        for (String activeCommandId : ACTIVE_COMMANDS) {

            // If told to emergency stop, immediately halt operations.
            if (emergencyStop) {
                break;
            }

            // Attempt to locate the command with the given ID.
            CommandBase activeCommand = SCHEDULED_COMMANDS.get(activeCommandId);

            // Throw an error if no command exists with the given ID.
            if (activeCommand == null) {
                throw new UnscheduledCommandException(activeCommandId);
            }

            // Check whether or not the command has finished running.
            if (activeCommand.isFinished()) {

                // Get the next command that should be run.
                CommandBase nextCommand = getNextCommand(activeCommand);

                // If the nextCommand isn't a stop command, call the nextCommand's init method and
                // add it to the command activation queue.
                if (!(nextCommand instanceof StopCommand)) {
                    nextCommand.init();
                    commandActivationQueue.add(nextCommand.getID());
                }

                // Call this command's "end" method and add it to the command removal queue.
                activeCommand.end(false);
                commandRemovalQueue.add(activeCommandId);

                // Skip to the next command.
                continue;
            }

            // Run the active command.
            activeCommand.execute();
        }

        // Remove all of the commands in the removal queue. This is done before the emergency stop to prevent unexpected behavior.
        ACTIVE_COMMANDS.removeAll(commandRemovalQueue);

        // Add all of the commands in the command activation queue. This is done before emergency stopping everything
        // in order to prevent dumb decisions made inside of init methods from destroying or damaging the robot.
        ACTIVE_COMMANDS.addAll(commandActivationQueue);

        // If requested, stop all running commands.
        if (emergencyStop) {
            this.stopAll();
            return;
        }
    }

    /**
     * Returns which command should be run next according to the terminating command.
     *
     * @param terminatingCommand The command that has finished running.
     * @return The command that should run next.
     * @throws UnscheduledCommandException If the a command with the returned command ID that should be run next hasn't been scheduled.
     * @throws NullCommandException If the directive the terminating command gives returns null.
     */
    private CommandBase getNextCommand(CommandBase terminatingCommand) throws UnscheduledCommandException, NullCommandException {

        // Get the command result from the terminating command,
        CommandResult commandResult = terminatingCommand.getCommandResult();
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
                throw new NullCommandException(terminatingCommand.getID());
            }
        }

        // Return the next command that should be run.
        return nextCommand;
    }

    /**
     * Returns the command with the given ID.
     *
     * @param commandID The ID of the command that will be obtained.
     * @return The command with the given ID.
     * @throws UnscheduledCommandException Thrown if there isn't a scheduled command with the given ID.
     */
    public CommandBase getCommandWithID(String commandID) throws UnscheduledCommandException {

        // Attempt to locate a scheduled command with the given name.
        CommandBase command = SCHEDULED_COMMANDS.get(commandID);
        if (command == null) {
            throw new UnscheduledCommandException(commandID);
        }

        // Return the command with the given ID.
        return command;
    }

    /**
     * Returns the command at the specified index. In the index is our of bounds, null will be returned
     * instead.
     *
     * @param index The index of the command you with to locate.
     * @return The command at the specified index.
     */
    public CommandBase getCommandAtIndex(int index) {

        // Make sure the index isn't out of bounds.
        // If it is, return null, as there is no command at that location.
        if (SCHEDULED_COMMANDS.size() <= index) {
            return null;
        }

        // Keep track of the currently iterated index.
        int currentIndex = 0;

        // Loop through all of the commands in the list of scheduled commands and locate the command
        // at the specified index.
        CommandBase commandAtIndex = null;
        for (Map.Entry<String, CommandBase> entry : SCHEDULED_COMMANDS.entrySet()) {

            // If we are at the desired index, store the command and break from the loop.
            if (currentIndex == index) {
                commandAtIndex = entry.getValue();
                break;
            }

            // Increment the current index by 1.
            currentIndex++;
        }

        // Return the located command.
        return commandAtIndex;
    }

    /**
     * Returns the index of the command with the specified ID.
     *
     * @param commandID The ID of the command
     * @return The index of the command with the specified ID.
     * @throws UnscheduledCommandException If no command has been scheduled with the specified ID.
     */
    @Override
    public int getCommandIndex(String commandID) throws UnscheduledCommandException {

        // Verify that the command with the specified ID exists.
        if (SCHEDULED_COMMANDS.get(commandID) == null) {
            throw new UnscheduledCommandException(commandID);
        }

        // Keep track of the number of commands that have been iterated through.
        int index = 0;

        // Loop through all of the scheduled commands until the currently iterated over command
        // has the same ID as the specified command ID.
        for (Map.Entry<String, CommandBase> entry : SCHEDULED_COMMANDS.entrySet()) {

            // If the currently iterated command has the same ID as the provided command ID, break, and return the index of the command.
            if (entry.getKey().equals(commandID)) {
                break;
            }

            // Increase index by 1.
            index++;
        }

        // Return the index of the specified command.
        return index;
    }
}