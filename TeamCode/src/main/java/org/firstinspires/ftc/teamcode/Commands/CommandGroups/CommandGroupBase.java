package org.firstinspires.ftc.teamcode.Commands.CommandGroups;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandManager;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

import java.util.LinkedHashMap;
import java.util.Map;

public abstract class CommandGroupBase extends CommandBase implements CommandManager {

    protected final Map<String, CommandBase> SCHEDULED_COMMANDS = new LinkedHashMap<>();

    /**
     * Returns the command with the given ID.
     *
     * @param commandID The ID of the command that will be obtained.
     * @return The command with the given ID.
     * @throws UnscheduledCommandException Thrown if there isn't a scheduled command with the given ID.
     */
    @Override
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
    @Override
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
