package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Events.EventCommand;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The CommandScheduler is responsible for managing the execution, scheduling, and termination of commands in the system.
 * It handles active commands, ensures commands do not interfere with each other, and coordinates command execution order.
 */
public class CommandScheduler implements CommandManager {

    // Singleton instance of the CommandScheduler.
    private static CommandScheduler instance;

    // Maps to track scheduled commands, their states, and subsystem usage.
    private final Map<String, CommandBase> SCHEDULED_COMMANDS = new LinkedHashMap<>();
    private final Map<String, CommandState> COMMAND_STATES = new HashMap<>();
    private final Map<Subsystem, List<CommandState>> SUBSYSTEM_USAGE_MAP = new HashMap<>();

    // Maps to track event commands.
    private final Map<String, EventCommand> SCHEDULED_EVENT_COMMANDS = new HashMap<>();

    // Set to track currently active commands.
    private final Set<String> ACTIVE_COMMANDS = new HashSet<>();
    private final Set<String> ACTIVE_EVENT_COMMANDS = new HashSet<>();

    // Flag to handle emergency stop condition.
    private boolean emergencyStop = false;

    /**
     * Private constructor to prevent direct instantiation. The CommandScheduler follows the Singleton pattern.
     */
    private CommandScheduler() {}

    /**
     * Returns the single instance of the CommandScheduler, creating it if it doesn't exist yet.
     * This ensures there is only one instance managing command scheduling.
     *
     * @return The CommandScheduler instance.
     */
    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    @Override
    public void stopAll() {
        stopCommands(ACTIVE_COMMANDS, true);
    }

    /**
     * Stops the specified commands and marks them as stopped.
     *
     * @param commandsToRemove A set of command IDs to stop.
     * @param interrupted A flag indicating whether the stop was due to an interruption.
     */
    public void stopCommands(Set<String> commandsToRemove, boolean interrupted) {
        for (String commandID : commandsToRemove) {
            CommandBase command = SCHEDULED_COMMANDS.get(commandID);
            CommandState commandState = COMMAND_STATES.get(commandID);

            if (commandState != null) {
                commandState.setStatus(CommandStatus.STOPPED);
                command.end(interrupted);
                ACTIVE_COMMANDS.remove(commandID);
            }
        }
    }

    /**
     * Activates the specified commands and sets their status to RUNNING.
     *
     * @param commandsToSchedule A set of command IDs to activate.
     */
    public void activateCommands(Set<String> commandsToSchedule) throws UnscheduledCommandException {
        for (String commandID : commandsToSchedule) {
            CommandBase command = SCHEDULED_COMMANDS.get(commandID);
            CommandState commandState = COMMAND_STATES.get(commandID);

            if (commandState != null && canRunCommand(command)) {
                commandState.setStatus(CommandStatus.RUNNING);
                command.init();
                ACTIVE_COMMANDS.add(commandID);
            }
        }
    }

    /**
     * Toggles the state of an event command (either starting or stopping it).
     *
     * If the command is currently active, it will be stopped, and if it's inactive, it will be started.
     *
     * @param commandID The unique identifier of the command to toggle.
     * @throws UnscheduledCommandException If the command is not scheduled.
     */
    public void toggleEventCommand(String commandID) throws UnscheduledCommandException {

        // Retrieve the EventCommand associated with the given commandID from the scheduled commands map
        EventCommand eventCommand = SCHEDULED_EVENT_COMMANDS.get(commandID);

        // Retrieve the current state of the command (running or stopped) from the command states map
        CommandState commandState = COMMAND_STATES.get(commandID);

        // Check if the command is currently active
        if (ACTIVE_EVENT_COMMANDS.contains(commandID)) {

            // If active, stop the command
            commandState.setStatus(CommandStatus.STOPPED); // Mark the command as stopped in the state map
            eventCommand.end(false); // End the command execution (pass false to indicate it's not a forced stop)
            ACTIVE_EVENT_COMMANDS.remove(commandID); // Remove the command from the list of active commands
        } else {

            // If inactive, check if the command is allowed to run (canRunCommand verifies if conditions are met)
            if (canRunCommand(eventCommand)) {
                commandState.setStatus(CommandStatus.RUNNING); // Mark the command as running in the state map
                eventCommand.init(); // Initialize the command (e.g., set up necessary state or resources)
                ACTIVE_EVENT_COMMANDS.add(commandID); // Add the command to the list of active commands
            }
        }
    }

    /**
     * Schedules a command to run with a specified priority and subsystem requirements.
     * The command will be executed during autonomous operation based on its priority and subsystem availability.
     *
     * @param command The command to be scheduled.
     * @param priority The priority of the command. Higher priority commands override lower priority ones.
     * @param requiredSubsystems Subsystems that the command requires for execution.
     */
    @Override
    public void scheduleCommand(CommandBase command, int priority, Subsystem... requiredSubsystems) {

        // Add command to the schedule.
        if (command instanceof EventCommand) {
            SCHEDULED_EVENT_COMMANDS.put(command.getID(), (EventCommand) command);
        } else {
            SCHEDULED_COMMANDS.put(command.getID(), command);
        }

        // Create and store the command's state.
        CommandState commandState = new CommandState(command.getID(), new HashSet<>(Arrays.asList(requiredSubsystems)), CommandStatus.STOPPED, priority);
        COMMAND_STATES.put(command.getID(), commandState);

        // Update the subsystem usage map with the required subsystems.
        for (Subsystem subsystem : requiredSubsystems) {
            SUBSYSTEM_USAGE_MAP.computeIfAbsent(subsystem, k -> new ArrayList<>()).add(commandState);
        }
    }

    /**
     * Executes all active commands, checks for completion, and schedules the next command to run.
     * Handles emergency stop condition and manages command lifecycle.
     *
     * @throws UnscheduledCommandException If a command references an unscheduled command.
     * @throws NullCommandException If a command returns null for the next command to execute.
     */
    public void execute() throws UnscheduledCommandException, NullCommandException {

        // Keep track of the ID's of commands that need to be removed or added from the list of active commands.
        Set<String> commandRemovalQueue = new HashSet<>();
        Set<String> commandActivationQueue = new HashSet<>();

        // Run event commands.
        executeEventCommands();

        // Loop through all of the currently active commands and run or stop them if necessary.
        for (String activeCommandID : ACTIVE_COMMANDS) {

            // If told to emergency stop, immediately halt operations.
            if (emergencyStop) {
                stopAll();
                return;
            }

            // Attempt to locate the command with the given ID.
            CommandBase activeCommand = SCHEDULED_COMMANDS.get(activeCommandID);

            // Throw an error if no command exists with the given ID.
            if (activeCommand == null) {
                throw new UnscheduledCommandException(activeCommandID);
            }

            // Check whether or not the command has finished running.
            if (activeCommand.isFinished()) {

                // Update the ending command's status and add it to the queue of commands to remove.
                CommandState activeCommandState = COMMAND_STATES.get(activeCommand.getID());
                activeCommandState.setStatus(CommandStatus.STOPPED);
                commandRemovalQueue.add(activeCommandID);

                // Get the next command that should be run.
                CommandBase nextCommand = getNextCommand(activeCommand);

                // If the nextCommand isn't a stop command, call the nextCommand's init method and
                // add it to the command activation queue.
                if (canRunCommand(nextCommand)) {
                    commandActivationQueue.add(nextCommand.getID());
                }

                // Skip to the next command.
                continue;
            }

            // Run the active command.
            activeCommand.execute();
        }

        stopCommands(commandRemovalQueue, false);
        activateCommands(commandActivationQueue);
    }

    /**
     * Executes all the currently active event commands.
     *
     * This method iterates through the list of active commands and executes them.
     * It is called periodically to run all commands that are currently active.
     *
     * @throws NullCommandException If a command in the scheduled list is null.
     * @throws UnscheduledCommandException If a command in the active list has not been scheduled.
     */
    private void executeEventCommands() throws NullCommandException, UnscheduledCommandException {

        // Iterate through each active event command ID in the active commands list
        for (String commandID : ACTIVE_EVENT_COMMANDS) {

            // Retrieve the EventCommand associated with the given commandID
            EventCommand eventCommand = SCHEDULED_EVENT_COMMANDS.get(commandID);

            // Execute the command (actual work or action of the command)
            eventCommand.execute();
        }
    }

    /**
     * Determines the next command to run after the completion of a given command.
     *
     * @param terminatingCommand The command that has finished execution.
     * @return The next command to execute.
     * @throws UnscheduledCommandException If the next command is not scheduled.
     * @throws NullCommandException If the next command is null.
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
     * Checks if the next command can be executed based on the current command states
     * and subsystem requirements.
     *
     * @param nextCommand The command to be evaluated for execution.
     * @return true if the command can run, false otherwise.
     * @throws UnscheduledCommandException if the command is not scheduled.
     */
    private boolean canRunCommand(CommandBase nextCommand) throws UnscheduledCommandException {

        // If the next command is a stop command, it cannot be run.
        if (nextCommand instanceof StopCommand) {
            return false;
        }

        // Retrieve the command state associated with the next command.
        CommandState nextCommandState = COMMAND_STATES.get(nextCommand.getID());

        // If no command state exists for the next command, it is considered unscheduled.
        if (nextCommandState == null) {
            throw new UnscheduledCommandException(nextCommand.getID());
        }

        // Get the subsystems required by the next command.
        Set<Subsystem> requiredSubsystems = nextCommandState.getRequiredSubsystems();

        // If no subsystems are required, the command can always run.
        if (requiredSubsystems == null || requiredSubsystems.isEmpty()) {
            return true;
        }

        // Flag to track if the command can run.
        boolean canRun = true;

        // Set to track commands that need to be terminated due to conflicting subsystem usage.
        Set<String> commandTerminationQueue = new HashSet<>();

        // Loop through all required subsystems for the next command.
        for (Subsystem requiredSubsystem : requiredSubsystems) {

            // Retrieve all command states that are using the current subsystem.
            List<CommandState> commandStates = SUBSYSTEM_USAGE_MAP.get(requiredSubsystem);

            // Ensure that the command states list is not null.
            assert commandStates != null;

            // Iterate through all command states using the current subsystem.
            for (CommandState commandState : commandStates) {

                // Skip if the current command state is the same as the next command's state.
                if (nextCommandState.equals(commandState)) {
                    continue;
                }

                // Skip if the current command state is not associated with a running command.
                if (commandState.getStatus() != CommandStatus.RUNNING) {
                    continue;
                }

                // If the next command has a lower priority than the currently iterated command, then do not run this command.
                if (nextCommandState.getPriority() < commandState.getPriority()) {
                    canRun = false;
                    break;
                }

                // If command is a higher priority than the currently iterated command, stop the currently iterated command.
                if (nextCommandState.getPriority() > commandState.getPriority()) {
                    commandTerminationQueue.add(commandState.getCommandID());
                }
            }

            // If the command can no longer run, break out of the loop early.
            if (!canRun) {
                break;
            }
        }

        // Stop all of the commands that interfere
        stopCommands(commandTerminationQueue, false);

        // Return whether the command can run or not.
        return canRun;
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