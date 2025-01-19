package org.firstinspires.ftc.teamcode.Commands.CommandGroups;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandState;
import org.firstinspires.ftc.teamcode.Commands.CommandStatus;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * A command group that can run multiple commands simultaneously. This command group does not allow
 * for the usage of commands or command directives, so keep that in mind.
 */
public class ParallelCommandGroup extends CommandGroupBase {

    // Maps to track scheduled commands, their states, and subsystem usage.
    private final Map<String, CommandState> COMMAND_STATES = new HashMap<>();
    private final Map<Subsystem, List<CommandState>> SUBSYSTEM_USAGE_MAP = new HashMap<>();

    // Keep track of active commands.
    private final Set<String> ACTIVE_COMMAND_IDS = new HashSet<>();

    // Flags
    private boolean isFinished = false;
    private boolean emergencyStop = false;

    @Override
    public void init() {

        // The command just started running.
        this.isFinished = false;

        // Activate all scheduled commands.
        try {
            activateCommands(this.SCHEDULED_COMMANDS.keySet());
        } catch (UnscheduledCommandException e) {
            throw new RuntimeException(e);
        }
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
                this.ACTIVE_COMMAND_IDS.remove(commandID);
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
                this.ACTIVE_COMMAND_IDS.add(commandID);
            }
        }
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // Keep track of the ID's of commands that need to be removed or added from the list of active commands.
        Set<String> commandRemovalQueue = new HashSet<>();

        // If there aren't any active commands left, this command group has finished running.
        if (this.ACTIVE_COMMAND_IDS.isEmpty()) {
            this.isFinished = true;
            return;
        }

        // Loop through all of the currently active commands and run or stop them if necessary.
        for (String activeCommandID : this.ACTIVE_COMMAND_IDS) {

            // If told to emergency stop, immediately halt operations.
            if (emergencyStop) {
                stopAll();
                return;
            }

            // Attempt to get the active command from the command ID.
            CommandBase activeCommand = this.SCHEDULED_COMMANDS.get(activeCommandID);

            // Throw an error if no command exists with the given ID.
            if (activeCommand == null) {
                throw new UnscheduledCommandException(activeCommandID);
            }

            // Check whether or not the command has finished running.
            if (activeCommand.isFinished()) {

                // Add the command's ID to the list of commands to be removed.
                commandRemovalQueue.add(activeCommandID);

                // Skip to the next command.
                continue;
            }

            // Run the active command.
            activeCommand.execute();
        }

        // Stop all commands that have been requested to be stopped.
        stopCommands(commandRemovalQueue, false);
    }

    /**
     * Checks if the next command can be executed based on the current command states
     * and subsystem requirements.
     *
     * @param command The command to be evaluated for execution.
     * @return true if the command can run, false otherwise.
     * @throws UnscheduledCommandException if the command is not scheduled.
     */
    private boolean canRunCommand(CommandBase command) throws UnscheduledCommandException {

        // If the next command is a stop command, it cannot be run.
        if (command instanceof StopCommand) {
            return false;
        }

        // Retrieve the command state associated with the next command.
        CommandState nextCommandState = COMMAND_STATES.get(command.getID());

        // If no command state exists for the next command, it is considered unscheduled.
        if (nextCommandState == null) {
            throw new UnscheduledCommandException(command.getID());
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

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void scheduleCommand(CommandBase command, int priority, Subsystem... requiredSubsystems) {

        // Set the command's parent.
        command.setParent(this);

        // Add command to the schedule.
        SCHEDULED_COMMANDS.put(command.getID(), command);

        // Create and store the command's state.
        CommandState commandState = new CommandState(command.getID(), new HashSet<>(Arrays.asList(requiredSubsystems)), CommandStatus.STOPPED, priority);
        COMMAND_STATES.put(command.getID(), commandState);

        // Update the subsystem usage map with the required subsystems.
        for (Subsystem subsystem : requiredSubsystems) {
            SUBSYSTEM_USAGE_MAP.computeIfAbsent(subsystem, k -> new ArrayList<>()).add(commandState);
        }
    }

    /**
     * Stop all actively running commands.
     */
    @Override
    public void stopAll() {
        emergencyStop = true;
        stopCommands(this.ACTIVE_COMMAND_IDS, true);
    }
}
