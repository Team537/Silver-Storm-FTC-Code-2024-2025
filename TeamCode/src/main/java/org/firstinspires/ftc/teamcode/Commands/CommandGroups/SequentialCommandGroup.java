package org.firstinspires.ftc.teamcode.Commands.CommandGroups;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandResult;
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

public class SequentialCommandGroup extends CommandGroupBase {

    // Maps to track scheduled commands, their states, and subsystem usage.
    private final Map<String, CommandState> COMMAND_STATES = new HashMap<>();
    private final Map<Subsystem, List<CommandState>> SUBSYSTEM_USAGE_MAP = new HashMap<>();

    // Storage
    private String activeCommandID;
    private String startingCommandID;

    // Flags
    private boolean isFinished = false;
    private boolean emergencyStop = false;

    @Override
    public void init() {

        // The command just started running.
        this.isFinished = false;

        // Setup the starting command for the command group.
        // If a valid starting command has been provided, use that.
        // Otherwise use the first command added to the group.
        if (startingCommandID != null && this.SCHEDULED_COMMANDS.get(startingCommandID) != null) {
            this.activeCommandID = startingCommandID;
        } else {
            this.activeCommandID = getCommandAtIndex(0).getID();
        }
    }

    /**
     * Sets which command will run first when this command group is run.
     *
     * @param startingCommandID The ID of the command that will be run first.
     */
    public void setStartingCommandID(String startingCommandID) {
        this.startingCommandID = startingCommandID;
    }

    /**
     * Stops the specified commands and marks it as stopped.
     *
     * @param commandToRemove The ID of the command to stop.
     * @param interrupted A flag indicating whether the stop was due to an interruption.
     */
    public void stopCommand(String commandToRemove, boolean interrupted) {
        CommandBase command = SCHEDULED_COMMANDS.get(commandToRemove);
        CommandState commandState = COMMAND_STATES.get(commandToRemove);

        // Stops the command if it can be found.
        if (commandState != null && command != null) {
            commandState.setStatus(CommandStatus.STOPPED);
            command.end(interrupted);
            activeCommandID = null;
        }
    }

    /**
     * Activates the specified command and sets its status to RUNNING.
     *
     * @param commandToSchedule The ID of the command to activate.
     */
    public void activateCommand(String commandToSchedule) {
        CommandBase command = SCHEDULED_COMMANDS.get(commandToSchedule);
        CommandState commandState = COMMAND_STATES.get(commandToSchedule);

        // Starts the command if it can be found.
        if (commandState != null && command != null) {
            commandState.setStatus(CommandStatus.RUNNING);
            command.init();
            activeCommandID = commandToSchedule;
        }
    }

    @Override
    public void execute() throws NullCommandException, UnscheduledCommandException {

        // If there isn't a active command, this command group has finished running.
        if (this.activeCommandID == null) {
            this.isFinished = true;
            return;
        }

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

            // Get the next command that should be run.
            CommandBase nextCommand = getNextCommand(activeCommand);

            // Stop the active command.
            stopCommand(activeCommandID, false);

            // If the next command is a stop command, this command group has finished running.
            if (nextCommand instanceof StopCommand) {
                isFinished = true;
                return;
            }

            // Start the next command.
            activateCommand(nextCommand.getID());

            // Stop running the rest of the code, since the formerly active command is no longer active.
            return;
        }

        // Run the active command.
        activeCommand.execute();
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
            nextCommand = this.SCHEDULED_COMMANDS.get(commandResult.getCommandID());

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
        this.SCHEDULED_COMMANDS.put(command.getID(), command);

        // Create and store the command's state.
        CommandState commandState = new CommandState(command.getID(), new HashSet<>(Arrays.asList(requiredSubsystems)), CommandStatus.STOPPED, priority);
        this.COMMAND_STATES.put(command.getID(), commandState);

        // Update the subsystem usage map with the required subsystems.
        for (Subsystem subsystem : requiredSubsystems) {
            this.SUBSYSTEM_USAGE_MAP.computeIfAbsent(subsystem, k -> new ArrayList<>()).add(commandState);
        }
    }

    /**
     * Stop all actively running commands.
     */
    @Override
    public void stopAll() {
        emergencyStop = true;
        stopCommand(activeCommandID, true);
    }
}
