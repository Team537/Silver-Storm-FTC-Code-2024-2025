package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.Set;

public class CommandState {
    private String commandID;
    private Set<Subsystem> requiredSubsystems;
    private CommandStatus status;
    private int priority;

    /**
     * Initializes a new CommandState with the specified command details.
     *
     * @param commandID The unique identifier for this command.
     * @param requiredSubsystems A set of subsystems that this command requires.
     * @param status The current status of the command (e.g., RUNNING, STOPPED, PAUSED).
     * @param priority The priority of this command. Commands with higher priority will override those with lower priority.
     *                 If two commands have the same priority, the most recently scheduled command will take precedence.
     */
    public CommandState(String commandID, Set<Subsystem> requiredSubsystems, CommandStatus status, int priority) {
        this.commandID = commandID;
        this.requiredSubsystems = requiredSubsystems;
        this.status = status;
        this.priority = priority;
    }

    /**
     * Sets the current status of this command.
     *
     * @param commandStatus The new status of the command (e.g., RUNNING, STOPPED, PAUSED).
     */
    public void setStatus(CommandStatus commandStatus) {
        this.status = commandStatus;
    }

    /**
     * Retrieves the unique identifier for this command.
     *
     * @return The command's unique identifier as a string.
     */
    public String getCommandID() {
        return this.commandID;
    }

    /**
     * Retrieves the set of subsystems that this command requires to operate.
     *
     * @return A set of the required subsystems.
     */
    public Set<Subsystem> getRequiredSubsystems() {
        return this.requiredSubsystems;
    }

    /**
     * Retrieves the current status of this command.
     *
     * @return The current status of the command (e.g., RUNNING, STOPPED, PAUSED).
     */
    public CommandStatus getStatus() {
        return this.status;
    }

    /**
     * Retrieves the priority level of this command.
     *
     * @return The priority of the command, where higher values indicate higher priority.
     */
    public int getPriority() {
        return this.priority;
    }
}