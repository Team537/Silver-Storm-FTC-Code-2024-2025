package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Events.EventCommand;
import org.firstinspires.ftc.teamcode.Commands.Events.EventListeners.EventListener;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class EventManager {

    // Singleton instance of the EventManager.
    private EventManager instance;

    // Storage for mapping event listeners to commands and tracking active commands.
    private final Map<EventListener, EventCommand> EVENT_MAP = new HashMap<>();
    private CommandScheduler commandScheduler;  // The command scheduler instance to manage command execution.
    private final Set<EventCommand> ACTIVE_EVENT_COMMANDS = new HashSet<>();  // A set to track currently active event commands.

    /**
     * Private constructor to ensure singleton pattern.
     * Initializes the command scheduler instance.
     */
    private EventManager() {
        this.commandScheduler = CommandScheduler.getInstance(); // Gets the single instance of CommandScheduler.
    }

    /**
     * Returns the singleton instance of EventManager.
     * Creates a new instance if it doesn't exist.
     *
     * @return The single instance of EventManager.
     */
    public EventManager getInstance() {
        if (this.instance == null) {
            this.instance = new EventManager(); // Create a new instance if it hasn't been initialized.
        }
        return this.instance;
    }

    /**
     * Registers an event with the EventManager by associating an EventListener with an EventCommand.
     * Also schedules the event command in the CommandScheduler and stores it in the event map.
     *
     * @param commandEvent The listener that triggers the command.
     * @param command The event command that gets executed.
     * @param requiredSubsystems The subsystems that are required for this event to run.
     */
    public void registerEvent(EventListener commandEvent, EventCommand command, Subsystem... requiredSubsystems) {

        // Schedule the command in the command scheduler with a very high priority (999999999).
        commandScheduler.scheduleCommand(command, 999999999, requiredSubsystems);

        // Store the event listener and its associated command in the event map.
        EVENT_MAP.put(commandEvent, command);
    }

    /**
     * Executes the currently registered event commands.
     * Loops through all event listeners and checks if the corresponding event is active.
     * If active, adds the command to the active commands list and triggers the command to execute.
     *
     * @throws UnscheduledCommandException If a command has not been scheduled but is being executed.
     */
    public void execute() throws UnscheduledCommandException {

        // Iterate through all registered event listeners.
        for (EventListener eventListener : EVENT_MAP.keySet()) {

            // Get the associated event command for the current event listener.
            EventCommand eventCommand = EVENT_MAP.get(eventListener);

            // Check if the event listener's event is active.
            if (eventListener.isEventActive()) {

                // If the event is active and the command isn't already in the active commands list, activate it.
                if (!ACTIVE_EVENT_COMMANDS.contains(eventCommand)) {
                    ACTIVE_EVENT_COMMANDS.add(eventCommand); // Add the command to the set of active commands.
                    commandScheduler.toggleEventCommand(eventCommand.getID()); // Toggle the command in the scheduler.
                }
            } else {

                // If the event is not active and the command is in the active commands list, deactivate it.
                if (ACTIVE_EVENT_COMMANDS.contains(eventCommand)) {
                    ACTIVE_EVENT_COMMANDS.remove(eventCommand); // Remove the command from the set of active commands.
                    commandScheduler.toggleEventCommand(eventCommand.getID()); // Toggle the command in the scheduler (stop it).
                }
            }
        }
    }
}