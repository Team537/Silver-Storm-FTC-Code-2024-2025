package org.firstinspires.ftc.teamcode.Commands.Directives;

import org.firstinspires.ftc.teamcode.Commands.CommandBase;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.UtilityCommands.StopCommand;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public class SkipDirective implements Directive {

    private String commandID;
    private int step;

    /**
     * Constructs a SkipDirective that determines the next command to execute based on the given
     * command ID and step amount.
     *
     * @param commandID The ID of the command that this directive is associated with.
     * @param step The number of steps (positive or negative) to move forward or backward
     *             from the command with the specified ID to determine the next command.
     */
    public SkipDirective(String commandID, int step) {
        this.commandID = commandID;
        this.step = step;
    }

    /**
     * Sets the command ID for this directive.
     *
     * @param commandID The ID of the command that this directive is associated with.
     */
    public void setCommandID(String commandID) {
        this.commandID = commandID;
    }

    /**
     * Executes this directive by determining the next command to schedule based on the current
     * command's index and the specified step.
     *
     * If no valid command is found at the calculated index, a StopCommand is returned to halt
     * further execution.
     *
     * @return The next command to be executed, based on the current command's index and the step.
     * @throws UnscheduledCommandException If the specified command ID is invalid or unscheduled.
     */
    @Override
    public CommandBase execute() throws UnscheduledCommandException {

        // Get the CommandScheduler instance to interact with the scheduled commands.
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        // Retrieve the index of the command currently being executed.
        int activeCommandIndex = commandScheduler.getCommandIndex(commandID);

        // Calculate the next command by moving 'step' indexes forward or backward from the active command.
        CommandBase nextCommand = commandScheduler.getCommandAtIndex(activeCommandIndex + step);

        // If no valid command is found at the calculated index, return a StopCommand to halt further execution.
        if (nextCommand == null) {
            nextCommand = new StopCommand();
        }

        // Return the next command to execute.
        return nextCommand;
    }
}