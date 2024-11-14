package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Exceptions.NullCommandException;
import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Constants;

abstract public class CommandBase {

    private String parentID; // The ID of the command group this command is located in. If the command isn't in a command group, there won't be any parent ID.
    private String commandID;

    // Flags
    private boolean calculatedID = false;

    // Abstract Methods
    public abstract void init(); // Runs whenever the command is activated.
    public abstract void execute() throws NullCommandException, UnscheduledCommandException; // Runs each time the command scheduler "execute" method is called if active.
    public abstract boolean isFinished();
    public abstract void end(boolean interrupted); // Runs when the command ends.

    // Non-abstract methods
    public void setParentID(CommandBase commandParent) {

        // Store the parent command's ID. This will be used alongside this command's
        this.parentID = commandParent.getID();

        // This commands ID needs to be recalculated due to the change in parent.
        this.calculatedID = false;
    }

    /**
     * Determines what will happen after this command finishes running. This is determined by which
     * directive, or commandID, is returned as a CommandResult object. By default this method returns
     * the "NEXT_COMMAND" directive, which moves on to the next scheduled command.
     *
     * @return A CommandResult object determining what will happen when this finished running.
     */
    public CommandResult getCommandResult() {
        return CommandResult.fromDirective(null);
    }

    /**
     * Returns this command's ID, as a string.
     *
     * @return This command's ID.
     */
    public String getID() {

        // If this command's ID has not been calculated, then calculate it.
        if (!calculatedID) {
            calculateCommandID();
            calculatedID = true;
        }

        // Return this command's ID.
        return this.commandID;
    }

    /**
     * Calculates this command's ID.
     */
    private void calculateCommandID() {

        // Set this command's ID equal to the string conversion of this
        this.commandID = this.toString();

        // If this command has a parent, then add its ID in from of this command' ID.
        if (parentID != null) {
            this.commandID = parentID + Constants.CommandConstants.COMMAND_SEPARATION_CHARACTER +  this.commandID;
        }
    }
}
