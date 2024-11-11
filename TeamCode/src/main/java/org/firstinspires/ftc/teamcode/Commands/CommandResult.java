package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Directives.Directive;

public class CommandResult {

    private String commandID;
    private Directive directive;

    // Private constructor to enforce controlled instantiation.
    private CommandResult(String commandID, Directive directive) {
        this.commandID = commandID;
        this.directive = directive;
    }

    /**
     * Returns a new CommandResult storing the given command ID.
     *
     * @param commandID The ID of the command that will be run next.
     *                  This command MUST be scheduled in the command scheduler.
     * @return A new CommandResult storing the given command ID.
     */
    public static CommandResult fromCommandID(String commandID) {
        return new CommandResult(commandID, null);
    }

    /**
     * Returns a new CommandResult storing the given directive.
     *
     * @param directive The directive that will be used to determine which command to run next.
     * @return A new CommandResult storing the given directive.
     */
    public static CommandResult fromDirective(Directive directive) {
        return new CommandResult(null, directive);
    }

    /**
     * Returns whether or not this CommandResult is a commandID.
     *
     * @return whether or not this CommandResul is a commandID.
     */
    public boolean isCommand() {
        return this.commandID != null;
    }

    /**
     * Returns whether or not this CommandResult is a directive.
     *
     * @return whether or not this CommandResul is a directive.
     */
    public boolean isDirective() {
        return this.directive != null;
    }

    /**
     * Return this CommandResult's commandID.
     *
     * @return This CommandResult's commandID.
     */
    public String getCommandID() {
        return this.commandID;
    }

    /**
     * Return this CommandResult's directive.
     *
     * @return This CommandResult's directive.
     */
    public Directive getDirective() {
        return this.directive;
    }
}
