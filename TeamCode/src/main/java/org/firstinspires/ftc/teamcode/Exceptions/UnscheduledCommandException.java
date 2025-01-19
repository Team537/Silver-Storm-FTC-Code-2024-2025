package org.firstinspires.ftc.teamcode.Exceptions;

public class UnscheduledCommandException extends Exception {
    public UnscheduledCommandException(String commandId) {
        super("Command with ID " + commandId + " was not scheduled and cannot be executed.");
    }
}
