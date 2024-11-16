package org.firstinspires.ftc.teamcode.Exceptions;

public class NullCommandException extends Exception {
    public NullCommandException(String terminatingCommandID) {
        super("Command with ID " + terminatingCommandID + " gave a directive that returned a null command.");
    }
}
