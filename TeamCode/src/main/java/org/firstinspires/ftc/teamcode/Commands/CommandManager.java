package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;

public interface CommandManager {

    void scheduleCommand(CommandBase commandBase);
    void stopAll();

    CommandBase getCommandWithID(String commandID) throws UnscheduledCommandException;
    CommandBase getCommandAtIndex(int index);
    int getCommandIndex(String commandID) throws UnscheduledCommandException;
}
