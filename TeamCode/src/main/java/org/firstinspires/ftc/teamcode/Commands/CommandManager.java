package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Exceptions.UnscheduledCommandException;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public interface CommandManager {

    void scheduleCommand(CommandBase commandBase, int priority, Subsystem... requiredSubsystems);

    void stopAll();

    CommandBase getCommandWithID(String commandID) throws UnscheduledCommandException;
    CommandBase getCommandAtIndex(int index);
    int getCommandIndex(String commandID) throws UnscheduledCommandException;
}
