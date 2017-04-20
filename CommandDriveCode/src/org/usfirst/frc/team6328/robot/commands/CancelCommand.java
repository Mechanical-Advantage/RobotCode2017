package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * cancels the passed command
 */
public class CancelCommand extends InstantCommand {
	
	private Command commandToCancel;

    public CancelCommand(Command command) {
        super("CancelCommand");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        commandToCancel = command;
    }

    // Called once when the command executes
    protected void initialize() {
    	commandToCancel.cancel();
    }
}
