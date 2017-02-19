package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Runs the input command for a certain time
 */
public class RunCommandForTime extends TimedCommand {
	
	private Command command;

    public RunCommandForTime(Command commandToRun, double timeout) {
        super(timeout);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        command = commandToRun;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	command.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Called once after timeout
    protected void end() {
    	command.cancel();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	command.cancel();
    }
}
