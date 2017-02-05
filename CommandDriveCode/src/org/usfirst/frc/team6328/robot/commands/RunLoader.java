package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Runs the ball loader to move balls to trigger
 */
public class RunLoader extends Command {
	
	private boolean backwards;

    public RunLoader(boolean runBackwards) {
    	super("RunLoader");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.loaderSubsystem);
    	backwards = runBackwards;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (backwards) {
    		Robot.loaderSubsystem.reverse();
    	} else {
    		Robot.loaderSubsystem.run();
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.loaderSubsystem.stop();
    }
}
