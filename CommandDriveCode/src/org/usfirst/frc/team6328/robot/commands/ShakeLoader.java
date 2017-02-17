package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Shakes the loader up and down
 */
public class ShakeLoader extends Command {
	
	private final int cyclesPerSwitch = 5;
	
	private int cycleCounter;
	private boolean toggleTracker;

    public ShakeLoader() {
    	super("ShakeLoader");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.loaderSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	cycleCounter = cyclesPerSwitch; // cause it to run immediately
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	cycleCounter+=1;
    	if (cycleCounter>=cyclesPerSwitch) {
    		if (toggleTracker) {
    			Robot.loaderSubsystem.reverse();;
    			toggleTracker = false;
    		} else {
    			Robot.loaderSubsystem.run();
    			toggleTracker = true;
    		}
    		cycleCounter = 0;
    	}
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
