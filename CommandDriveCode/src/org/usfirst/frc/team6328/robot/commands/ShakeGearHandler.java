package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Vibrate the gear handler to unstick gears
 */
public class ShakeGearHandler extends Command {
	
	private final int cyclesPerSwitch = 25;
	
	private int cycleCounter;
	private boolean toggleTracker;

    public ShakeGearHandler() {
    	super("ShakeGearHandler");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.gearHandlerSubsystem);
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
    			Robot.gearHandlerSubsystem.open();
    			toggleTracker = false;
    		} else {
    			Robot.gearHandlerSubsystem.close();
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
    	Robot.gearHandlerSubsystem.open();
    }
}
