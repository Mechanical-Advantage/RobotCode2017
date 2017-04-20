package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Backs up from the boiler to prepare for shooting, reading amount from dashboard
 */
public class BackUpFromBoiler extends Command {
	
	private Command backUpCommand;
	private double lastDistance;
	private int cyclesRunning;
	
	/* When the sub-command start is called, it does not really start until the next cycle.
	 * This means that isRunning returns false for it on the first cycle of the outer command.
	 * This would cause the outer isFinished to return true, in which case the inner command
	 * runs while the outer does not. This means that interrupted would not get called, because
	 * the command is treated as done.
	 * 
	 * This code forces isFinished to return false on the first cycle to avoid this. */

    public BackUpFromBoiler() {
    	super("BackUpFromBoiler");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double currentDistance = SmartDashboard.getNumber("Boiler Back-Up Distance", 36);
    	if (lastDistance != currentDistance) {
    		backUpCommand = new TalonDriveDistance(currentDistance*-1);
    	}
    	backUpCommand.start();
    	cyclesRunning = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	cyclesRunning+=1;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !backUpCommand.isRunning() && cyclesRunning>1; // cyclesRunning gets incremented once before this is called on the first cycle
//        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	backUpCommand.cancel();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
