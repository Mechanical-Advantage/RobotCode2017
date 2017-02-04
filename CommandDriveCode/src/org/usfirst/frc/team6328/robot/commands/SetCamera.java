package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Sets camera to front or rear
 */
public class SetCamera extends Command {

	private boolean frontCamera;
	
    public SetCamera(boolean useFrontCamera) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("SetCamera");
    	frontCamera = useFrontCamera;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (frontCamera) {
    		Robot.mjpegServer.setSource(Robot.frontCamera);
    	}
    	else {
    		Robot.mjpegServer.setSource(Robot.rearCamera);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
