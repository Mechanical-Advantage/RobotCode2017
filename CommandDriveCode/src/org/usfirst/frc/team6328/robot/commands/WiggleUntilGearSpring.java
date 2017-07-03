package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Wiggles back and forth until the gear in sensed
 * Wiggles to the right first
 */
public class WiggleUntilGearSpring extends Command {

	private final int cyclesPerSwitch = 20;
	private final double sideSpeed = 0.1;

	private int cycleCounter;
	private boolean toggleTracker;

    public WiggleUntilGearSpring() {
    	super("WiggleUntilGearSpring");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	cycleCounter = cyclesPerSwitch; // cause it to run immediately, do not use with half wiggle
    	toggleTracker = false;
    	// do a half wiggle first so we are centered
    	Robot.driveSubsystem.drive(sideSpeed*RobotMap.maxVelocity, 0);
    	cycleCounter = cyclesPerSwitch/2; // this creates a half-cycle
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	cycleCounter+=1;
    	if (cycleCounter>=cyclesPerSwitch) {
    		if (toggleTracker) {
    			Robot.driveSubsystem.drive(sideSpeed*RobotMap.maxVelocity, 0);
    			toggleTracker = false;
    		} else {
    			Robot.driveSubsystem.drive(0, sideSpeed*RobotMap.maxVelocity);
    			toggleTracker = true;
    		}
    		cycleCounter = 0;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.expelGearSubsystem.getSpringSensor();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.driveSubsystem.disable();
    }
}
