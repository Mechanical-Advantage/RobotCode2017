package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Turns for the specified time at the specified speed, positive speed for right
 */
public class SpinCenterOffsetForTime extends TimedCommand {
	
	private double runSpeed;
	private double turnOffset; // added to both sides

    public SpinCenterOffsetForTime(double speed, double offset, double timeout) {
        super(timeout);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        runSpeed = speed;
        turnOffset = offset;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveSubsystem.drive(RobotMap.maxVelocity*((runSpeed*-1)+turnOffset), RobotMap.maxVelocity*(runSpeed+turnOffset)); // make positive number
    		// drive right backward so the robot turns right
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Called once after timeout
    protected void end() {
    	Robot.driveSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
