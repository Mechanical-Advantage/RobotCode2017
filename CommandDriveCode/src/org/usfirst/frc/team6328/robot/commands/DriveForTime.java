package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Drives for the specified time, speed as -1 to 1
 */
public class DriveForTime extends TimedCommand {
	
	private double runSpeed;

    public DriveForTime(double speed, double timeout) {
        super(timeout);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        runSpeed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveSubsystem.drive(runSpeed*RobotMap.maxVelocity, runSpeed*RobotMap.maxVelocity);
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
    	Robot.driveSubsystem.stop();
    }
}
