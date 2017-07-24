package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives a specific distance using talon position close loop
 */
public class TalonDriveDistance extends Command {
	
	final double kToleranceInches = 0.5;
	
	double targetDistance;
	boolean motionStarted;
	boolean driveStarted;

    public TalonDriveDistance(double distance) {
    	super("TalonDriveDistance");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    	targetDistance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	motionStarted = false;
    	driveStarted = false;
    	execute(); // read values from networktables first
    	System.out.println("TalonDriveDistance Start");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (RobotMap.tuningMode) {
    		if (SmartDashboard.containsKey("Distance PID/P")) {
    			Robot.driveSubsystem.setPID(SmartDashboard.getNumber("Distance PID/P", 0),
    					SmartDashboard.getNumber("Distance PID/I", 0),
    					SmartDashboard.getNumber("Distance PID/D", 0),
    					SmartDashboard.getNumber("Distance PID/F", 0), 0); // since I is not used for distance, i zone not needed
    		}
    		SmartDashboard.putNumber("Left Distance", Robot.driveSubsystem.getDistanceLeft());
    		SmartDashboard.putNumber("Right Distance", Robot.driveSubsystem.getDistanceRight());
    	}
    	if (!driveStarted && Math.abs(Robot.driveSubsystem.getVelocityLeft()) <= 0.1 && Math.abs(Robot.driveSubsystem.getVelocityRight()) <= 0.1) {
    		Robot.driveSubsystem.driveDistance(targetDistance, true);
    		driveStarted = true;
    	}
    	if (driveStarted && Robot.driveSubsystem.getVelocityLeft() != 0 && Robot.driveSubsystem.getVelocityRight() != 0) {
    		motionStarted = true;
    	}
    	System.out.println("driveStarted: " + driveStarted + " motionStarted: " + motionStarted);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//        return (Math.abs(Robot.driveSubsystem.getDistanceLeft() - targetDistance) < kToleranceInches) && 
//				(Math.abs(Robot.driveSubsystem.getDistanceRight() - targetDistance) < kToleranceInches);
    	return motionStarted && Robot.driveSubsystem.getVelocityLeft() == 0 &&
    			Robot.driveSubsystem.getVelocityRight() == 0;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.stopDistanceDrive();
    	System.out.println("TalonDriveDistance Stop");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
