package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Runs the shooter wheel using bang-bang control in the execute function
 */
public class RunShooterSimpleBangBang extends Command {
		
	private final int targetSpeed = 85; // 83 is 5000 rpm

    public RunShooterSimpleBangBang() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("RunShooterSimpleBangBang");
    	requires(Robot.shooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
	protected void execute() {
    	double speed = Robot.shooterSubsystem.getSpeed();
    	// if in tuning mode, get target from dashboard if it exists
    	int finalTargetSpeed = RobotMap.tuningMode ? (int)SmartDashboard.getNumber("Shooter Target RPS", targetSpeed) : targetSpeed;
    	if (!Robot.oi.getOpenLoopShooter()) {
    		if (speed>finalTargetSpeed) {
    			Robot.shooterSubsystem.stop();
    		} else {
    			Robot.shooterSubsystem.run();
    		}
    		if (RobotMap.tuningMode) {
    			SmartDashboard.putNumber("Shooter Speed", speed);
    		}
    	} else if (Robot.oi.getOpenLoopShooter()) {
    		Robot.shooterSubsystem.runOpenLoop();
    		if (RobotMap.tuningMode) {
    			SmartDashboard.putNumber("Shooter Speed", speed);
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooterSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.shooterSubsystem.stop();
    }
}
