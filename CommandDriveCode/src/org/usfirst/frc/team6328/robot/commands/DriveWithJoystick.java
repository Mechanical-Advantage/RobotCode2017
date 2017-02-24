package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives with the joystick from OI
 */
public class DriveWithJoystick extends Command {
	
	private final double deadband = 0.05;

    public DriveWithJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("DriveWithJoystick");
    	requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double joystickLeft = 0, joystickRight = 0;
    	// cube to improve low speed control, multiply by -1 because negative joystick means forward
    	if (Math.abs(Robot.oi.getRightAxis()) > deadband) {
    		joystickRight = Robot.oi.getRightAxis()*Math.abs(Robot.oi.getRightAxis())*RobotMap.maxVelocity*-1;
    	}
    	if (Math.abs(Robot.oi.getLeftAxis()) > deadband) {
    		joystickLeft = Robot.oi.getLeftAxis()*Math.abs(Robot.oi.getLeftAxis())*RobotMap.maxVelocity*-1;
    	}
		Robot.driveSubsystem.drive(joystickRight, joystickLeft);
    	//System.out.println("Left: " + Robot.oi.getLeftAxis() + " Right: " + Robot.oi.getRightAxis());
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
    }
}
