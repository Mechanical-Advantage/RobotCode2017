package org.usfirst.frc.team6328.robot.commands;

import java.util.StringJoiner;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Uses the operator joystick to run the climber
 */
public class ClimbWithJoystick extends Command {

    public ClimbWithJoystick() {
    	super("ClimbWithJoystick");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climberSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.climberSubsystem.run(Robot.oi.getClimbAxis());
    	if (RobotMap.tuningMode) {
    		SmartDashboard.putNumber("Voltage Graph ClimbWithJoystick", Robot.climberSubsystem.getVoltage());
    		SmartDashboard.putNumber("Current Graph ClimbWithJoystick", Robot.climberSubsystem.getCurrent());
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
    }
    
    @SuppressWarnings("unused")
	private String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
}
