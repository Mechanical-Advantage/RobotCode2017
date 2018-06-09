package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.LogitechOI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Keeps the climber running in hold mode
 */
public class ClimberHold extends Command {

    public ClimberHold() {
    	super("ClimberHold");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climberSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.climberSubsystem.hold();
    	Robot.oi.updateLED(OILED.AUTOHOLD, true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
    	Robot.climberSubsystem.stop();
    	Robot.oi.updateLED(OILED.AUTOHOLD, false);
    }
}
