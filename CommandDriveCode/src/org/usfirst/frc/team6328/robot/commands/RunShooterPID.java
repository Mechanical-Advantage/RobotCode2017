package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Runs the shooter using a PID
 */
public class RunShooterPID extends Command implements PIDOutput {
	
	private final double kP = 0.001;
	private final double kI = 0;
	private final double kD = 0;
	private final double kF = 0;
	private final double updatePeriod = 0.05;
	private final double targetSpeed = 85;
	
	private double shooterPIDSpeed;
	private PIDController shooterSpeedController;

    public RunShooterPID() {
    	super("RunShooterPID");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	shooterSpeedController = new PIDController(kP, kI, kD, kF, Robot.shooterSubsystem, this, updatePeriod);
    	shooterSpeedController.setSetpoint(targetSpeed);
    	shooterSpeedController.setOutputRange(0, 1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	shooterSpeedController.enable();
    	if (RobotMap.tuningMode) {
    		shooterSpeedController.setSetpoint(SmartDashboard.getNumber("Shooter Target RPS", targetSpeed));
    	}
    	if (Robot.oi.getOpenLoopShooter()) {
    		Robot.shooterSubsystem.runOpenLoop();
    	} else {
    		Robot.shooterSubsystem.run(shooterPIDSpeed);
    	}
    	if (RobotMap.tuningMode) {
    		SmartDashboard.putNumber("Shooter Speed", Robot.shooterSubsystem.getSpeed());
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
    	shooterSpeedController.disable();
    	shooterPIDSpeed = 0;
    	Robot.shooterSubsystem.stop();
    }
    
    @Override
    public void pidWrite(double output) {
    	shooterPIDSpeed = output;
    }
}
