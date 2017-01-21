package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnToAngle extends Command implements PIDOutput {
	
	static final double kP = 0.1;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    static final double kToleranceDegrees = 2.0f;
    private PIDController turnController;
    private double targetAngle;
    
    private double rotateToAngleRate;

    public TurnToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("TurnToAngle");
    	requires(Robot.driveSubsystem);
    	turnController = new PIDController(kP, kI, kD, kF, Robot.ahrs, this);
    	turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
        
        // limit input to -180 to 180
        targetAngle = (angle>180) ? 180 : angle;
        targetAngle = (targetAngle<-180) ? -180 : targetAngle;
        
        turnController.setSetpoint(targetAngle);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ahrs.reset();
    	turnController.enable();
    	 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double outputVelocity = rotateToAngleRate*RobotMap.maxVelocity;
    	SmartDashboard.putNumber("Angle", Robot.ahrs.getAngle());
    	SmartDashboard.putNumber("Rate", Robot.ahrs.getRate());
    	SmartDashboard.putNumber("Yaw", Robot.ahrs.getYaw());
    	
    	Robot.driveSubsystem.drive(outputVelocity*-1, outputVelocity);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return turnController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	turnController.disable();
    	Robot.driveSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    @Override
    public void pidWrite(double output) {
    	rotateToAngleRate = output;
    }
}
