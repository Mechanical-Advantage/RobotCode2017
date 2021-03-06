package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Turns the specified number of degrees, -180 to 180. Resets the gyro at the beginning
 */
public class TurnToAngle extends Command implements PIDOutput {
	
	private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double kToleranceDegrees;
    private int kToleranceBufSamples;
    private double updatePeriod;
    private PIDController turnController;
    private double targetAngle;
    private boolean resetCompleted;
    private double startingUpdateCount;
    
    private double rotateToAngleRate;

    public TurnToAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("TurnToAngle");
    	requires(Robot.driveSubsystem);

        // limit input to -180 to 180
        targetAngle = (angle>180) ? 180 : angle;
        targetAngle = (targetAngle<-180) ? -180 : targetAngle;
        if (RobotMap.practiceRobot) {
        	kP = 0.01;
        	kI = 0;
        	kD = 0.003;
        	kF = 0;
        	kToleranceDegrees = 1.0;
        	kToleranceBufSamples = 10;
        	updatePeriod = 0.05;
        } else {
        	kP = 0.0077; // 0.008
        	kI = 0;
        	kD = 0.0137; // 0.014
        	kF = 0;
        	kToleranceDegrees = 1.0;
        	kToleranceBufSamples = 10;
        	updatePeriod = 0.02;
        }
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	turnController = new PIDController(kP, kI, kD, kF, Robot.ahrs, this, updatePeriod);
    	turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setToleranceBuffer(kToleranceBufSamples);
        turnController.setContinuous(true);
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
        
        turnController.setSetpoint(targetAngle);
    	Robot.ahrs.zeroYaw();
    	startingUpdateCount = Robot.ahrs.getUpdateCount();
    	resetCompleted = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*if (Robot.ahrs.getRate() < 0.3 && !resetStarted) {
    		Robot.ahrs.zeroYaw();
    		resetStarted = true;
    	}*/
    	if (Robot.ahrs.getUpdateCount() >= startingUpdateCount+2) {
    		resetCompleted = true;
    		turnController.enable();
    	}
    	if (resetCompleted) {
	    	double outputVelocity = rotateToAngleRate*RobotMap.maxVelocity;
	    	if (RobotMap.tuningMode) {
	    		SmartDashboard.putNumber("Angle", Robot.ahrs.getAngle());
		    	SmartDashboard.putNumber("Rate", Robot.ahrs.getRate());
		    	SmartDashboard.putNumber("Yaw", Robot.ahrs.getYaw());
		    	SmartDashboard.putNumber("Turn to angle rate", rotateToAngleRate);
		    	SmartDashboard.putNumber("Velocity", outputVelocity);
	    	}
	    	Robot.driveSubsystem.drive(outputVelocity*-1, outputVelocity);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return resetCompleted && (Math.abs(Robot.ahrs.getYaw() - targetAngle) < kToleranceDegrees) && turnController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.disable();
    	turnController.disable();
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
