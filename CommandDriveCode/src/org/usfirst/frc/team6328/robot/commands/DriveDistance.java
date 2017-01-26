package org.usfirst.frc.team6328.robot.commands;

import java.util.StringJoiner;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveDistance extends Command {

	static final double kP = 0.03;
    static final double kI = 0.0;
    static final double kD = 0.0;
    static final double kF = 0.0;
    static final double kToleranceInches = 1.0f;
    static final int kToleranceBufSamples = 10;
    private PIDController distanceControllerLeft;
    private PIDController distanceControllerRight;
	private double targetDistance;
	private DistancePIDSource pidSourceLeft = new DistancePIDSource(false);
	private DistancePIDSource pidSourceRight = new DistancePIDSource(true);
	private DistancePIDOutput pidOutputLeft = new DistancePIDOutput();
	private DistancePIDOutput pidOutputRight = new DistancePIDOutput();
	private boolean resetCompleted;
	
    public DriveDistance(double distance) { // in inches
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("DriveDistance");
    	requires(Robot.driveSubsystem);
    	targetDistance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	distanceControllerLeft = new PIDController(kP, kI, kD, kF, pidSourceLeft, pidOutputLeft);
    	distanceControllerRight = new PIDController(kP, kI, kD, kF, pidSourceRight, pidOutputRight);
        setupDistanceController(distanceControllerLeft);
        setupDistanceController(distanceControllerRight);
        Robot.driveSubsystem.resetPosition();
        resetCompleted = false;
        System.out.println("Starting distance right " + Robot.driveSubsystem.getDistanceRight());
        System.out.println("Starting Distance Left " + Robot.driveSubsystem.getDistanceLeft());
    }

	private void setupDistanceController(PIDController distanceController) {
		distanceController.setOutputRange(-1.0f, 1.0f);
        distanceController.setAbsoluteTolerance(kToleranceInches);
        distanceController.setToleranceBuffer(kToleranceBufSamples);
        distanceController.setContinuous(true);
        distanceController.setSetpoint(targetDistance);
	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.driveSubsystem.getDistanceLeft()<0.1 && Robot.driveSubsystem.getDistanceRight()<0.1) {
    		resetCompleted = true;
    		distanceControllerLeft.enable();
            distanceControllerRight.enable();
    	}
    	if (resetCompleted) {
	    	double outputVelocityLeft = pidOutputLeft.getDriveDistanceRate()*RobotMap.maxVelocity;
	    	double outputVelocityRight = pidOutputRight.getDriveDistanceRate()*RobotMap.maxVelocity;
	    	Robot.driveSubsystem.drive(outputVelocityRight, outputVelocityLeft);
	    	SmartDashboard.putNumber("Right Velocity", outputVelocityRight);
	    	SmartDashboard.putNumber("Left Velocity", outputVelocityLeft);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("onTarget Left " + distanceControllerLeft.onTarget() + " " + Robot.driveSubsystem.getDistanceLeft());
    	SmartDashboard.putNumber("Left Distance", Robot.driveSubsystem.getDistanceLeft());
    	SmartDashboard.putNumber("Right Distance", Robot.driveSubsystem.getDistanceRight());
    	SmartDashboard.putString("DriveDistance Graph", genGraphStr(targetDistance, Robot.driveSubsystem.getDistanceLeft(), 
    			Robot.driveSubsystem.getDistanceRight()));
    	System.out.println("onTarget Right " + distanceControllerRight.onTarget() + " " + Robot.driveSubsystem.getDistanceRight());
        return (distanceControllerLeft.onTarget() && distanceControllerRight.onTarget());
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.disable();
    	distanceControllerLeft.disable();
    	distanceControllerRight.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    private class DistancePIDSource implements PIDSource {
    	private boolean getRightSide;
    	public DistancePIDSource(boolean rightSide) {
    		getRightSide = rightSide;
    	}
    	@Override
    	public double pidGet() {
    		if (getRightSide) {
    			return (Robot.driveSubsystem.getDistanceRight());
    		}
    		else {
    			return (Robot.driveSubsystem.getDistanceLeft());
    		}
    	}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			// TODO Auto-generated method stub
			return PIDSourceType.kDisplacement;
		}
    }
    
    private class DistancePIDOutput implements PIDOutput {
    	private double driveDistanceRate;
    	public double getDriveDistanceRate() {
			return driveDistanceRate;
		}
		@Override
    	public void pidWrite(double output) {
    		driveDistanceRate = output;
    	}
    }
    
    private String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
}
