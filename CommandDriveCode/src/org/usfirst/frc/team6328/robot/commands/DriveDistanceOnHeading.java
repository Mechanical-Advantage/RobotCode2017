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
 * Drive the specified distance on the specified heading
 * Does not turn first, really use it to drive straight
 * Also is not good if the robot is strongly knocked off course (like being kicked)
 */
public class DriveDistanceOnHeading extends Command {
	
	static final double kPDistance = 0.017;
    static final double kIDistance = 0.0;
    static final double kDDistance = 0.0;
    static final double kFDistance = 0.5;
    static final double kToleranceInches = 0.5f;
    static final int kToleranceBufSamplesDistance = 10;
    static final double kUpdatePeriodDistance = 0.02;
    
    static final double kPAngle = 0.07;
    static final double kIAngle = 0.000;
    static final double kDAngle = 0.0;
    static final double kFAngle = 0.0;
    static final double kToleranceDegrees = 0.5f;
    static final int kToleranceBufSamplesAngle = 10;
    static final double kTurnCorrectionAmount = 0.2;
    
    // PID output will be limited to negative to positive this. Multiplied by RobotMap maxVelocity to get target
    static final double kMaxOutput = 0.9;
    // Limit change in one iteration to this - % of max output
    static final double kMaxChange = 0.03; 

    static final double maxOutputVelocityChange = RobotMap.maxVelocity * kMaxOutput * kMaxChange;
    private PIDController distanceController;
    private PIDController turnController;
	private double targetDistance;
	private double targetAngle;
	private boolean useStartingYaw;
	private DistancePIDSource pidSourceDistance = new DistancePIDSource();
	private PIDOutputter pidOutputDistance = new PIDOutputter();
	private PIDOutputter pidOutputAngle = new PIDOutputter();
	private boolean resetCompletedDistance;
	private double lastOutputDistance;
    
    public DriveDistanceOnHeading(double distance) {
    	this(distance, 0);
    	useStartingYaw = true;
    }
    
    public DriveDistanceOnHeading(double distance, double heading) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("DriveDistanceOnHeading");
    	requires(Robot.driveSubsystem);
    	targetDistance = distance;
    	targetAngle = (heading>180) ? 180 : heading;
        targetAngle = (targetAngle<-180) ? -180 : targetAngle;
        useStartingYaw = false;
    }

	// Called just before this Command runs the first time
    protected void initialize() {
    	if (useStartingYaw) {
    		targetAngle = Robot.ahrs.getYaw();
    	}
    	
    	distanceController = new PIDController(kPDistance, kIDistance, kDDistance, kFDistance, pidSourceDistance, pidOutputDistance, kUpdatePeriodDistance);
    	turnController = new PIDController(kPAngle, kIAngle, kDAngle, kFAngle, Robot.ahrs, pidOutputAngle);
    	distanceController.setOutputRange(-1, 1);
    	turnController.setOutputRange(-1, 1);
    	distanceController.setAbsoluteTolerance(kToleranceInches);
        distanceController.setToleranceBuffer(kToleranceBufSamplesDistance);
        distanceController.setContinuous(true);
        distanceController.setSetpoint(targetDistance);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setToleranceBuffer(kToleranceBufSamplesAngle);
        turnController.setContinuous(true);
        turnController.setSetpoint(targetAngle);
        lastOutputDistance = 0;
        resetCompletedDistance = false;
        Robot.driveSubsystem.resetPosition();
    }
    
    private double calcNewVelocity(PIDOutputter pidOut, double lastOutput) {
    	double targetOutput = pidOut.getPIDRate()*RobotMap.maxVelocity*(kMaxOutput-kTurnCorrectionAmount);
    	if (Math.abs(lastOutput - targetOutput) > maxOutputVelocityChange){
    		if (lastOutput < targetOutput) {
        		targetOutput = lastOutput + maxOutputVelocityChange;
    		} else {
    			targetOutput = lastOutput - maxOutputVelocityChange;
    		}
    	}
    	return targetOutput;
	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.driveSubsystem.getVelocityLeft()<1 && Robot.driveSubsystem.getVelocityRight()<1) {
    		resetCompletedDistance = true;
    		distanceController.enable();
            turnController.enable();
    	}
    	if (resetCompletedDistance) {
    		double outputVelocity = calcNewVelocity(pidOutputDistance, lastOutputDistance);
    		lastOutputDistance = outputVelocity;
    		double outputTurnVelocity = pidOutputAngle.getPIDRate()*RobotMap.maxVelocity*kTurnCorrectionAmount;
    		// if driving backwards, invert the turn so it still turns the right way (sides need to be adjusted the opposite direction
    		if (targetDistance<0) {
    			outputTurnVelocity*=-1;
    		}
    		// subtract from right side, add to left side (drive left on positive)
    		Robot.driveSubsystem.drive(outputVelocity-outputTurnVelocity, outputVelocity+outputTurnVelocity);
    		SmartDashboard.putString("Velocity Graph", genGraphStr(outputVelocity, outputVelocity+outputTurnVelocity, outputVelocity-outputTurnVelocity));
    		SmartDashboard.putNumber("Turn controller output", pidOutputAngle.getPIDRate());
    		SmartDashboard.putString("Yaw Graph", genGraphStr(targetAngle, Robot.ahrs.getYaw()));
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return resetCompletedDistance && distanceController.onTarget() && turnController.onTarget() && 
        		(Math.abs(getAverageDistance() - targetDistance) < kToleranceInches) &&
        		(Math.abs(Robot.ahrs.getYaw() - targetAngle) < kToleranceDegrees);
    }

    // Called once after isFinished returns true
    protected void end() {
    	turnController.disable();
    	distanceController.disable();
    	Robot.driveSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    private double getAverageDistance() {
    	return (Robot.driveSubsystem.getDistanceLeft()+Robot.driveSubsystem.getDistanceRight())/2;
    }
    
    private class DistancePIDSource implements PIDSource {
    	@Override
    	public double pidGet() {
    		return getAverageDistance();
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
    
    private class PIDOutputter implements PIDOutput {
    	private double PIDRate;
    	public double getPIDRate() {
			return PIDRate;
		}
		@Override
    	public void pidWrite(double output) {
    		PIDRate = output;
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
