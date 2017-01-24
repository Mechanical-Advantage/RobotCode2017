package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistance extends Command implements PIDOutput {

	static final double kP = 0.1;
    static final double kI = 0.0;
    static final double kD = 0.0;
    static final double kF = 0.0;
    static final double kToleranceInches = 1.0f;
    static final int kToleranceBufSamples = 10;
    private PIDController distanceController;
	private double targetDistance;
	private DistancePIDSource pidSource = new DistancePIDSource();
	
	private double driveDistanceRate;
	
    public DriveDistance(double distance) { // in inches
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("DriveDistance");
    	requires(Robot.driveSubsystem);
    	targetDistance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	distanceController = new PIDController(kP, kI, kD, kF, pidSource, this);
        distanceController.setOutputRange(-1.0f, 1.0f);
        distanceController.setAbsoluteTolerance(kToleranceInches);
        distanceController.setToleranceBuffer(kToleranceBufSamples);
        distanceController.setContinuous(true);
        distanceController.setSetpoint(targetDistance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double outputVelocity = driveDistanceRate*RobotMap.maxVelocity;
    	Robot.driveSubsystem.drive(outputVelocity, outputVelocity);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return distanceController.onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.disable();
    	distanceController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    private class DistancePIDSource implements PIDSource {
    	@Override
    	public double pidGet() {
    		return (Robot.driveSubsystem.getDistanceRight()+Robot.driveSubsystem.getDistanceLeft())/2;
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
    
    @Override
    public void pidWrite(double output) {
    	driveDistanceRate = output;
    }
}
