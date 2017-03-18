package org.usfirst.frc.team6328.robot.subsystems;

import java.io.File;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.TrajectoryPoint;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Robot Drive Train
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	/*
	 * Talon SRX Unit Notes:
	 * 
	 * CTRE Mag Encoder Relative: 4096 ticks/native units per rotation
	 * Meets talon requirements for scaling to rotations/rpm without telling talon number of ticks
	 * Conversion factor: 6.8266 native units/100ms = 1rpm (for encoder with 4096 ticks only)
	 * 
	 * Quad encoder needs talon to be told number of ticks (1440 for test bot)
	 * Current code tells talon, so units can be in rpm
	 * For practice robot, 2.4 native units/100ms = 1 rpm
	 * 
	 * Some functions use native units even when scaling available
	 * get() and set() functions use scaling if available
	 * getEncVelocity(), getEncPosition(), getClosedLoopError() use native units
	 * This code uses getEncVelocity(), not get()
	 * 
	 * This code does its own getEncPosition() to rotations scaling using the ticksPerRotation variable
	 * 
	 * Native units are ticks, native units in velocity is ticks per 100ms
	 * 
	 * See Talon SRX Software Reference Manual sections 17.1, 17.2
	 */
	
	private static final double kPPractice = 2;
	private static final double kIPractice = 0;
	private static final double kDPractice = 40;
	private static final double kFPractice = 1.07;
	private static final int kIZonePractice = 0;
	
	private static final double kPCompetition = 0.6;
	private static final double kICompetition = 0.0007;
	private static final double kDCompetition = 6;
	private static final double kFCompetition = 0.2842;
	//private static final int kIZoneCompetition = 0; // disable i zone for now since it is not tested
	private static final int kIZoneCompetition = 4096*50/600; // 4096: encoder ticks per rotation; 25: rpm, set this; 600: converting minute to 100ms
	
	private static final double safetyExpiration = 2;
	private static final double sniperMode = 0.25; // multiplied by velocity in sniper mode
	private static final boolean sniperModeLocked = false; // when set, sniper mode uses value above, when unset, value comes from throttle control on joystick
	private static final int currentLimit = 50;
	private static final boolean enableCurrentLimit = false;
	private static final int requiredTrajPoints = 5; // point to send to talon before starting profile
	
	private CANTalon rightTalonMaster;
	private CANTalon rightTalonSlave;
	private CANTalon rightTalonSlave2;
	private CANTalon leftTalonMaster;
	private CANTalon leftTalonSlave;
	private CANTalon leftTalonSlave2;
	private FeedbackDevice encoderType;
	private int ticksPerRotation; // getEncPosition values in one turn
	private double wheelDiameter; // inches
	private double wheelBaseWidth; // inches, distance between left and right wheels
	private boolean reverseSensorLeft;
	private boolean reverseSensorRight;
	private boolean motionProfilingActive = false; // true when motion profile mode is loaded, even if not running
	private ProcessTalonMotionProfileBuffer processTalonMotionProfile/* = new ProcessTalonMotionProfileBuffer()*/;
	private Notifier processMotionProfileNotifier/* = new Notifier(processTalonMotionProfile)*/;
	private double motionProfileNotifierUpdateTime;
	
	public DriveTrain() {
		rightTalonMaster = new CANTalon(RobotMap.rightMaster);
		rightTalonSlave = new CANTalon(RobotMap.rightSlave);
		leftTalonMaster = new CANTalon(RobotMap.leftMaster);
		leftTalonSlave = new CANTalon(RobotMap.leftSlave);
		if (RobotMap.practiceRobot) {
			encoderType = FeedbackDevice.QuadEncoder;
			ticksPerRotation = 1440;
			wheelDiameter = 6;
			wheelBaseWidth = 27.75;
			reverseSensorRight = true;
			reverseSensorLeft = false;
			rightTalonMaster.configEncoderCodesPerRev(ticksPerRotation/4); // For quad encoders, talon codes are 4 ticks
			leftTalonMaster.configEncoderCodesPerRev(ticksPerRotation/4);
			rightTalonMaster.reverseSensor(reverseSensorRight);
			rightTalonMaster.reverseOutput(true);
			leftTalonMaster.reverseSensor(reverseSensorLeft);
			leftTalonMaster.reverseOutput(false);
		} else {
			rightTalonSlave2 = new CANTalon(RobotMap.rightSlave2);
			leftTalonSlave2 = new CANTalon(RobotMap.leftSlave2);
			encoderType = FeedbackDevice.CtreMagEncoder_Relative;
			ticksPerRotation = 4096;
			wheelDiameter = 4.1791666667;
			reverseSensorRight = false;
			reverseSensorLeft = true;
			wheelBaseWidth = 0; // TODO set this
			rightTalonMaster.reverseSensor(reverseSensorRight);
			rightTalonMaster.reverseOutput(true);
			leftTalonMaster.reverseSensor(reverseSensorLeft);
			leftTalonMaster.reverseOutput(false);
		}
		rightTalonMaster.setFeedbackDevice(encoderType);
		rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		rightTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		rightTalonMaster.EnableCurrentLimit(enableCurrentLimit);
		rightTalonMaster.setCurrentLimit(currentLimit);
		leftTalonMaster.setFeedbackDevice(encoderType);
		leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		leftTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftTalonMaster.EnableCurrentLimit(enableCurrentLimit);
		leftTalonMaster.setCurrentLimit(currentLimit);
		useClosedLoop();
		resetPosition();
		rightTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightTalonSlave.set(RobotMap.rightMaster);
		rightTalonSlave.EnableCurrentLimit(enableCurrentLimit);
		rightTalonSlave.setCurrentLimit(currentLimit);
		leftTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftTalonSlave.set(RobotMap.leftMaster);
		leftTalonSlave.EnableCurrentLimit(enableCurrentLimit);
		leftTalonSlave.setCurrentLimit(currentLimit);
		rightTalonMaster.setExpiration(safetyExpiration);
		rightTalonSlave.setExpiration(safetyExpiration);
		leftTalonMaster.setExpiration(safetyExpiration);
		leftTalonSlave.setExpiration(safetyExpiration);
		if (!RobotMap.practiceRobot){
			rightTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
			rightTalonSlave2.set(RobotMap.rightMaster);
			leftTalonSlave2.changeControlMode(CANTalon.TalonControlMode.Follower);
			leftTalonSlave2.set(RobotMap.leftMaster);
			rightTalonSlave2.setExpiration(safetyExpiration);
			leftTalonSlave2.setExpiration(safetyExpiration);
		}
		enableBrakeMode(true);
		
		/*rightTalonMaster.setSafetyEnabled(false);
		rightTalonSlave.setSafetyEnabled(false);
		leftTalonMaster.setSafetyEnabled(false);
		leftTalonSlave.setSafetyEnabled(false);*/
	}
	
	private void setupVelocityClosedLoop(double p, double i, double d, double f, int iZone) {
		rightTalonMaster.changeControlMode(TalonControlMode.Speed);
		leftTalonMaster.changeControlMode(TalonControlMode.Speed);
		//rightTalonMaster.setProfile(0);
		rightTalonMaster.setF(f);
		rightTalonMaster.setP(p);
		rightTalonMaster.setI(i);
		rightTalonMaster.setD(d);
		rightTalonMaster.setIZone(iZone);
		//leftTalonMaster.setProfile(0);
		leftTalonMaster.setF(f);
		leftTalonMaster.setP(p);
		leftTalonMaster.setI(i);
		leftTalonMaster.setD(d);
		leftTalonMaster.setIZone(iZone);
	}
	
	/**
	 * Sets the current control mode to closed loop
	 * 
	 * Should only ever be called by ApplyOpenLoopSwitch or stopMotionProfile
	 */
	public void useClosedLoop() {
		if (!motionProfilingActive) {
			if (RobotMap.practiceRobot) {
				setupVelocityClosedLoop(kPPractice, kIPractice, kDPractice, kFPractice, kIZonePractice);
			} else {
				setupVelocityClosedLoop(kPCompetition, kICompetition, kDCompetition, kFCompetition, kIZoneCompetition);
			}	
		}
	}
	
	/**
	 * Sets the current control mode to open loop
	 * 
	 * Should only ever be called by ApplyOpenLoopSwitch or stopMotionProfile
	 */
	public void useOpenLoop() {
		if (!motionProfilingActive) {
			rightTalonMaster.changeControlMode(TalonControlMode.PercentVbus);
			leftTalonMaster.changeControlMode(TalonControlMode.PercentVbus);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void drive(int right, int left) {
    	drive((double)right, (double)left);
    }
    
    private double calcActualVelocity(double input) {
    	if (input>=-0.1 && input<=0.1) {
    		return 0;
    	}
    	else if (input>0.1 && input<RobotMap.minVelocity) {
    		return RobotMap.minVelocity;
    	}
    	else if (input<-0.1 && input>RobotMap.minVelocity*-1) {
    		return RobotMap.minVelocity*-1;
    	}
    	else {
    		return input;
    	}
    }
    
    public void drive(double right, double left) {
    	if (Robot.oi.getDriveEnabled() && !motionProfilingActive) {
    		double velocityRight;
    		double velocityLeft;
    		enable();
    		if (Robot.oi.getSniperMode()) {
    			if (sniperModeLocked) {
    				velocityRight = calcActualVelocity(right*sniperMode);
    				velocityLeft = calcActualVelocity(left*sniperMode);
    			} else {
    				velocityRight = calcActualVelocity(right*Robot.oi.getSniperLevel());
    				velocityLeft = calcActualVelocity(left*Robot.oi.getSniperLevel());
    			}
    		} else {
    			velocityRight = calcActualVelocity(right);
    			velocityLeft = calcActualVelocity(left);
    		}
    		if (Robot.oi.getOpenLoop()) {
    			velocityRight/=-RobotMap.maxVelocity; // reverse needed for open loop only TODO tie to reverse output
    			velocityLeft/=RobotMap.maxVelocity;
    		}
    		rightTalonMaster.set(velocityRight);
    		leftTalonMaster.set(velocityLeft);
    		//System.out.println("Distance: Right: " + getDistanceRight() + " Left: " + getDistanceLeft());
    	} else if (!Robot.oi.getDriveEnabled()) {
    		disable();
    	}
    }
    
    public void stop() {
    	if (!motionProfilingActive) {
    		rightTalonMaster.set(0);
    		leftTalonMaster.set(0);
    	}
    }
    
    public void enable() {
    	rightTalonMaster.enable();
    	leftTalonMaster.enable();
    	rightTalonSlave.set(RobotMap.rightMaster); // reset slave master, talons lose their master when disabled
    	leftTalonSlave.set(RobotMap.leftMaster);
    	if (!RobotMap.practiceRobot) {
    		rightTalonSlave2.set(RobotMap.rightMaster);
    		leftTalonSlave2.set(RobotMap.leftMaster);
    	}
    }
    
    public void disable() {
    	rightTalonMaster.disable();
    	leftTalonMaster.disable();
    }
    
    public void enableBrakeMode(boolean enable) {
    	rightTalonMaster.enableBrakeMode(enable);
		leftTalonMaster.enableBrakeMode(enable);
		rightTalonSlave.enableBrakeMode(enable);
		leftTalonSlave.enableBrakeMode(enable);
		if (!RobotMap.practiceRobot) {
			rightTalonSlave2.enableBrakeMode(enable);
			leftTalonSlave2.enableBrakeMode(enable);
		}
    }
    
    public void resetPosition() {
    	rightTalonMaster.setEncPosition(0);
		leftTalonMaster.setEncPosition(0);
    }
    
    // values are cast to doubles to prevent integer division
    public double getRotationsLeft() {
    	double rotLeft = (double)leftTalonMaster.getEncPosition()/(double)ticksPerRotation;
    	if (reverseSensorLeft) {
    		rotLeft*=-1;
    	}
    	return rotLeft;
    }
    
    public double getRotationsRight() {
    	double rotRight = (double)rightTalonMaster.getEncPosition()/(double)ticksPerRotation;
    	if (reverseSensorRight) {
    		rotRight*=-1;
    	}
    	return rotRight;
    }
    
    // diameter times pi equals circumference times rotations equals distance
    public double getDistanceRight() {
    	return wheelDiameter*Math.PI*getRotationsRight();
    }
    
    public double getDistanceLeft() {
    	return wheelDiameter*Math.PI*getRotationsLeft();
    }
    
    public double getVelocityRight() {
    	return rightTalonMaster.getEncVelocity();
    }
    
    public double getVelocityLeft() {
    	return leftTalonMaster.getEncVelocity();
    }
    
    // average current of left and right masters
    public double getCurrent() {
    	return (rightTalonMaster.getOutputCurrent()+leftTalonMaster.getOutputCurrent())/2;
    }
    
    
    /**
     * Loads the passed trajectory, and activates motion profiling mode
     * @param trajectory
     */
    public void loadMotionProfile(Trajectory trajectory) {
    	motionProfilingActive = true;
    	rightTalonMaster.changeControlMode(TalonControlMode.MotionProfile);
    	leftTalonMaster.changeControlMode(TalonControlMode.MotionProfile);
    	rightTalonMaster.set(CANTalon.SetValueMotionProfile.Disable.value);
    	leftTalonMaster.set(CANTalon.SetValueMotionProfile.Disable.value);
    	
    	TankModifier modifier = new TankModifier(trajectory);
    	modifier.modify(wheelBaseWidth);
    	TrajectoryPoint[] leftTrajectory = convertToTalonPoints(modifier.getRightTrajectory(), false); // Pathfinder seems to return swapped left/right, so swap them back
    	TrajectoryPoint[] rightTrajectory = convertToTalonPoints(modifier.getLeftTrajectory(), false);
    	File leftFile = new File("/tmp/traj-left.csv");
    	File rightFile = new File("/tmp/traj-right.csv");
    	Pathfinder.writeToCSV(leftFile, modifier.getRightTrajectory()); // also swap here
    	Pathfinder.writeToCSV(rightFile, modifier.getLeftTrajectory());
    	motionProfileNotifierUpdateTime = trajectory.segments[0].dt/2;
    	//processTalonMotionProfile.reset();
    	processTalonMotionProfile = new ProcessTalonMotionProfileBuffer(); // could be changed back to resetting one Runnable
    	processMotionProfileNotifier = new Notifier(processTalonMotionProfile);
    	rightTalonMaster.changeMotionControlFramePeriod((int) (motionProfileNotifierUpdateTime*1000));
    	leftTalonMaster.changeMotionControlFramePeriod((int) (motionProfileNotifierUpdateTime*1000));
    	rightTalonMaster.clearMotionProfileTrajectories();
    	leftTalonMaster.clearMotionProfileTrajectories();
    	for (int i = 0; i < trajectory.length(); i++) {
    		rightTalonMaster.pushMotionProfileTrajectory(rightTrajectory[i]);
    		leftTalonMaster.pushMotionProfileTrajectory(leftTrajectory[i]);
    	}
    }
    
    public void startMotionProfile() {
    	if (motionProfilingActive && Robot.oi.getDriveEnabled()) {
    		enable();
    		processMotionProfileNotifier.startPeriodic(motionProfileNotifierUpdateTime);
    	} else if (!Robot.oi.getDriveEnabled()) {
    		disable();
    	}
    }

    public void stopMotionProfile() {
    	if (motionProfilingActive) {
    		processMotionProfileNotifier.stop();
    		rightTalonMaster.set(CANTalon.SetValueMotionProfile.Disable.value);
    		leftTalonMaster.set(CANTalon.SetValueMotionProfile.Disable.value);
    		motionProfilingActive = false; // this needs to be changed before calling useClosed/OpenLoop so that they work
    		if (Robot.oi.getOpenLoop()) {
    			useOpenLoop();
    		} else {
    			useClosedLoop();
    		}
    	}
    }
    
    /**
     * Returns true if both sides have reacher their last trajectory point
     * @return
     */
    public boolean isMotionProfileComplete() {
    	CANTalon.MotionProfileStatus leftStatus = new CANTalon.MotionProfileStatus();
    	CANTalon.MotionProfileStatus rightStatus = new CANTalon.MotionProfileStatus();
    	if (motionProfilingActive) {
    		rightTalonMaster.getMotionProfileStatus(rightStatus);
    		leftTalonMaster.getMotionProfileStatus(leftStatus);
    		return leftStatus.activePoint.isLastPoint && rightStatus.activePoint.isLastPoint;
    	}
    	return false; // if motion profiling not active
    }
    
    private TrajectoryPoint[] convertToTalonPoints(Trajectory t, boolean invert) {
    	TrajectoryPoint[] points = new TrajectoryPoint[t.length()];
    	for (int i = 0; i < points.length; i++) {
    		Segment s = t.get(i);
    		TrajectoryPoint point = new TrajectoryPoint();
    		point.position = s.position/(wheelDiameter*Math.PI);
    		point.velocity = (s.velocity/(wheelDiameter*Math.PI)) * 60;
    		point.timeDurMs = (int) (s.dt * 1000.0);
    		point.profileSlotSelect = 0;
    		point.velocityOnly = false;
    		point.zeroPos = i == 0;
    		point.isLastPoint = i == t.length() - 1;

    		if (invert) {
    			point.position = -point.position;
    			point.velocity = -point.velocity;
    		}

    		points[i] = point;
    	}
    	return points;
    }
    
    private class ProcessTalonMotionProfileBuffer implements Runnable {
    	
    	private boolean profileStarted = false;
    	private boolean clearCompleted = false;
    	private CANTalon.MotionProfileStatus leftStatus = new CANTalon.MotionProfileStatus();
    	private CANTalon.MotionProfileStatus rightStatus = new CANTalon.MotionProfileStatus();
    	
    	@SuppressWarnings("unused")
		public void reset() { // right now a new thread is created each time so this is not used
    		System.out.println("Thread reset, previous clearCompleted: " + clearCompleted);
    		profileStarted = false;
    		clearCompleted = false;
    	}
    	
    	private void startProfile() {
    		rightTalonMaster.set(CANTalon.SetValueMotionProfile.Enable.value);
    		leftTalonMaster.set(CANTalon.SetValueMotionProfile.Enable.value);
    	}

    	public void run() {
    		rightTalonMaster.getMotionProfileStatus(rightStatus);
    		leftTalonMaster.getMotionProfileStatus(leftStatus);
    		if (clearCompleted) {
    			rightTalonMaster.processMotionProfileBuffer();
    			leftTalonMaster.processMotionProfileBuffer();
    			System.out.println("R- Bottom Buffer Points: " + rightStatus.btmBufferCnt + " Active Point pos: " + rightStatus.activePoint.position + " vel: " + rightStatus.activePoint.velocity);
    			System.out.println("L- Bottom Buffer Points: " + leftStatus.btmBufferCnt + " Active Point pos: " + leftStatus.activePoint.position + " vel: " + leftStatus.activePoint.velocity);
    			if (!profileStarted && rightStatus.btmBufferCnt>=requiredTrajPoints && leftStatus.btmBufferCnt>=requiredTrajPoints) {
    				startProfile();
    				profileStarted = true;
    			}
    		} else {
    			System.out.println("Clear not completed " + leftStatus.btmBufferCnt);
    			clearCompleted = leftStatus.btmBufferCnt == 0 && rightStatus.btmBufferCnt == 0;
    			if (clearCompleted) {
    				System.out.println("Before points pushed R - active point pos " + rightStatus.activePoint.position + " vel " 
    						+ rightStatus.activePoint.velocity + " valid " + rightStatus.activePointValid + " bottom buffer " + rightStatus.btmBufferCnt);
    				System.out.println("Before points pushed L - active point pos " + leftStatus.activePoint.position + " vel " 
    						+ leftStatus.activePoint.velocity + " valid " + leftStatus.activePointValid + " bottom buffer " + leftStatus.btmBufferCnt);
    			}
    		}
    	}
    }
}

