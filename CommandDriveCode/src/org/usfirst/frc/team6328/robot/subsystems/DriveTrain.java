package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick;

import org.usfirst.frc.team6328.robot.CANTalon.CANTalon;
import org.usfirst.frc.team6328.robot.CANTalon.CANTalon.FeedbackDevice;
import org.usfirst.frc.team6328.robot.CANTalon.CANTalon.StatusFrameRate;
import org.usfirst.frc.team6328.robot.CANTalon.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Trajectory;

/**
 * Robot Drive Train
 * MP Support has been removed because it would be complicated to make work on phoenix
 * (It would probably make more sense to port this code directly to phoenix than to fix MP support in the compatibility layer anyway)
 * (Talon MP support is obsolete anyway, updating the on rio support is better)
 * (The 2017 code doesn't really need MP support anymore)
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
	private static final double kPPracticeMP = 2; // the MP settings are also used for distance close loop
	private static final double kIPracticeMP = 0;
	private static final double kDPracticeMP = 40;
	private static final double kFPracticeMP = 1.0768;
	private static final int kIZonePracticeMP = 0;
	private static final int kAllowableErrorPracticeDistance = 8; // ticks sent to talon as allowable error for distance close loop
	
	private static final double kPCompetition = 0.6;
	private static final double kICompetition = 0.0007;
	private static final double kDCompetition = 6;
	private static final double kFCompetition = 0.2842;
	private static final int kIZoneCompetition = 4096*50/600; // 4096: encoder ticks per rotation; 25: rpm, set this; 600: converting minute to 100ms
	private static final double kPCompetitionMP = 2;
//	private static final double kPCompetitionMP = 0.6;
	private static final double kICompetitionMP = 0.0007;
	private static final double kDCompetitionMP = 45;
//	private static final double kDCompetitionMP = 6;
	private static final double kFCompetitionMP = 0.2842;
	private static final int kIZoneCompetitionMP = 4096*50/600;
	private static final int kAllowableErrorCompetitionDistance = 24; // ticks sent to talon as allowable error for distance close loop
	
	private static final double safetyExpiration = 2;
	private static final double sniperMode = 0.25; // multiplied by velocity in sniper mode
	private static final boolean sniperModeLocked = false; // when set, sniper mode uses value above, when unset, value comes from throttle control on joystick
	private static final int currentLimit = 50;
	private static final boolean enableCurrentLimit = false;
	private CANTalon rightTalonMaster;
	private CANTalon rightTalonSlave;
	private CANTalon rightTalonSlave2;
	private CANTalon leftTalonMaster;
	private CANTalon leftTalonSlave;
	private CANTalon leftTalonSlave2;
	private FeedbackDevice encoderType;
	private int ticksPerRotation; // getEncPosition values in one turn
	private double wheelDiameter; // inches
	private boolean reverseSensorLeft;
	private boolean reverseSensorRight;
	private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE; // enum defined at end of file
	
	public DriveTrain() {
		rightTalonMaster = new CANTalon(RobotMap.rightMaster);
		rightTalonSlave = new CANTalon(RobotMap.rightSlave);
		leftTalonMaster = new CANTalon(RobotMap.leftMaster);
		leftTalonSlave = new CANTalon(RobotMap.leftSlave);
		if (RobotMap.practiceRobot) {
			encoderType = FeedbackDevice.QuadEncoder;
			ticksPerRotation = 1440;
			wheelDiameter = 5.9000000002; // 6
			reverseSensorRight = true;
			reverseSensorLeft = false;
			rightTalonMaster.configEncoderCodesPerRev(ticksPerRotation/4); // For quad encoders, talon codes are 4 ticks
			leftTalonMaster.configEncoderCodesPerRev(ticksPerRotation/4);
			rightTalonMaster.reverseSensor(reverseSensorRight);
			rightTalonMaster.reverseOutput(true);
			leftTalonMaster.reverseSensor(reverseSensorLeft);
			leftTalonMaster.reverseOutput(false);
			setupMotionProfilePID(kPPracticeMP, kIPracticeMP, kDPracticeMP, kFPracticeMP, kIZonePracticeMP);
		} else {
			rightTalonSlave2 = new CANTalon(RobotMap.rightSlave2);
			leftTalonSlave2 = new CANTalon(RobotMap.leftSlave2);
			encoderType = FeedbackDevice.CtreMagEncoder_Relative;
			ticksPerRotation = 4096;
			wheelDiameter = 4.1791666667; // before worlds
//			wheelDiameter = 4.24881941; // 7:48 AM worlds
			reverseSensorRight = false;
			reverseSensorLeft = true;
			rightTalonMaster.reverseSensor(reverseSensorRight);
			rightTalonMaster.reverseOutput(true);
			leftTalonMaster.reverseSensor(reverseSensorLeft);
			leftTalonMaster.reverseOutput(false);
			setupMotionProfilePID(kPCompetitionMP, kICompetitionMP, kDCompetitionMP, kFCompetitionMP, kIZoneCompetitionMP);
		}
		rightTalonMaster.setFeedbackDevice(encoderType);
//		rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f); // currently set in useClosedLoop so that motion profiling can change this
		rightTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		rightTalonMaster.EnableCurrentLimit(enableCurrentLimit);
		rightTalonMaster.setCurrentLimit(currentLimit);
		rightTalonMaster.setMotionMagicCruiseVelocity(RobotMap.maxVelocity);
		rightTalonMaster.setMotionMagicAcceleration(RobotMap.maxAcceleration);
		leftTalonMaster.setFeedbackDevice(encoderType);
//		leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		leftTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftTalonMaster.EnableCurrentLimit(enableCurrentLimit);
		leftTalonMaster.setCurrentLimit(currentLimit);
		leftTalonMaster.setMotionMagicCruiseVelocity(RobotMap.maxVelocity);
		leftTalonMaster.setMotionMagicAcceleration(RobotMap.maxAcceleration);
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
	
	private void setupMotionProfilePID(double p, double i, double d, double f, int iZone) {
		// motion profiling PID is stored in slot 1, main PID is slot 0
		rightTalonMaster.setProfile(1);
		leftTalonMaster.setProfile(1);
//		rightTalonMaster.setPID(p, i, d, f, iZone, 0, 1);
//		leftTalonMaster.setPID(p, i, d, f, iZone, 0, 1);
		rightTalonMaster.setF(f);
		rightTalonMaster.setP(p);
		rightTalonMaster.setI(i);
		rightTalonMaster.setD(d);
		rightTalonMaster.setIZone(iZone);
		leftTalonMaster.setF(f);
		leftTalonMaster.setP(p);
		leftTalonMaster.setI(i);
		leftTalonMaster.setD(d);
		leftTalonMaster.setIZone(iZone);
		// make sure profile gets set back, it does not select it and just uses it
		rightTalonMaster.setProfile(0);
		leftTalonMaster.setProfile(0);
	}
	
	private void setupVelocityClosedLoop(double p, double i, double d, double f, int iZone) {
		rightTalonMaster.changeControlMode(TalonControlMode.Speed);
		leftTalonMaster.changeControlMode(TalonControlMode.Speed);
		rightTalonMaster.setProfile(0);
		rightTalonMaster.setF(f);
		rightTalonMaster.setP(p);
		rightTalonMaster.setI(i);
		rightTalonMaster.setD(d);
		rightTalonMaster.setIZone(iZone);
		leftTalonMaster.setProfile(0);
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
		if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
			if (RobotMap.practiceRobot) {
				setupVelocityClosedLoop(kPPractice, kIPractice, kDPractice, kFPractice, kIZonePractice);
			} else {
				setupVelocityClosedLoop(kPCompetition, kICompetition, kDCompetition, kFCompetition, kIZoneCompetition);
			}
			rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f); // this is changed in motion profiling mode, and ignored by talon in open loop
			leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		}
	}
	
	/**
	 * Sets the current control mode to open loop
	 * 
	 * Should only ever be called by ApplyOpenLoopSwitch or stopMotionProfile
	 */
	public void useOpenLoop() {
		if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
			rightTalonMaster.changeControlMode(TalonControlMode.PercentVbus);
			leftTalonMaster.changeControlMode(TalonControlMode.PercentVbus);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void driveInchesPerSec(double left, double right) {
    		drive((left/(wheelDiameter*Math.PI))*60, (left/(wheelDiameter*Math.PI))*60);
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
    	if (Robot.oi.getDriveEnabled() && currentControlMode == DriveControlMode.STANDARD_DRIVE) {
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
    	if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
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
     * Sets the PID parameters for the current control mode, useful for tuning.
     * Calling this effects everything using the subsystem, use with care.
     * @param p
     * @param i
     * @param d
     * @param f
     * @param iZone
     */
    public void setPID(double p, double i, double d, double f, int iZone) {
    	int profile = currentControlMode == DriveControlMode.STANDARD_DRIVE ? 0 : 1;
    	rightTalonMaster.setPID(p, i, d, f, iZone, 0, profile);
    	leftTalonMaster.setPID(p, i, d, f, iZone, 0, profile);
    }
    
    public void changeSensorRate(int ms) {
    		leftTalonMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, ms);
    		rightTalonMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, ms);
    }
    public void resetSensorRate() {
    		leftTalonMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 100);
		rightTalonMaster.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 100);
    }
    
    
    /**
     * Uses the talon native distance close loop to drive a distance.
     * @param inches
     */
    public void driveDistance(double inches, boolean motionMagic) {
    	if (currentControlMode == DriveControlMode.STANDARD_DRIVE && Robot.oi.getDriveEnabled()) {
    		currentControlMode = DriveControlMode.DISTANCE_CLOSE_LOOP;
    		rightTalonMaster.setProfile(1);
    		leftTalonMaster.setProfile(1);
    		rightTalonMaster.configNominalOutputVoltage(+1.0f, -1.0f);
    		leftTalonMaster.configNominalOutputVoltage(+1.0f, -1.0f);
    		if (RobotMap.practiceRobot) {
    			rightTalonMaster.setAllowableClosedLoopErr(kAllowableErrorPracticeDistance);
    			leftTalonMaster.setAllowableClosedLoopErr(kAllowableErrorPracticeDistance);
    		} else {
    			rightTalonMaster.setAllowableClosedLoopErr(kAllowableErrorCompetitionDistance);
    			leftTalonMaster.setAllowableClosedLoopErr(kAllowableErrorCompetitionDistance);
    		}
    		if (motionMagic) {
    			rightTalonMaster.changeControlMode(TalonControlMode.MotionMagic);
    			leftTalonMaster.changeControlMode(TalonControlMode.MotionMagic);
    		} else {
    			rightTalonMaster.changeControlMode(TalonControlMode.Position);
    			leftTalonMaster.changeControlMode(TalonControlMode.Position);
    		}
    		rightTalonMaster.set(inches/(Math.PI*wheelDiameter)*-1);
    		leftTalonMaster.set(inches/(Math.PI*wheelDiameter));
    	}
    }

    public void stopDistanceDrive() {
    	if (currentControlMode == DriveControlMode.DISTANCE_CLOSE_LOOP) {
    		rightTalonMaster.set(0);
    		leftTalonMaster.set(0);
    		rightTalonMaster.setAllowableClosedLoopErr(0); // motion profiling does not use this
    		leftTalonMaster.setAllowableClosedLoopErr(0);
    		currentControlMode = DriveControlMode.STANDARD_DRIVE; // this needs to be changed before calling useClosed/OpenLoop so that they work
    		if (Robot.oi.getOpenLoop()) {
    			useOpenLoop();
    		} else {
    			useClosedLoop();
    		}
    	}
    }

    
    /**
     * Loads the passed trajectory, and activates motion profiling mode
     * @param trajectory
     */
    @Deprecated
    public void loadMotionProfile(Trajectory trajectory) {
    	loadMotionProfile(trajectory, false);
    }
    /**
     * Loads the passed trajectory, and activates motion profiling mode
     * @param trajectory
     */
    @Deprecated
    public void loadMotionProfile(Trajectory trajectory, boolean flipLeftRight) {
    	// Removed for phoenix compatibility
    }
    
    @Deprecated
    public void startMotionProfile() {
    	// Removed for phoenix compatibility
    }
    
    @Deprecated
    public void stopMotionProfile() {
    	// Removed for phoenix compatibility
    }
    
    /**
     * Returns true if both sides have reached their last trajectory point
     * @return
     */
    public boolean isMotionProfileComplete() {
    	// Removed for phoenix compatibility
    	return true;
    }    
    
    private enum DriveControlMode {
    	STANDARD_DRIVE, MOTION_PROFILE, DISTANCE_CLOSE_LOOP
    }
}

