package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ball shooter
 */
public class Shooter extends Subsystem implements PIDSource {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final int currentLimit = 40;
	private final boolean enableCurrentLimit = true;
	private final boolean brakeMode = false;
	private final double shooterSpeedNormal = -1;
	private final boolean lockShooterSpeed = false;
	private final double shooterSpeedOpenLoop = -0.45; // speed used when banner is disabled
	private final int maxSpeed = 150; // higher numbers are treated as noise and will be ignored

	
	private CANTalon shooterMaster;
	private CANTalon shooterSlave;
	private Counter sensorCounter;
	private double lastSpeed = 0;
	
	public Shooter() {
		if (!RobotMap.practiceRobot) {
			shooterMaster = new CANTalon(RobotMap.shooterMaster);
			shooterMaster.EnableCurrentLimit(enableCurrentLimit);
			shooterMaster.setCurrentLimit(currentLimit);
			shooterMaster.reverseOutput(false); // setting this to true does nothing, don't know why
			shooterMaster.enableBrakeMode(brakeMode);
			shooterSlave = new CANTalon(RobotMap.shooterSlave);
			shooterSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
			shooterSlave.set(RobotMap.shooterMaster);
			shooterSlave.reverseOutput(true);
			shooterSlave.EnableCurrentLimit(enableCurrentLimit);
			shooterSlave.setCurrentLimit(currentLimit);
			shooterSlave.enableBrakeMode(brakeMode);
			
			sensorCounter = new Counter(9);
			sensorCounter.setSemiPeriodMode(false);
			sensorCounter.setSamplesToAverage(2);
			sensorCounter.setDistancePerPulse(1.0);
			
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void run() {
    	if (lockShooterSpeed) {
    		shooterMaster.set(shooterSpeedNormal*-1);
    	} else {
    		shooterMaster.set(Robot.oi.getShooterSpeed()*-1);
    	}
    }
    
    /**
     * Runs the shooter at a predefined speed
     * @param speed
     */
    public void run(double speed) {
    	shooterMaster.set(speed*-1); // everything is backwards
    }
    
    public void runOpenLoop() {
    	shooterMaster.set(shooterSpeedOpenLoop);
    }
    
    public void stop() {
    	shooterMaster.set(0);
    }
    
    public double getSpeed() {
    	//return 1/sensorCounter.getPeriod()/3;
    	double currentSpeed = sensorCounter.getRate();
    	if (currentSpeed<maxSpeed) {
    		lastSpeed = currentSpeed;
    		return currentSpeed;
    	} else {
    		return lastSpeed;
    	}
    }
    
    public int getCount() {
    	return sensorCounter.get();
    }
    
    public double getPeriod() {
    	return sensorCounter.getPeriod();
    }

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// type can't be changed
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kRate;
	}

	@Override
	public double pidGet() {
		return getSpeed();
	}
}

