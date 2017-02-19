package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ball shooter
 */
public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final int currentLimit = 50;
	private final boolean enableCurrentLimit = false;
	private final boolean brakeMode = false;
	private final double shooterSpeed = 1;
	private final boolean lockShooterSpeed = false;
	
	private CANTalon shooterMaster;
	private CANTalon shooterSlave;
	private Counter sensorCounter;
	
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
    		shooterMaster.set(shooterSpeed*-1);
    	} else {
    		shooterMaster.set(Robot.oi.getShooterSpeed()*-1);
    	}
    }
    
    public void stop() {
    	shooterMaster.set(0);
    }
    
    public double getSpeed() {
    	//return 1/sensorCounter.getPeriod()/3;
    	return sensorCounter.getRate();
    }
    
    public int getCount() {
    	return sensorCounter.get();
    }
    
    public double getPeriod() {
    	return sensorCounter.getPeriod();
    }
}

