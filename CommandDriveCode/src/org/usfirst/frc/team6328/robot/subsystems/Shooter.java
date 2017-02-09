package org.usfirst.frc.team6328.robot.subsystems;

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
	
	private CANTalon shooterMaster;
	private CANTalon shooterSlave;
	private Counter sensorCounter;
	
	public Shooter() {
		if (!RobotMap.practiceRobot) {
			shooterMaster = new CANTalon(RobotMap.shooterMaster);
			shooterSlave = new CANTalon(RobotMap.shooterSlave);
			shooterSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
			shooterSlave.set(RobotMap.shooterMaster);
			
			sensorCounter = new Counter(0);
			
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void run() {
    	shooterMaster.set(1);
    }
    
    public void stop() {
    	shooterMaster.set(0);
    }
    
    public double getSpeed() {
    	return 1/sensorCounter.getPeriod()/3;
    }
    
    public int getCount() {
    	return sensorCounter.get();
    }
    
    public double getPeriod() {
    	return sensorCounter.getPeriod();
    }
}

