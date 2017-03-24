package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Operates lower gear system that places the gear
 */
public class GearExpeller extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private long filterNanoseconds = 400000000; // 400ms
	
	DoubleSolenoid solenoid;
	DigitalInput sensor1;
	DigitalInput sensor2;
	DigitalGlitchFilter filter1; // at the moment we don't know if we need this
	DigitalGlitchFilter filter2;
	
	public GearExpeller() {
		if (!RobotMap.practiceRobot) {
			solenoid = new DoubleSolenoid(RobotMap.gearExpellerSolenoid1, RobotMap.gearExpellerSolenoid2);
			sensor1 = new DigitalInput(RobotMap.gearExpellerSensor1);
			sensor2 = new DigitalInput(RobotMap.gearExpellerSensor2);
			filter1 = new DigitalGlitchFilter();
			filter2 = new DigitalGlitchFilter();
			filter1.setPeriodNanoSeconds(filterNanoseconds);
			filter2.setPeriodNanoSeconds(filterNanoseconds);
			filter1.add(sensor1);
			filter2.add(sensor2);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void eject() {
    	solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void close() {
    	solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void disable() {
    	solenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    
    /**
     * Returns whether the sensor senses the lift spring
     * Will return true if either of the two sensors senses it
     * 
     * Note: Also returns true if either sensor in not connected
     */
    public boolean getSpringSensor() {
    	return sensor1.get() || sensor2.get();
    }
}

