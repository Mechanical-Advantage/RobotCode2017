package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Operates lower gear system that places the gear
 */
public class GearExpeller extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	DoubleSolenoid solenoid;
	
	public GearExpeller() {
		if (!RobotMap.practiceRobot) {
			solenoid = new DoubleSolenoid(RobotMap.gearExpellerSolenoid1, RobotMap.gearExpellerSolenoid2);
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
}

