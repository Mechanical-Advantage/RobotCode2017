package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Operates upper gear system that receives the gear
 */
public class TopGear extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	DoubleSolenoid solenoid;
	
	public TopGear() {
		if (!RobotMap.practiceRobot) {
			solenoid = new DoubleSolenoid(RobotMap.topGearSolenoid1, RobotMap.topGearSolenoid2);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void open() {
    	solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void close() {
    	solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void disable() {
    	solenoid.set(DoubleSolenoid.Value.kOff);
    }
}

