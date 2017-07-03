package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
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
    
 // this is done separately because the constructor can't refer to the non-static OI method, since subsystems are static
    public void initLED() {
    	if (!RobotMap.practiceRobot) {
    		Robot.oi.updateLED(OILED.TOP_GEAR_OPEN, false);
			Robot.oi.updateLED(OILED.TOP_GEAR_CLOSE, true);
    	}
    }
    
    public void open() {
    	solenoid.set(DoubleSolenoid.Value.kForward);
    	Robot.oi.updateLED(OILED.TOP_GEAR_OPEN, true);
		Robot.oi.updateLED(OILED.TOP_GEAR_CLOSE, false);
    }
    
    public void close() {
    	solenoid.set(DoubleSolenoid.Value.kReverse);
    	Robot.oi.updateLED(OILED.TOP_GEAR_OPEN, false);
		Robot.oi.updateLED(OILED.TOP_GEAR_CLOSE, true);
    }
    
    public void disable() {
    	solenoid.set(DoubleSolenoid.Value.kOff);
    	Robot.oi.updateLED(OILED.TOP_GEAR_OPEN, false);
		Robot.oi.updateLED(OILED.TOP_GEAR_CLOSE, false);
    }
}

