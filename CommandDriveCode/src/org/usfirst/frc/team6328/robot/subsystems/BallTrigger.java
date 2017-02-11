package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Trigger that loads balls into shooter
 */
public class BallTrigger extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final double speed = 1;
	private final int currentLimit = 50;
	private final boolean enableCurrentLimit = false;
	
	private CANTalon triggerTalon;
	
	public BallTrigger() {
		if(!RobotMap.practiceRobot) {
			triggerTalon = new CANTalon(RobotMap.trigger);
			triggerTalon.EnableCurrentLimit(enableCurrentLimit);
			triggerTalon.setCurrentLimit(currentLimit);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void forward() {
    	triggerTalon.enable();
    	triggerTalon.set(speed);
    }
    
    public void reverse() {
    	triggerTalon.enable();
    	triggerTalon.set(speed*-1);
    }
    
    public void stop() {
    	triggerTalon.disable();
    }
}

