package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ball intake from ground
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final double speed = 1;
	
	private CANTalon intakeTalon;
	
	public Intake() {
		if (!RobotMap.practiceRobot) {
			intakeTalon = new CANTalon(RobotMap.intake);
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void run() {
    	intakeTalon.enable();
    	intakeTalon.set(speed);
    }
    
    public void stop() {
    	intakeTalon.disable();
    }
    
    public void reverse() {
    	intakeTalon.enable();
    	intakeTalon.set(speed*-1);
    }
}
