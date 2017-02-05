package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ball loader from hopper to trigger
 */
public class Loader extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final double speed = 1;
	
	private CANTalon loaderTalon;
	
	public Loader() {
		if (!RobotMap.practiceRobot) {
			loaderTalon = new CANTalon(RobotMap.loader);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void run() {
    	loaderTalon.enable();
    	loaderTalon.set(speed);
    }
    
    public void stop() {
    	loaderTalon.disable();
    }
    
    public void reverse() {
    	loaderTalon.enable();
    	loaderTalon.set(speed*-1);
    }
}

