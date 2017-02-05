package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.commands.ClimbWithJoystick;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Robot Climber
 */
public class Climber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final double holdSpeed = 0.2;
	private final double intakeSpeed = -0.1;
	private final int currentLimit = 0; // in amps
	
	private CANTalon climberTalon;
	
	public Climber() {
		if (!RobotMap.practiceRobot) {
			climberTalon = new CANTalon(RobotMap.climber);
			climberTalon.EnableCurrentLimit(true);
			climberTalon.setCurrentLimit(currentLimit);
		}
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	if (!RobotMap.practiceRobot) {
    		setDefaultCommand(new ClimbWithJoystick());
    	}
    }
    
    public void run(double speed) {
    	climberTalon.enable();
    	climberTalon.set(speed);
    }
    
    public void hold() {
    	climberTalon.enable();
    	climberTalon.set(holdSpeed);
    }
    
    public void runAsIntake() {
    	climberTalon.enable();
    	climberTalon.set(intakeSpeed);
    }
    
    public void stop() {
    	climberTalon.disable();
    }
}

