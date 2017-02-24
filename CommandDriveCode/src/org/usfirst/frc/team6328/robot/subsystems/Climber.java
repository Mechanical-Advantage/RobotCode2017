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
	private final double intakeSpeed = 1; // speed when run as intake
	private final int currentLimit = 50;
	private final boolean enableCurrentLimit = true;
	private final boolean brakeMode = true;
	
	private CANTalon climberTalon;
	
	public Climber() {
		if (!RobotMap.practiceRobot) {
			climberTalon = new CANTalon(RobotMap.climber);
			climberTalon.EnableCurrentLimit(enableCurrentLimit);
			climberTalon.setCurrentLimit(currentLimit);
			climberTalon.enableBrakeMode(brakeMode);
			//climberTalon.reverseOutput(true); // not working
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
    	climberTalon.set(speed*-1); // inverted
    }
    
    public void hold() {
    	climberTalon.enable();
    	climberTalon.set(holdSpeed*-1);
    }
    
    public void runAsIntake() {
    	climberTalon.enable();
    	climberTalon.set(intakeSpeed*-1);
    }
    
    public void stop() {
    	climberTalon.disable();
    }
    
    public double getCurrent() {
    	return climberTalon.getOutputCurrent();
    }
    
    public double getVoltage() {
    	return climberTalon.getOutputVoltage();
    }
}

