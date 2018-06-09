package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.LogitechOI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.CANTalon.CANTalon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ball intake from ground
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private final double speed = 0.6;
	private final double shootSpeed = 1;
	private final int currentLimit = 50;
	private final boolean enableCurrentLimit = false;
	private final boolean brakeMode = false;
	
	private CANTalon intakeTalon;
	
	public Intake() {
		if (!RobotMap.practiceRobot) {
			intakeTalon = new CANTalon(RobotMap.intake);
			intakeTalon.EnableCurrentLimit(enableCurrentLimit);
			intakeTalon.setCurrentLimit(currentLimit);
			intakeTalon.enableBrakeMode(brakeMode);
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    // this is done separately because the constructor can't refer to the non-static OI method, since subsystems are static
    public void initLED() {
    	if (!RobotMap.practiceRobot) {
    		Robot.oi.updateLED(OILED.INTAKE_OFF, true);
    		Robot.oi.updateLED(OILED.INTAKE_ON, false);
    	}
    }
    
    public void run() {
    	intakeTalon.enable();
    	intakeTalon.set(speed);
    	Robot.oi.updateLED(OILED.INTAKE_OFF, false);
		Robot.oi.updateLED(OILED.INTAKE_ON, true);
    }
    
    public void runForShoot() {
    	intakeTalon.enable();
    	intakeTalon.set(shootSpeed);
    	Robot.oi.updateLED(OILED.INTAKE_OFF, false);
		Robot.oi.updateLED(OILED.INTAKE_ON, true);
    }
    
    public void stop() {
    	intakeTalon.disable();
    	Robot.oi.updateLED(OILED.INTAKE_OFF, true);
		Robot.oi.updateLED(OILED.INTAKE_ON, false);
    }
    
    public void reverse() {
    	intakeTalon.enable();
    	intakeTalon.set(speed*-1);
    	Robot.oi.updateLED(OILED.INTAKE_OFF, false);
		Robot.oi.updateLED(OILED.INTAKE_ON, true);
    }
    
    public double getCurrent() {
    	return intakeTalon.getOutputCurrent();
    }
    
    public double getVoltage() {
    	return intakeTalon.getOutputVoltage();
    }
}

