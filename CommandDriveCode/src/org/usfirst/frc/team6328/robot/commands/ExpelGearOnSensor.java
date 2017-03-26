package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Waits until the sensor is tripped, then expel the gear and stop and back up
 */
public class ExpelGearOnSensor extends Command {
	
	ExpelGear simpleGearExpel = new ExpelGear();
	ExpelGearWithDrive gearExpel = new ExpelGearWithDrive();
	
    public ExpelGearOnSensor() {
    	super("ExpelGearOnSensor");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// ensure that the simple placing gets stopped when switching.
    	// when switching to simple, it will always run immediately, so not needed that way
    	// but when switching to smart, it won't start until the sensor so it needs to be explicitly stopped
    	if (Robot.oi.getSmartGearExpel() && simpleGearExpel.isRunning()) {
    		simpleGearExpel.cancel();
    	}
    	if (Robot.oi.getSmartGearExpel() && Robot.expelGearSubsystem.getSpringSensor()) {
    		gearExpel.start();
    	} else if (!Robot.oi.getSmartGearExpel()) {
    		simpleGearExpel.start();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	gearExpel.cancel();
    	simpleGearExpel.cancel();
    }
}
