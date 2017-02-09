package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables or disables closed loop drive
 */
public class EnableClosedLoop extends InstantCommand {

	private boolean enableClosedDrive;
	
    public EnableClosedLoop(boolean enable) {
        super("EnableClosedLoop");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        enableClosedDrive = enable;
    }

    // Called once when the command executes
    protected void initialize() {
    	if (enableClosedDrive) {
    		Robot.driveSubsystem.useClosedLoop();
    	} else {
    		Robot.driveSubsystem.useOpenLoop();
    	}
    }

}
