package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables or disables closed loop drive
 * 
 * Should only be called from OI because the switch on the operator console is switched,
 * calling and setting a state different than the switch will not work correctly
 */
public class ApplyOpenLoopSwitch extends InstantCommand {

	private boolean enableClosedLoopDrive;
	
    public ApplyOpenLoopSwitch(boolean enable) {
        super("ApplyOpenLoopSwitch");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
        enableClosedLoopDrive = enable;
    }

    // Called once when the command executes
    protected void initialize() {
    	if (enableClosedLoopDrive) {
    		Robot.driveSubsystem.useClosedLoop();
    	} else {
    		Robot.driveSubsystem.useOpenLoop();
    	}
    }

}
