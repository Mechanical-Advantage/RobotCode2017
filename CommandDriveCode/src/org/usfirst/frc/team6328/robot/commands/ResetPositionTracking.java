package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Resets the drivetrain position tracking
 */
public class ResetPositionTracking extends InstantCommand {

    public ResetPositionTracking() {
        super("ResetPositionTracking");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    		Robot.driveSubsystem.resetPositionTracking();
    		Robot.ahrs.zeroYaw();
    		Robot.ahrs.resetDisplacement();
    }

}
