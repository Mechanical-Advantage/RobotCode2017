package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Closes the top gear handler
 */
public class CloseTopGear extends InstantCommand {

    public CloseTopGear() {
        super("CloseTopGear");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.topGearSubsystem);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.topGearSubsystem.close();
    }

}
