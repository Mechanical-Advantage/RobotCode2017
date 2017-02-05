package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Closes the gear handler
 */
public class CloseGearHandler extends InstantCommand {

    public CloseGearHandler() {
        super("CloseGearHandler");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.gearHandlerSubsystem);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.gearHandlerSubsystem.close();
    }

}
