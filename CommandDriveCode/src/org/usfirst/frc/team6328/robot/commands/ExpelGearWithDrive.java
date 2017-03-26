package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Expels the gear, stops, and drive backward
 */
public class ExpelGearWithDrive extends CommandGroup {

    public ExpelGearWithDrive() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addSequential(new DriveForTime(0, 0)); // stop
    	addParallel(new RunCommandForTime(new ExpelGear(), 2)); // place gear
    	addSequential(new Delay(1)); // allow the gear to leave before backing up
    	addSequential(new DriveForTime(-0.25, 1)); // back up to allow the gear to be lifted
    }
}
