package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * If centered behind the airship, cross auto line
 * Needs specific values
 */
public class CrossLineBehindAirship extends CommandGroup {

    public CrossLineBehindAirship() {
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
    	addSequential(new DriveDistanceOnHeading(1)); // drive away from wall
    	addSequential(new TurnAndDriveDistance(1, -90)); // turn and drive so not behind airship
    	addSequential(new TurnAndDriveDistance(1, 90)); // turn back and cross line
    }
}
