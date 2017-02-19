package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear from side, backs up and crosses line
 * Needs distance defined
 */
public class PlaceGearSideOfAirshipCrossLine extends CommandGroup {
	
	private PlaceGearSideOfAirship gearPlacer;

    public PlaceGearSideOfAirshipCrossLine(boolean rightSide) {
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
    	gearPlacer = new PlaceGearSideOfAirship(rightSide);
    	addSequential(gearPlacer);
    	addSequential(new DriveDistanceOnHeading(-1)); // back up from airship
    	addSequential(new TurnAndDriveDistance(1, gearPlacer.turnAmount*-1)); // drive over line, turning back from airship
    }
}
