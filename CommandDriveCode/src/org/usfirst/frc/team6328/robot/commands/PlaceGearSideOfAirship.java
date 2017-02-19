package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear when to the left on right of airship on left or right pegs
 * Needs starting location, distances, angles defined
 */
public class PlaceGearSideOfAirship extends CommandGroup {
	
	public double turnAmount = 1; // define for left side, flipped for right side

    public PlaceGearSideOfAirship(boolean rightSide) {
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
    	if (rightSide) {
    		turnAmount*=-1;
    	}
    	addSequential(new DriveDistanceOnHeading(1)); // drive forward away from wall
    	addSequential(new TurnAndDriveDistance(1, turnAmount)); // turn and drive into airship, define distance
    }
}
