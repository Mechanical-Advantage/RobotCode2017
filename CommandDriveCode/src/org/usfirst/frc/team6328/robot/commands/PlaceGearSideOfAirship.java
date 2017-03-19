package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear when to the left on right of airship on left or right pegs
 * Needs starting location, distances, angles defined
 */
public class PlaceGearSideOfAirship extends CommandGroup {
	
	private double turnAmount = 60; // define for right side, flipped for left side
	private double shakeSpeed = -0.1; // used when shaking back and forth
	private double offset = 0.1; // added to speed on both sides

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
    	if (rightSide) { // on right of airship, robot needs to turn left
    		turnAmount*=-1;
    	}
    	addSequential(new DriveDistanceOnHeading(66.98)); // drive forward away from wall
    	addSequential(new TurnAndDriveDistance(66.8, turnAmount)); // turn and drive into airship, define distance
    	addSequential(new OpenTopGear());
    	addSequential(new DriveDistanceOnHeading(5, turnAmount)); // slow drive coming in, align to heading
    	addSequential(new Delay(0.5)); // wait for human player to lift if possible
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed, offset, 0.1));
    	addSequential(new Delay(1)); // if previous shake fixed, allow lifting
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed*-1, offset, 0.2)); // turn the other way
    	addSequential(new Delay(1)); // time to possibly lift
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed, offset, 0.1)); // re-center
    }
}
