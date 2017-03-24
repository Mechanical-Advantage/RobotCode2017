package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear when to the left on right of airship on left or right pegs
 */
public class PlaceGearSideOfAirship extends CommandGroup {
	
	private double turnAmount = 60; // define for right side, flipped for left side

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
    	addSequential(new TurnAndDriveDistance(70.8, turnAmount)); // turn and drive into airship
//    	addSequential(new DriveUntilGearSpring(turnAmount)); // slowly drive into the spring
    	addSequential(new WiggleUntilGearSpring());
    	addSequential(new Delay(0.5)); // allow the robot to stop moving
    	addParallel(new ExpelGear()); // place gear
    	addSequential(new Delay(1)); // allow the gear to leave before backing up
    	addSequential(new DriveForTime(-0.25, 1)); // back up to allow the gear to be lifted
    }
}
