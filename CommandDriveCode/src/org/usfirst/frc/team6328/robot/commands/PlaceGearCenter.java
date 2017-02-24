package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear when the robot is centered behind the airship
 */
public class PlaceGearCenter extends CommandGroup {
	
	private double shakeSpeed = -0.1; // used when shaking back and forth, make negative to turn left first
	private double offset = 0.1; // added to speed on both sides

    public PlaceGearCenter() {
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
    	addSequential(new DriveDistanceOnHeading(71.5));
    	addSequential(new DriveDistanceOnHeading(5)); // slowly drive the rest of the way
    	addSequential(new Delay(2)); // wait for human player to lift if possible
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed, offset, 0.1));
    	addSequential(new Delay(2)); // if previous shake fixed, allow lifting
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed*-1, offset, 0.2)); // turn the other way
    	addSequential(new Delay(2)); // time to possibly lift
    	addSequential(new SpinCenterOffsetForTime(shakeSpeed, offset, 0.1)); // re-center
    }
}
