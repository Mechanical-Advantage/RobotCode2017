package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Drives to the boiler, shoots balls, and turns around and places gear
 * Needs distance, amount to turn around
 */
public class DriveToBoilerShootBallsPlaceGear extends CommandGroup {

    public DriveToBoilerShootBallsPlaceGear(boolean blueAlliance) {
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
    	addSequential(new DriveToBoilerShootBallsCrossLine(blueAlliance));
    	addSequential(new DriveDistanceOnHeading(-1)); // back up from boiler
    	addSequential(new TurnAndDriveDistance(1, 180)); // turn around and drive forward into lift, will not turn 180
    }
}
