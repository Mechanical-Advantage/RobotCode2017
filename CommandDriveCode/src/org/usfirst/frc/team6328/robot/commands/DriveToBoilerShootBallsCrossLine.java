package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Drives to the boiler, shoots balls, and crosses line
 * Needs distance
 */
public class DriveToBoilerShootBallsCrossLine extends CommandGroup {
	
	private DriveToBoilerShootBalls driveAndShoot;

    public DriveToBoilerShootBallsCrossLine(boolean blueAlliance) {
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
    	driveAndShoot = new DriveToBoilerShootBalls(blueAlliance);
    	addSequential(driveAndShoot);
    	addSequential(new DriveDistanceOnHeading(-1)); // back up from boiler
    	addSequential(new TurnAndDriveDistance(-1, driveAndShoot.turnAmount*-1)); // turn back and drive backwards across line
    }
}
