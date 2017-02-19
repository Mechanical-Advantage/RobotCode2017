package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Drives to the boiler and shoots preloaded balls, start backwards
 * Needs distances, angles defined
 */
public class DriveToBoilerShootBalls extends CommandGroup {
	
	public double turnAmount = 1; // define for red alliance, remember robot starts backwards
	private final double shootTime = 10; // in seconds

    public DriveToBoilerShootBalls(boolean blueAlliance) {
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
    	if (blueAlliance) {
    		turnAmount*=-1;
    	}
    	addSequential(new DriveDistanceOnHeading(1)); // drive away from wall
    	addSequential(new TurnAndDriveDistance(-1, turnAmount)); // 
    	addParallel(new RunCommandForTime(new RunShooter(), shootTime));
    	addParallel(new RunCommandForTime(new RunLoader(false), shootTime));
    	addParallel(new RunCommandForTime(new RunTrigger(false), shootTime));
    }
}
