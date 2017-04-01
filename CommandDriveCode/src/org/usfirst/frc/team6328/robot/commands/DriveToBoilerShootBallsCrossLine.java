package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Drives to the boiler and shoots preloaded balls, start backwards
 */
public class DriveToBoilerShootBallsCrossLine extends CommandGroup {
	
	public double turnAmount = -45; // define for red alliance, remember robot starts backwards
	private final double shootTime = 4; // in seconds
	
	private Command runShooter;

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
    	switch (RobotMap.shooterControlType) {
		case PID:
			runShooter = new RunShooterPID();
			break;
		case SIMPLE_BANG_BANG:
			runShooter = new RunShooterSimpleBangBang();
			break;
		case FAST_BANG_BANG:
		default:
			runShooter = new RunShooterFastBangBang();
			break;
		}
    	
    	if (blueAlliance) {
    		turnAmount*=-1;
    	}
    	addParallel(runShooter);
    	addSequential(new DriveDistanceOnHeading(-26)); // drive away from wall
    	//addSequential(new TurnAndDriveDistance(-1, turnAmount));
    	addSequential(new TurnToAngle(turnAmount)); // face boiler
    	addSequential(new DriveForTime(0.2, 1.5)); // quickly just run into the boiler, don't worry about precision
    	//addSequential(new SpinCenterOffsetForTime(0.2, 0.2, 3)); // rotate to boiler
    	//addParallel(new RunCommandForTime(new RunShooter(), shootTime));
    	addSequential(new Delay(0.5));
    	addParallel(new RunCommandForTime(new RunLoader(false), shootTime));
    	addSequential(new RunCommandForTime(new RunTrigger(false), shootTime));
    	addSequential(new DriveDistanceOnHeading(-100, 0)); // back up from boiler
    }
}
