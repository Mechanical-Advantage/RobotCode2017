package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear on the side of the airship using motion profiling optionally
 */
public class PlaceGearSideOfAirship2 extends Command {
	
	private double turnAmount = 60; // define for left side, flipped for right side
	private boolean commandStarted = false;
	
	private PlaceGearSideOfAirshipGroup placeGearGroup;
	private boolean motionProfiling;

    public PlaceGearSideOfAirship2(Boolean motionProfiling) {
    	super("PlaceGearSideOfAirshipMP");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.motionProfiling = motionProfiling;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	placeGearGroup = new PlaceGearSideOfAirshipGroup(Robot.secondStageSide, Robot.rightSideSelected);
    	placeGearGroup.start();
    	commandStarted = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return commandStarted && !placeGearGroup.isRunning();
    }

    // Called once after isFinished returns true
    protected void end() {
    	commandStarted = false;
    	placeGearGroup.cancel();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    private class PlaceGearSideOfAirshipGroup extends CommandGroup {
    	public PlaceGearSideOfAirshipGroup(String secondStage, Boolean rightSide) {
    		String extraSecondStage = "";
    		
    		if (motionProfiling) {
    			addSequential(new RunMotionProfileFromFile("sideAuto", rightSide));
    		} else {
    			addSequential(new DriveDistanceOnHeading(/*66.98*/69.28)); // drive forward away from wall
    			addSequential(new TurnAndDriveDistance(68.8, rightSide ? turnAmount*=-1 : turnAmount)); // turn and drive into airship
    		}
    		if (!RobotMap.practiceRobot) {
    			addSequential(new WiggleUntilGearSpring());
    			addSequential(new Delay(0.5)); // allow the robot to stop moving
            	addParallel(new ExpelGear(), 2); // place gear
            	addSequential(new Delay(1)); // allow the gear to leave before backing up
    		}
        	if (secondStage != null) {
        		addSequential(new DriveForTime(-0.4, 1.2)); // back up to prepare for the motion profile
        		
        		if (secondStage.contains(":")) {
        			extraSecondStage = secondStage.split(":")[1];
        			secondStage = secondStage.split(":")[0];
        		}
        		
        		addSequential(new RunMotionProfileFromFile(secondStage, rightSide));
        		
        		switch (extraSecondStage) {
        			case "edge":
        				int edgeTurnAmount = 0;
        				addSequential(new DriveDistanceOnHeading(144, rightSide ? edgeTurnAmount*=-1 : edgeTurnAmount));
        				break;
        			case "center":
        				int centerTurnAmount = 45;
        				addSequential(new DriveDistanceOnHeading(216, rightSide ? centerTurnAmount*=-1 : centerTurnAmount));
        				break;
        			default:
        				break;
        		}
        	} else {
        		addSequential(new DriveForTime(-0.25, 1)); // back up to allow the gear to be lifted
        	}
    	}
    	
    }
}
