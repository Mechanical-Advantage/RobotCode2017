package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Places the gear centered behind the airship using motion profiling optionally
 */
public class PlaceGearCenter2 extends Command {
	
	private PlaceGearCenterGroup placeGearGroup;
	private boolean commandStarted = false;

    public PlaceGearCenter2() {
    	super("PlaceGearCenter2");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	placeGearGroup = new PlaceGearCenterGroup(Robot.secondStageCenter, Robot.rightSideSelected);
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
    
    private class PlaceGearCenterGroup extends CommandGroup {
    	public PlaceGearCenterGroup(String secondStage, Boolean rightSide) {
    		String extraSecondStage = "";
    		
    		addSequential(new DriveDistanceOnHeading(75.5));
    		if (!RobotMap.practiceRobot) {
    			addSequential(new WiggleUntilGearSpring());
    			addSequential(new Delay(0.5)); // allow the robot to stop moving
            	addParallel(new ExpelGear(), 2); // place gear, closing after drive finished
            	addSequential(new Delay(1)); // allow the gear to leave before backing up
    		}
        	if (secondStage != null) {
        		int backUpAngle = -90; // for left side
//        		addSequential(new DriveForTime(-0.4, 1.6)); // back up to prepare for the motion profile
        		addSequential(new DriveDistanceOnHeading(-48, rightSide ? backUpAngle*=-1 : backUpAngle)); // back up and face to the side
        		
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
