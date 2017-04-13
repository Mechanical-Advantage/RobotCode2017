package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Loads a motion profile from the specified file name (in /home/lvuser/motionprofiles)
 */
public class RunMotionProfileFromFile extends Command {
	
	Trajectory trajectory;
	boolean flipLeftRight;

    public RunMotionProfileFromFile(String fileName, boolean flipLeftRight) {
    	super("RunMotionProfileFromFile");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    	this.flipLeftRight = flipLeftRight;
    	File file = new File("/home/lvuser/motionprofiles/" + fileName + ".traj");
    	trajectory = Pathfinder.readFromFile(file);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Running profile");
    	Robot.driveSubsystem.loadMotionProfile(trajectory, flipLeftRight);
    	Robot.driveSubsystem.startMotionProfile();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.driveSubsystem.isMotionProfileComplete();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSubsystem.stopMotionProfile();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
