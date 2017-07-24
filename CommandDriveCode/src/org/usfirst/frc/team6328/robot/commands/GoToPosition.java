package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Takes an x y position and navigates there, currently open loop
 */
public class GoToPosition extends Command {
	
	private double x;
	private double y;
	private Command driveCommand;
	private boolean commandStarted = false;

	public GoToPosition(double x, double y) {
		super("GoToPosition");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		this.x = x;
		this.y = y;
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    		double angle = (Math.toDegrees(Math.atan2(y-Robot.driveSubsystem.getPositionY(), x-Robot.driveSubsystem.getPositionX()))-90)*-1;
		double distanceX = Math.abs(x-Robot.driveSubsystem.getPositionX());
		double distanceY = Math.abs(y-Robot.driveSubsystem.getPositionY());
    		double distance = Math.sqrt((distanceX * distanceX) + (distanceY * distanceY));
		System.out.println("Angle: " + angle);
		System.out.println("Distance: " + distance);
		driveCommand = new TurnAndDriveDistance(distance, angle, true, true);
		driveCommand.start();
		commandStarted = true;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return commandStarted && !driveCommand.isRunning();
    }

    // Called once after isFinished returns true
    protected void end() {
    		commandStarted = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		driveCommand.cancel();
    		commandStarted = false;
    }
}
