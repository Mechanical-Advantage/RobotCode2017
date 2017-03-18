package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * Motion profile test command
 */
public class MotionProfileTest extends Command {

    public MotionProfileTest() {
    	super("MotionProfileTest");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Waypoint[] points = new Waypoint[] {
        		new Waypoint(0, 0, 0),
        		new Waypoint(60, -24, Pathfinder.d2r(-45)),
        		new Waypoint(72, -36, Pathfinder.d2r(-45)),
        };
        
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 124.4, 62.2, 1866);
        Trajectory trajectory = Pathfinder.generate(points, config);
        /*for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = trajectory.get(i);

            System.out.printf(i+1 + ": %f,%f,%f,%f,%f,%f,%f,%f\n", 
                seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
                    seg.acceleration, seg.jerk, seg.heading);
        }*/
        File csvFile = new File("/tmp/trajectory.csv");
        Pathfinder.writeToCSV(csvFile, trajectory);
        Robot.driveSubsystem.loadMotionProfile(trajectory);
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
