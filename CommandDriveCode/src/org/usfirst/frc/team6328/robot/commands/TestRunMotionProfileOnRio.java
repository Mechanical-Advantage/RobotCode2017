package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * Tests RunMotionProfileOnRio
 */
public class TestRunMotionProfileOnRio extends CommandGroup {

    public TestRunMotionProfileOnRio(boolean mp) {
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
    		if (mp) {
    			System.out.println("Generating...");
    			Waypoint[] points = new Waypoint[] {
    	    			new Waypoint(0, 0, 0),
    	    			new Waypoint(120, 60, Pathfinder.d2r(90))};
    			final Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
    	    			Trajectory.Config.SAMPLES_HIGH, 0.02, 100, 55, 2700);
    	    		Trajectory trajectory = Pathfinder.generate(points, config);
    	    		File file = new File("/tmp/trajectory.csv");
    	    		Pathfinder.writeToCSV(file, trajectory);
    	    		addSequential(new RunMotionProfileOnRio(trajectory));
    	    		System.out.println("Generation complete");
    		} else {
    			addSequential(new DriveDistanceOnHeading(120));
    			addSequential(new TurnAndDriveDistance(60, 90));
    		}
    }
}
