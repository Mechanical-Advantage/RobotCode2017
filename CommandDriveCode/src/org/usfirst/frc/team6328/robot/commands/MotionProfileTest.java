package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Motion profile test command
 */
public class MotionProfileTest extends Command {
	//TODO Make nice command with new profiling system, switches between robots
	
	final double PositionErrorThreshold = 1;
	final double AngleErrorThreshold = 1;
	
	CustomDistanceFollower leftFollower, rightFollower;
	boolean positionEndReached;

    public MotionProfileTest() {
    	super("MotionProfileTest");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	Waypoint[] points = new Waypoint[] {
//        		new Waypoint(0, 0, 0),
//        		new Waypoint(91.52/*+1+2*/, -57.26/*+9+3*/, Pathfinder.d2r(-60)),
//        		new Waypoint(100.02, -71.98, Pathfinder.d2r(-60)),
//        };
    	Waypoint[] points = new Waypoint[] {
    			new Waypoint(0, 0, 0),
//    			new Waypoint(60, 0, 0),
//    			new Waypoint(108, -50, Pathfinder.d2r(-60)),
    			new Waypoint(120, 60, Pathfinder.d2r(90))
    	};
//    	Waypoint[] points = new Waypoint[] {
//        		new Waypoint(0, 0, 0),
//        		new Waypoint(100, -40, Pathfinder.d2r(-60)),
//        };
    	/*Waypoint[] points = new Waypoint[] {
    			new Waypoint(0, 0, 0),
    			new Waypoint(24, 0, Pathfinder.d2r(5)),
    	};*/
//    	points = new Waypoint[] {
//    			new Waypoint(0, 0, 0),
//    			new Waypoint(96.02, 77.26, Pathfinder.d2r(60))
//    	};
        
    	// practice robot
//        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, /*124.4*/62.2, /*62.2*/40, 1866);
//    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 60, 15, 1866);
    	final Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
    			Trajectory.Config.SAMPLES_HIGH, 0.02, /*124.4*/60, /*62.2,41.4*/55, /*1866*/2700);
    		Trajectory trajectory = Pathfinder.generate(points, config);
//        for (int i = 0; i < trajectory.length(); i++) {
//            Trajectory.Segment seg = trajectory.get(i);
//
//            System.out.printf(i+1 + ": %f,%f,%f,%f,%f,%f,%f,%f\n", 
//                seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
//                    seg.acceleration, seg.jerk, seg.heading);
//        }
        File csvFile = new File("/tmp/trajectory.csv");
        Pathfinder.writeToCSV(csvFile, trajectory);
//        Trajectory trajectory = Pathfinder.readFromCSV(csvFile);
        /*System.out.println("Trajectory generated, loading");
        Robot.driveSubsystem.loadMotionProfile(trajectory);
        System.out.println("Trajectory loaded, starting");
        Robot.driveSubsystem.startMotionProfile();
        System.out.println("Trajectory started");*/
    		TankModifier modifier = new TankModifier(trajectory);
    		// This likely won't be the real wheelbase, tune with gyro correction disabled
    		modifier.modify(21);
    		leftFollower = new CustomDistanceFollower(modifier.getRightTrajectory());
    		rightFollower = new CustomDistanceFollower(modifier.getLeftTrajectory());
    		positionEndReached = false;
    		// Note: without velocity scaling, 1 P is very little, 1*maxVel is really 1 as in doc, I think
    		leftFollower.configurePIDVA(12, 0, 0, 1, 0); // Perhaps don't scale velocity to percentage, pass directly to drivetrain
    		rightFollower.configurePIDVA(12, 0, 0, 1, 0);
    		Robot.driveSubsystem.resetPosition();
    		Robot.ahrs.zeroYaw();
    		// TODO Add a check to determine when gyro reset completes
    		try {
    			Thread.sleep(500);
    		}
    		catch (InterruptedException e) {
    			
    		}
    }
    /*
     * Tuning Notes:
     * High P to keep up with trajectory
     * Tune wheelbase with gyro correction disabled until heading is about accurate
     * D should be significantly lower than P
     * Too much D will overcorrect, robot will go too fast, not much is needed, in my initial test, any causes problems
     * In final version, P/D should maybe be multiplied by maxVelocity of profile so they are percent
     */

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double l = leftFollower.calculate(Robot.driveSubsystem.getDistanceLeft());
    	double r = rightFollower.calculate(Robot.driveSubsystem.getDistanceRight());

    	double gyro_heading = Robot.ahrs.getYaw();    // Assuming the gyro is giving a value in degrees
    	double desired_heading = Pathfinder.r2d(leftFollower.getHeading());  // Should also be in degrees

    	double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    	double turn = 0.8*60 * (-1.0/80.0) * angleDifference; // since no velocity scaling, multiple gyro P (0.8) by maxVelocity
//    	turn = 0;
    	
    	if (leftFollower.isFinished()) {
    		// set velocity to 0 so last profile velocity is not still applied, in case last velocity is not zero
    		leftFollower.configurePIDVA(5, 0, 0, 0, 0);
    		rightFollower.configurePIDVA(5, 0, 0, 0, 0);
    	}
    	
    	// system so that is yaw is off, correct for that without oscillation of position control
    	if (!positionEndReached && leftFollower.isFinished() && leftFollower.getLastError()<=PositionErrorThreshold &&
    				rightFollower.getLastError()<=PositionErrorThreshold) {
    		positionEndReached = true;
    	}

    	try {
//    		System.out.printf("DLeft: %f, DRight: %f, Yaw: %f, Target Yaw: %f, SLeft: %f, SRight: %f, Target DLeft: %f, Target DRight: %f, "
//    				+ "Profile SLeft: %f, Profile SRight: %f\n",
//    				Robot.driveSubsystem.getDistanceLeft(), Robot.driveSubsystem.getDistanceRight(), Robot.ahrs.getYaw(),
//    				Pathfinder.r2d(leftFollower.getHeading()), l, r, leftFollower.getSegment().position, rightFollower.getSegment().position, 
//    				leftFollower.getSegment().velocity, rightFollower.getSegment().velocity);
    	}
    	catch (ArrayIndexOutOfBoundsException e) {
    		// do nothing
    	}
    	if (!positionEndReached) {
    		Robot.driveSubsystem.drive((r + turn), (l - turn)); // velocity scaling removed
    	} else {
//    		System.out.println("Yaw Adjust");
    		Robot.driveSubsystem.drive(turn, turn*-1);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//        return Robot.driveSubsystem.isMotionProfileComplete();
    		System.out.printf("%b, %b, %b, %b", leftFollower.isFinished(), leftFollower.getLastError()<=PositionErrorThreshold,
    				rightFollower.getLastError()<=PositionErrorThreshold,
    				Math.abs(Robot.ahrs.getYaw()-Pathfinder.r2d(leftFollower.getHeading()))<=AngleErrorThreshold);
    		// current segment and heading should be same on left and right, only check one
    		return leftFollower.isFinished() && leftFollower.getLastError()<=PositionErrorThreshold &&
    				rightFollower.getLastError()<=PositionErrorThreshold &&
    				Math.abs(Robot.ahrs.getYaw()-Pathfinder.r2d(leftFollower.getHeading()))<=AngleErrorThreshold;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("MotionProfileTest end() called");
//    	Robot.driveSubsystem.stopMotionProfile();
    	Robot.driveSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("MotionProfileTest interrupted");
    	end();
    }
    
    // TODO make this not just be copy, extending DistanceFollower not possible because vars are private
    // based on DistanceFollower, ads getLastError, will track last point instead of returning 0
    private class CustomDistanceFollower {
        private double kp, ki, kd, kv, ka;

        private double last_error, heading;

        private int segment;
        private Trajectory trajectory;

        public CustomDistanceFollower(Trajectory traj) {
            this.trajectory = traj;
        }

        public CustomDistanceFollower() { }

        /**
         * Set a new trajectory to follow, and reset the cumulative errors and segment counts
         */
        public void setTrajectory(Trajectory traj) {
            this.trajectory = traj;
            reset();
        }

        /**
         * Configure the PID/VA Variables for the Follower
         * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
         * @param ki The integral term. Currently unused.
         * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
         * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
         *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
         *           motor controllers
         * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
         */
        public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
            this.kp = kp; this.ki = ki; this.kd = kd;
            this.kv = kv; this.ka = ka;
        }

        /**
         * Reset the follower to start again. Encoders must be reconfigured.
         */
        public void reset() {
            last_error = 0; segment = 0;
        }

        /**
         * Calculate the desired output for the motors, based on the distance the robot has covered.
         * This does not account for heading of the robot. To account for heading, add some extra terms in your control
         * loop for realignment based on gyroscope input and the desired heading given by this object.
         * @param distance_covered  The distance covered in meters
         * @return                  The desired output for your motor controller
         */
        // Modified to continue tracking last point at end of trajectory
        public double calculate(double distance_covered) {
        	Trajectory.Segment seg = trajectory.get(segment);
        	double error = seg.position - distance_covered;
        	double calculated_value =
        			kp * error +                                    // Proportional
        			kd * ((error - last_error) / seg.dt) +          // Derivative
        			(kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        	last_error = error;
        	heading = seg.heading;
        	if (segment < trajectory.length()-1) {
        		segment++;
        	}

        	return calculated_value;
        }

        /**
         * @return the desired heading of the current point in the trajectory
         */
        public double getHeading() {
            return heading;
        }

        /**
         * @return the current segment being operated on
         */
        public Trajectory.Segment getSegment() {
            return trajectory.get(segment);
        }

        /**
         * @return the position error last time calculate was called 
         */
        public double getLastError() {
			return last_error;
		}

		/**
         * @return whether we have finished tracking this trajectory or not.
         */
        // modified to work with my modified calculate that will not advance segment beyond end (added -1)
        public boolean isFinished() {
            return segment >= trajectory.length()-1;
        }
    }
}
