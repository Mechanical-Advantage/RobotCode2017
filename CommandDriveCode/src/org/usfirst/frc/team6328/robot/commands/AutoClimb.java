package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Attempts to automatically climb the rope
 */
public class AutoClimb extends CommandGroup {
	
	private final double wrapSpeed = 0.6;
	private final double wrapCurrent = 10;
	private final double climbSpeed = 1;
	private final double climbCurrent = 48;

    public AutoClimb() {
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
    	addSequential(new AutoClimbRunMotor(wrapSpeed, wrapCurrent, false));
    	addSequential(new AutoClimbRunMotor(climbSpeed, climbCurrent, true));
    }
    
    private class AutoClimbRunMotor extends Command {
    	
    	private double motorSpeed;
    	private double finalCurrent;
    	private double startTime;
    	private boolean stopMotor;

        public AutoClimbRunMotor(double speed, double finishedCurrent, boolean stopWhenDone) {
            // Use requires() here to declare subsystem dependencies
            // eg. requires(chassis);
        	requires(Robot.climberSubsystem);
        	motorSpeed = speed;
        	finalCurrent = finishedCurrent;
        	stopMotor = stopWhenDone;
        }

        // Called just before this Command runs the first time
        protected void initialize() {
        	Robot.climberSubsystem.run(motorSpeed);
        	startTime = Timer.getFPGATimestamp();
        }

        // Called repeatedly when this Command is scheduled to run
        protected void execute() {
        	if (RobotMap.tuningMode) {
        		SmartDashboard.putNumber("Voltage Graph ClimbWithJoystick", Robot.climberSubsystem.getVoltage());
        		SmartDashboard.putNumber("Current Graph ClimbWithJoystick", Robot.climberSubsystem.getCurrent());
        	}
        }

        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished() {
            return Robot.climberSubsystem.getCurrent()>=finalCurrent && Timer.getFPGATimestamp()>=startTime+0.5;
        }

        // Called once after isFinished returns true
        protected void end() {
        	if (stopMotor) {
        		Robot.climberSubsystem.stop();
        	}
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted() {
        }
    }
}
