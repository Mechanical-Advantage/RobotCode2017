package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Bang-bang shooter control using a thread for higher speed
 */
public class RunShooterFastBangBang extends Command {
	
	private final double controllerPeriod = 0.005;
	private final int targetSpeed = 85; // 83 is 5000 rpm
	
	private BangBangController bangBangController = new BangBangController();
	private Notifier bangBangControllerNotifier = new Notifier(bangBangController);
	private boolean reachedSpeed = false;

    public RunShooterFastBangBang() {
    	super("RunShooterFastBangBang");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	bangBangController.enable();
    	bangBangControllerNotifier.startPeriodic(controllerPeriod);
    	reachedSpeed = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	int finalTargetSpeed = RobotMap.tuningMode ? (int)SmartDashboard.getNumber("Shooter Target RPS", targetSpeed) : targetSpeed;
    	if (!reachedSpeed && Robot.shooterSubsystem.getSpeed() >= finalTargetSpeed) {
			Robot.oi.updateLED(OILED.SHOOT_BUTTON, true);
			reachedSpeed = true;
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	// Notifier.stop does not work reliably, so set it to single shot first so it will stop even if stop does not work
    	// also have a disable system on the thread to kill it immediately and just make sure it does not run the motor
    	 // written on WPILib 2017.3.1
    	bangBangController.disable();
    	bangBangControllerNotifier.startSingle(0);
    	bangBangControllerNotifier.stop();
    	Robot.shooterSubsystem.stop();
    	Robot.oi.updateLED(OILED.SHOOT_BUTTON, false);
    }
    
    private class BangBangController implements Runnable {
    	
    	private boolean enabled = false;

    	@Override
    	public void run() {
    		if (enabled) {
    			double speed = Robot.shooterSubsystem.getSpeed();
    			// if in tuning mode, get target from dashboard if it exists
    			int finalTargetSpeed = RobotMap.tuningMode ? (int)SmartDashboard.getNumber("Shooter Target RPS", targetSpeed) : targetSpeed;
    			if (!Robot.oi.getOpenLoopShooter()) {
    				if (speed>finalTargetSpeed) {
    					Robot.shooterSubsystem.stop();
    				} else {
    					Robot.shooterSubsystem.run();
    				}
    				if (RobotMap.tuningMode) {
    					SmartDashboard.putNumber("Shooter Speed", speed);
    				}
    			} else if (Robot.oi.getOpenLoopShooter()) {
    				Robot.shooterSubsystem.runOpenLoop();
    				if (RobotMap.tuningMode) {
    					SmartDashboard.putNumber("Shooter Speed", speed);
    				}
    			}
    		} else {
    			System.out.println("Thread called but not enabled");
    		}
    	}
		
		public void disable() {
			enabled = false;
			Robot.shooterSubsystem.stop();
		}
		
		public void enable() {
			enabled = true;
		}
    	
    }
}
