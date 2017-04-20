package org.usfirst.frc.team6328.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team6328.robot.commands.AutoClimb;
import org.usfirst.frc.team6328.robot.commands.BackUpFromBoiler;
import org.usfirst.frc.team6328.robot.commands.CancelCommand;
import org.usfirst.frc.team6328.robot.commands.ClimberHold;
import org.usfirst.frc.team6328.robot.commands.CloseTopGear;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystickOnHeading;
import org.usfirst.frc.team6328.robot.commands.ExpelGearOnSensor;
import org.usfirst.frc.team6328.robot.commands.ApplyOpenLoopSwitch;
import org.usfirst.frc.team6328.robot.commands.OpenTopGear;
import org.usfirst.frc.team6328.robot.commands.ReverseJoysticks;
import org.usfirst.frc.team6328.robot.commands.RunIntake;
import org.usfirst.frc.team6328.robot.commands.RunIntakeShoot;
import org.usfirst.frc.team6328.robot.commands.RunLoader;
import org.usfirst.frc.team6328.robot.commands.RunShooterFastBangBang;
import org.usfirst.frc.team6328.robot.commands.RunShooterPID;
import org.usfirst.frc.team6328.robot.commands.RunShooterSimpleBangBang;
import org.usfirst.frc.team6328.robot.commands.RunTrigger;
import org.usfirst.frc.team6328.robot.commands.SetCamera;
import org.usfirst.frc.team6328.robot.commands.ShakeLoader;
import org.usfirst.frc.team6328.robot.commands.TurnToAngle;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);
    
    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.
    
    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:
    
    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());
    
    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());
    
    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());
	
	private boolean joysticksReversed = false;
	
	// map left stick to ID 0 and right to ID 1 in driver station
	private Joystick leftController = new Joystick(0);
	private Joystick rightController = new Joystick(1);
	private Joystick oiController1 = new Joystick(2);
	private Joystick oiController2 = new Joystick(3);
	
	private Button right90Button = new JoystickButton(leftController, 5);
	private Button left90Button = new JoystickButton(leftController, 4);
	private Button right45Button = new JoystickButton(rightController, 5);
	private Button left45Button = new JoystickButton(rightController, 4);
	private Button frontCameraButton = new JoystickButton(rightController, 3);
	private Button rearCameraButton = new JoystickButton(rightController, 2);
	private Button backupFromBoiler = new JoystickButton(leftController, 6);
	private Button intakeOn = new JoystickButton(oiController2, 2);
	private Button intakeOff = new JoystickButton(oiController2, 3);
	private Button shoot = new JoystickButton(oiController2, 1);
	private Button topGearOpen = new JoystickButton(oiController2, 5);
	private Button topGearClose = new JoystickButton(oiController2, 6);
	private Button expelGear = new JoystickButton(oiController2, 4);
	private Button shooterDisableSwitch = new JoystickButton(oiController1, 7);
	private Button driveDisableSwitch = new JoystickButton(oiController1, 9);
//	private Button openLoopShooter = new JoystickButton(oiController1, 8);
	private Button smartGearExpel = new JoystickButton(oiController1, 8);
	private Button autoClimb = new JoystickButton(oiController2, 7);
	private Button climberHold = new JoystickButton(oiController2, 8);
	private Button openLoopDrive = new JoystickButton(oiController1, 10);
	private Button straightAssist = new JoystickButton(leftController, 1);
	private Button joysticksForward = new JoystickButton(leftController, 3);
	private Button joysticksBackward = new JoystickButton(leftController, 2);
	private Button sniperMode = new JoystickButton(rightController, 1);
	private Button shakeBalls = new JoystickButton(oiController2, 9);
	private Button ejectBalls = new JoystickButton(oiController2, 10);
	
	private Button loaderForward = new JoystickButton(oiController1, 3);
	private Button loaderBackward = new JoystickButton(oiController1, 4);
	private Button intakeForward = new JoystickButton(oiController1, 1);
	private Button intakeBackward = new JoystickButton(oiController1, 2);
	private Button triggerForward = new JoystickButton(oiController1, 5);
	private Button triggerBackward = new JoystickButton(oiController1, 6);
	
	Command runShooter;

	public OI() {
		if (!RobotMap.practiceRobot) {
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
		}
		
		// turn only while the button is held
		// note: can't just use whileHeld because that will repeatedly run the command
		TurnToAngle right90 = new TurnToAngle(90);
		TurnToAngle left90 = new TurnToAngle(-90);
		TurnToAngle right45 = new TurnToAngle(45);
		TurnToAngle left45 = new TurnToAngle(-45);
		right90Button.whenPressed(right90);
		right90Button.whenReleased(new CancelCommand(right90));
		left90Button.whenPressed(left90);
		left90Button.whenReleased(new CancelCommand(left90));
		right45Button.whenPressed(right45);
		right45Button.whenReleased(new CancelCommand(right45));
		left45Button.whenPressed(left45);
		left45Button.whenReleased(new CancelCommand(left45));
		
		frontCameraButton.whenPressed(new SetCamera(true));
		rearCameraButton.whenPressed(new SetCamera(false));
		
		RunIntake runIntakeNormal = new RunIntake(false);
		RunIntakeShoot runIntakeShoot = new RunIntakeShoot();
		RunLoader runLoader = new RunLoader(false);
		RunTrigger runTriggerShoot = new RunTrigger(false);
		//RunTrigger runTriggerNormal = new RunTrigger(true);
		//RunClimberForIntake runClimberIntake = new RunClimberForIntake();
		//intakeOn.whenPressed(runClimberIntake);
		//intakeOn.whenPressed(runTriggerNormal);
		//intakeOn.whenPressed(runLoader);
		intakeOn.whenPressed(runIntakeNormal);
		//intakeOff.cancelWhenPressed(runClimberIntake);
		//intakeOff.cancelWhenPressed(runTriggerNormal);
		//intakeOff.cancelWhenPressed(runTriggerShoot);
		//intakeOff.cancelWhenPressed(runLoader);
		intakeOff.cancelWhenPressed(runIntakeNormal);
		shoot.whileHeld(runTriggerShoot);
		shoot.whileHeld(runLoader);
		shoot.whileHeld(runIntakeShoot);
		shoot.whenReleased(runIntakeNormal);
		//shoot.whenReleased(runTriggerNormal);
		//shoot.whenReleased(runLoader);
		shakeBalls.whileHeld(new ShakeLoader());
		ejectBalls.whileHeld(new RunIntake(true));
		ejectBalls.whileHeld(new RunLoader(true));
		ejectBalls.whileHeld(new RunTrigger(true));
		topGearOpen.whenPressed(new OpenTopGear());
		topGearClose.whenPressed(new CloseTopGear());
		expelGear.whileHeld(new ExpelGearOnSensor());
		
		// trying to enable practice robot will not work
		if (!RobotMap.practiceRobot) {
			// there is no WhileNotHeld, so start when inactive and cancel when switch flipped on
			// if flipped at start, run
			shooterDisableSwitch.whenReleased(runShooter);
			shooterDisableSwitch.cancelWhenPressed(runShooter);
		}
		
		autoClimb.toggleWhenPressed(new AutoClimb());
		climberHold.toggleWhenPressed(new ClimberHold());
		openLoopDrive.whenPressed(new ApplyOpenLoopSwitch(false));
		openLoopDrive.whenReleased(new ApplyOpenLoopSwitch(true));
		straightAssist.whileHeld(new DriveWithJoystickOnHeading());
		joysticksForward.whenPressed(new ReverseJoysticks(false));
		joysticksBackward.whenPressed(new ReverseJoysticks(true));
		joysticksForward.whenPressed(new SetCamera(true));
		joysticksBackward.whenPressed(new SetCamera(false));
		BackUpFromBoiler backupFromBoilerCommand = new BackUpFromBoiler();
		backupFromBoiler.whenPressed(backupFromBoilerCommand);
		backupFromBoiler.whenReleased(new CancelCommand(backupFromBoilerCommand));
		
		loaderForward.whileHeld(new RunLoader(false));
		loaderBackward.whileHeld(new RunLoader(true));
		intakeForward.whileHeld(new RunIntake(false));
		intakeBackward.whileHeld(new RunIntake(true));
		triggerForward.whileHeld(new RunTrigger(false));
		triggerBackward.whileHeld(new RunTrigger(true));
	}
	
	public double getLeftAxis() {
		if (joysticksReversed) {
			return rightController.getRawAxis(1)*-1;
		} else {
			return leftController.getRawAxis(1);
		}
	}
	public double getRightAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1)*-1;
		} else {
			return rightController.getRawAxis(1);
		}
	}
	
	// reversing the joysticks should not change which joystick to use for straight drive, use
	// different function to make that correct
	// Note: Brian is left-handed
	public double getDriveStraightAxis() {
		if (joysticksReversed) {
			return leftController.getRawAxis(1)*-1;
		} else {
			return leftController.getRawAxis(1);
		}
	}

	public double getClimbAxis() {
		return oiController1.getRawAxis(1);
	}
	
	public boolean getOpenLoop() {
		return openLoopDrive.get();
	}
	
	public boolean getDriveEnabled() {
		return !driveDisableSwitch.get();
	}
	
	public boolean getOpenLoopShooter() {
		// the switch for this was replaced with gear expel override, so use
		// a switch on the dashboard
//		return openLoopShooter.get();
		return SmartDashboard.getBoolean("Open Loop Shooter", false);
	}
	
	public boolean getSniperMode() {
		return sniperMode.get();
	}
	
	public double getSniperLevel() {
		double sniperLimit = 0.5;
		return (1-((rightController.getRawAxis(2)+1)/2))*sniperLimit; // control returns -1 to 1, scale to 0 to 1, subtract from 1 so 1 is up
	}
	
	public double getShooterSpeed() {
		return (1-((leftController.getRawAxis(2)+1)/2));
	}
	
	public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
	}
	
	public boolean getSmartGearExpel() {
		return !smartGearExpel.get();
	}
	
	@SuppressWarnings("unused")
	public void initSwitches() {
		if (!shooterDisableSwitch.get() && !RobotMap.practiceRobot) {
			runShooter.start();
		}
		if (openLoopDrive.get()) {
			Robot.driveSubsystem.useOpenLoop();
		} else {
			Robot.driveSubsystem.useClosedLoop();
		}
	}
}