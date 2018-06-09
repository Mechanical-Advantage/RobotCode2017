package org.usfirst.frc.team6328.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
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
import org.usfirst.frc.team6328.robot.triggers.POVTrigger;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class LogitechOI {
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
	
	private Joystick logitechController = new Joystick(0);
	
	private Button intakeOn = new JoystickButton(logitechController, 1);
	private Button intakeOff = new JoystickButton(logitechController, 2);
	private Button shoot = new JoystickButton(logitechController, 6);
	private Button topGearOpen = new JoystickButton(logitechController, 3);
	private Button topGearClose = new JoystickButton(logitechController, 4);
	private Button expelGear = new JoystickButton(logitechController, 5);
	private Button smartGearExpel = new POVTrigger(logitechController, 270);
	private Button straightAssist = new JoystickButton(logitechController, 9);
	private Button joysticksForward = new POVTrigger(logitechController, 0);
	private Button joysticksBackward = new POVTrigger(logitechController, 180);
	private Button sniperMode = new JoystickButton(logitechController, 10);
	private Button shakeBalls = new JoystickButton(logitechController, 8);
	private Button ejectBalls = new JoystickButton(logitechController, 7);
	
	Command runShooter;
	NetworkTable table;

	public LogitechOI() {
		table = NetworkTable.getTable("LEDs");
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
		shoot.whileHeld(runShooter);
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
		// Now mapped to shoot button, this is for switch
		/*if (!RobotMap.practiceRobot) {
			// there is no WhileNotHeld, so start when inactive and cancel when switch flipped on
			// if flipped at start, run
			shooterDisableSwitch.whenReleased(runShooter);
			shooterDisableSwitch.cancelWhenPressed(runShooter);
		}*/
		
//		openLoopDrive.whenPressed(new ApplyOpenLoopSwitch(false));
//		openLoopDrive.whenReleased(new ApplyOpenLoopSwitch(true));
		straightAssist.whileHeld(new DriveWithJoystickOnHeading());
		joysticksForward.whenPressed(new ReverseJoysticks(false));
		joysticksBackward.whenPressed(new ReverseJoysticks(true));
		joysticksForward.whenPressed(new SetCamera(true));
		joysticksBackward.whenPressed(new SetCamera(false));
	}
	
	public double getLeftAxis() {
		if (joysticksReversed) {
			return logitechController.getRawAxis(5)*-1;
		} else {
			return logitechController.getRawAxis(1);
		}
	}
	public double getRightAxis() {
		if (joysticksReversed) {
			return logitechController.getRawAxis(1)*-1;
		} else {
			return logitechController.getRawAxis(5);
		}
	}
	
	// reversing the joysticks should not change which joystick to use for straight drive, use
	// different function to make that correct
	// Note: Brian is left-handed
	public double getSingleDriveAxis() {
		if (joysticksReversed) {
			return logitechController.getRawAxis(1)*-1;
		} else {
			return logitechController.getRawAxis(1);
		}
	}
	public double getHorizDriveAxis() {
		return logitechController.getRawAxis(4);
	}

	public double getClimbAxis() {
		double climbUpTrigger = logitechController.getRawAxis(3);
		double climbDownTrigger = logitechController.getRawAxis(2);
		if (climbUpTrigger > 0) {
			return climbUpTrigger;
		} else {
			return climbDownTrigger*-1;
		}
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

	
	public void reverseJoysticks(boolean reverse) {
		joysticksReversed = reverse;
	}
	
	public boolean getSmartGearExpel() {
		return !smartGearExpel.get();
	}
	
	public void initSwitches() {
		
	}
	
	public boolean getDriveEnabled() {
		return true;
	}
	
	public boolean getOpenLoop() {
		return false;
	}
	
	public double getSniperLevel() {
		return 0.25;
	}
	
	public double getShooterSpeed() {
		return 1;
	}

	
	
	public void updateLED(OILED led, boolean state) {
		boolean[] array = table.getBooleanArray("OI LEDs", new  boolean[]{false, false, false, false, false, false, false, false});
		array[led.ordinal()] = state;
		table.putBooleanArray("OI LEDs", array);
	}
	
	public enum OILED {
		TOP_GEAR_OPEN, TOP_GEAR_CLOSE, INTAKE_ON, INTAKE_OFF, SHOOT_BUTTON, AUTOCLIMB, AUTOHOLD, MISC_LED
	}
}