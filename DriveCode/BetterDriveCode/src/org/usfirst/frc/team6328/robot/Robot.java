/*
 * 
 *   ______ _____   _____  __ ___  ____   ___  
	|  ____|  __ \ / ____|/ /|__ \|___ \ / _ \ 
	| |__  | |__) | |    / /_   ) | __) | (_) |
	|  __| |  _  /| |   | '_ \ / / |__ < > _ < 
	| |    | | \ \| |___| (_) / /_ ___) | (_) |
	|_|    |_|  \_\\_____\___/____|____/ \___/ 
                                                   
 * 
 * 
 * 
 */

package org.usfirst.frc.team6328.robot;

//import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.CANTalon;
import com.ctre.CANTalon.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.StringJoiner;

/**
 * This is a short program demonstrating how to use the basic velocity closed loop throttle
 * mode of the new CAN Talon.
 */
public class Robot extends SampleRobot {
	RobotDrive myRobot; // class that handles basic drive operations
	boolean practiceRobot = false;
	CANTalon right1;
	CANTalon right2;
	CANTalon left1;
	CANTalon left2;
	CANTalon right3;
	CANTalon left3;
	Joystick controller; // set to ID 0 in DriverStation
	//Joystick leftStick; // set to ID 2 in DriverStation
	

	public Robot() { 
		// setupRobotDrive();
		// setupSmartDashboard
		if (!practiceRobot) {
			right1 = new CANTalon(14); // Initialize the CanTalonSRX on device 1.
			//right1.setSafetyEnabled(false);
			right2 = new CANTalon(12);
			//right2.setSafetyEnabled(false);
			left1 = new CANTalon(15);
			left2 = new CANTalon(0);
			right3 = new CANTalon(13);
			left3 = new CANTalon(1);
		} else {
			right1 = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
			//right1.setSafetyEnabled(false);
			right2 = new CANTalon(2);
			//right2.setSafetyEnabled(false);
			left1 = new CANTalon(3);
			left2 = new CANTalon(4);
		}
		right1.changeControlMode(TalonControlMode.Speed);
		if (!practiceRobot) {
			right1.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			right1.reverseSensor(true);
		} else {
			right1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			right1.reverseSensor(false);
		}
		right1.configNominalOutputVoltage(+0.0f, -0.0f);
		right1.configPeakOutputVoltage(+12.0f, -12.0f);
		//right1.setProfile(0);
		right1.setF(1.07);
		right1.setP(2);
		right1.setI(0);
		right1.setD(40);
		right1.setVoltageRampRate(0);
		left1.changeControlMode(TalonControlMode.Speed);
		if (!practiceRobot) {
			left1.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			left1.reverseSensor(true);
		} else {
			left1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			left1.reverseSensor(false);
		}
		left1.configNominalOutputVoltage(+0.0f, -0.0f);
		left1.configPeakOutputVoltage(+12.0f, -12.0f);
		//left1.setProfile(0);
		left1.setF(1.07);
		left1.setP(2);
		left1.setI(0);
		left1.setD(40);
		left1.setVoltageRampRate(0);
		if (!practiceRobot) {
			right2.changeControlMode(CANTalon.TalonControlMode.Follower);
			right2.set(14);
			left2.changeControlMode(CANTalon.TalonControlMode.Follower);
			left2.set(15);
			right3.changeControlMode(CANTalon.TalonControlMode.Follower);
			right3.set(14);
			left3.changeControlMode(CANTalon.TalonControlMode.Follower);
			left3.set(15);
		} else {
			right2.changeControlMode(CANTalon.TalonControlMode.Follower);
			right2.set(1);
			left2.changeControlMode(CANTalon.TalonControlMode.Follower);
			left2.set(3);
		}

		//myRobot = new RobotDrive(right1, right2, left1, left2);
		controller = new Joystick(0);
		
		CameraServer.getInstance().startAutomaticCapture(0);
	}

	private String genGraphStr(double...data) {
		StringJoiner sj = new StringJoiner(":");
		for (double item : data) {
			sj.add(String.valueOf(item));
		}
		return sj.toString();
	}
	/*
	 * TODO Try and make a new class for operating the code ex. RobotDriver
	 * 	 
	 */
	public void operatorControl() {
		/*left1.enable();
		left2.enable();
		right1.enable();
		right2.enable();*/
		while (isOperatorControl() && isEnabled()) {
			
			//right1.set(Math.pow(controller.getRawAxis(3), 3));
			//right2.set(Math.pow(controller.getRawAxis(3), 3));
			//left1.set(-1 * Math.pow(controller.getRawAxis(1), 3));
			//left2.set(-1 * Math.pow(controller.getRawAxis(1), 3));
			double velocity = right1.getEncVelocity();
			//double targetvelocity = controller.getRawAxis(3)*-1500; // uncomment to use controller
			//right1.set(controller.getRawAxis(3)*1500); // and this
			//left1.set(controller.getRawAxis(1)*-1500); // and this
			SmartDashboard.putNumber("Velocity", velocity );
			SmartDashboard.putString("GraphData", genGraphStr((double)velocity,SmartDashboard.getNumber("PID/setpoint", 0)*6.8266, right1.getClosedLoopError())); // PID tuning mode
			//SmartDashboard.putString("GraphData", genGraphStr((double)velocity,targetvelocity)); // controller mode
			SmartDashboard.putString("VoltageCurrentGraph", genGraphStr(right1.getOutputVoltage()*-1, right1.getOutputCurrent()));
			SmartDashboard.putNumber("Output Voltage", right1.getOutputVoltage());
			SmartDashboard.putNumber("Output Current", right1.getOutputCurrent());
			//SmartDashboard.putNumber("Target Velocity", targetvelocity); // and this
			
			// this block is for PID tuning
			left1.setP(SmartDashboard.getNumber("PID/p", 2));
			left1.setI(SmartDashboard.getNumber("PID/i", 0));
			left1.setD(SmartDashboard.getNumber("PID/d", 40));
			left1.setF(SmartDashboard.getNumber("PID/f", 1.07));
			right1.setP(SmartDashboard.getNumber("PID/p", 2));
			right1.setI(SmartDashboard.getNumber("PID/i", 0));
			right1.setD(SmartDashboard.getNumber("PID/d", 40));
			right1.setF(SmartDashboard.getNumber("PID/f", 1.07));
			if (!SmartDashboard.getBoolean("PID/disabled", false)) { // this makes red (false) on the dashboard mean disabled (and makes it default), the dashboard wirtes true on green
				left1.disable();
				right1.disable();
				left2.disable();
				right2.disable();
				if (!practiceRobot) {
					left3.disable();
					right3.disable();
				}
			}
			else {
				left1.enable();
				right1.enable();
				left2.enable();
				right2.enable();
				if (!practiceRobot) {
					left3.enable();
					right3.enable();
				}
				if (practiceRobot) {
					right2.set(1); // talons lose their setting but NOT mode when disabled
					left2.set(3);
				} else {
					right2.set(14);
					right3.set(14);
					left2.set(15);
					left3.set(15);
				}
				
				/*left1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
				right1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
				left1.set(1);
				right1.set(-1);*/
				left1.set(SmartDashboard.getNumber("PID/setpoint", 0));
				right1.set(SmartDashboard.getNumber("PID/setpoint", 0)*-1); // only multiply by -1 to drive in a straight line (sides are inverted). Remove top *-1
				//right1.set(1);
				//right2.set(1);
			}
			
			//System.out.println("PLEASE PRINT");
			//System.out.println(controller.getY());
			
			//right2.set(0.5);
			//left1.set(-0.5);
			//left2.set(-0.5);

			Timer.delay(0.01); // Note that the CANTalon only receives updates
								// every
								// 10ms, so updating more quickly would not gain
								// you
								// anything.
			
			//System.out.println(controller.getX());

		}
		right1.disable();
		right2.disable();
		left1.disable();
		left2.disable();
		
	}
}
