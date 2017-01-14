package org.usfirst.frc.team6666.robot;

//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a short sample program demonstrating how to use the basic throttle
 * mode of the new CAN Talon.
 */
public class Robot extends SampleRobot {
	RobotDrive myRobot; // class that handles basic drive operations
	CANTalon left1;
	CANTalon left2;
	CANTalon right1;
	CANTalon right2;
	Joystick controller; // set to ID 0 in DriverStation
	//Joystick rightStick; // set to ID 2 in DriverStation
	

	public Robot() {

		left1 = new CANTalon(1); // Initialize the CanTalonSRX on device 1.
		left2 = new CANTalon(2);
		right1 = new CANTalon(3);
		right2 = new CANTalon(4);
		left1.changeControlMode(TalonControlMode.Speed);
		left1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		left1.reverseSensor(false);
		left1.configNominalOutputVoltage(+0.0f, -0.0f);
		left1.configPeakOutputVoltage(+12.0f, -12.0f);
		left1.setProfile(0);
		left1.setF(1);
		left1.setP(1);
		left1.setI(0);
		left1.setD(0);
		right1.changeControlMode(TalonControlMode.Speed);
		right1.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		right1.reverseSensor(false);
		right1.configNominalOutputVoltage(+0.0f, -0.0f);
		right1.configPeakOutputVoltage(+12.0f, -12.0f);
		right1.setProfile(0);
		right1.setF(1);
		right1.setP(1);
		right1.setI(0);
		right1.setD(0);
		left2.changeControlMode(CANTalon.TalonControlMode.Follower);
		left2.set(1);
		right2.changeControlMode(CANTalon.TalonControlMode.Follower);
		right2.set(3);

		myRobot = new RobotDrive(left1, left2, right1, right2);
		controller = new Joystick(0);
	}

	/**
	 * Runs the motor.
	 */
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			// Set the motor's output to half power.
			// This takes a number from -1 (100% speed in reverse) to +1 (100%
			// speed
			
			//left1.set(Math.pow(controller.getRawAxis(3), 3));
			//left2.set(Math.pow(controller.getRawAxis(3), 3));
			//right1.set(-1 * Math.pow(controller.getRawAxis(1), 3));
			//right2.set(-1 * Math.pow(controller.getRawAxis(1), 3));
			int velocity = left1.getEncVelocity();
			left1.set(controller.getRawAxis(3)*1500);
			right1.set(controller.getRawAxis(1)*1500);
			SmartDashboard.putNumber("Velocity", velocity );
			
			//System.out.println("PLEASE PRINT");
			//System.out.println(controller.getY());
			
			//left2.set(0.5);
			//right1.set(-0.5);
			//right2.set(-0.5);

			Timer.delay(0.01); // Note that the CANTalon only receives updates
								// every
								// 10ms, so updating more quickly would not gain
								// you
								// anything.
			
			//System.out.println(controller.getX());

		}
		left1.disable();
		left2.disable();
		right1.disable();
		right2.disable();
		
	}
}
