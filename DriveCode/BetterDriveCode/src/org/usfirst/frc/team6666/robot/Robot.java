package org.usfirst.frc.team6666.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

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
			
			left1.set(Math.pow(controller.getRawAxis(5), 3));
			left2.set(Math.pow(controller.getRawAxis(5), 3));
			right1.set(-1 * Math.pow(controller.getRawAxis(1), 3));
			right2.set(-1 * Math.pow(controller.getRawAxis(1), 3));

			
			System.out.println("PLEASE PRINT");
			System.out.println(controller.getY());
			
			//left2.set(0.5);
			//right1.set(-0.5);
			//right2.set(-0.5);

			Timer.delay(0.01); // Note that the CANTalon only receives updates
								// every
								// 10ms, so updating more quickly would not gain
								// you
								// anything.
			
			System.out.println(controller.getX());

		}
		left1.disable();
		left2.disable();
		right1.disable();
		right2.disable();
	}
}
