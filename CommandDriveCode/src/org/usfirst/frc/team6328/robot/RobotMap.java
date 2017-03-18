package org.usfirst.frc.team6328.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static int leftMotor = 1;
    // public static int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
/*	public enum RobotType{
		PRACTICE(0), COMPETITION(1);
		private int type;
		private RobotType(int a){
			this.type=a;
		}
		public int getValue(){
			return type;			
		}
	}*/
	public static int rightMaster;
	public static int rightSlave;
	public static int rightSlave2;
	public static int leftMaster;
	public static int leftSlave;
	public static int leftSlave2;
	public static int intake;
	public static int loader;
	public static int trigger;
	public static int climber;
	public static int shooterMaster;
	public static int shooterSlave;
	public static int gearSolenoid1;
	public static int gearSolenoid2;
	public static int minVelocity; // lower values will be treated as this value
	public static int maxVelocity; // maximum velocity when sticks are fully forward (value of 1)
	public static final boolean practiceRobot = false;
	public static final boolean tuningMode = false;
	public RobotMap(){
		if (practiceRobot) {
				rightMaster = 1;
				rightSlave = 2;
				leftMaster = 3;
				leftSlave = 4;
				maxVelocity = 396; // 950 native units per 100ms
				minVelocity = 17; // 40 native units per 100ms
		}
		else {
			rightMaster = 14;
			rightSlave = 13;
			rightSlave2 = 12;
			leftMaster = 15;
			leftSlave = 0;
			leftSlave2 = 1;
			intake = 5;
			loader = 4;
			trigger = 9;
			climber = 2;
			shooterMaster = 10;
			shooterSlave = 11;
			gearSolenoid1 = 0;
			gearSolenoid2 = 1;
			maxVelocity = 525;
			minVelocity = 20;
		}
	}
	
	
	//RobotType robot = RobotType.PRACTICE;

	
	
	
}

