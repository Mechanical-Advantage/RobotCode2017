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
	public static int minVelocity = 40; // lower values will be treated as this value
	public static int maxVelocity = 950; // maximum velocity when sticks are fully forward (value of 1)
	public static final boolean practiceRobot = true;
	public static final boolean tuningMode = false;
	public RobotMap(){
		if (practiceRobot) {
				rightMaster = 1;
				rightSlave = 2;
				leftMaster= 3;
				leftSlave = 4;
		}
		else {
			rightMaster = 1;
			rightSlave = 2;
			rightSlave2 = 5;
			leftMaster = 3;
			leftSlave = 4;
			leftSlave2 = 6;
			intake = 0;
			loader = 0;
			trigger = 0;
			climber = 0;
			shooterMaster = 0;
			shooterSlave = 0;
			gearSolenoid1 = 0;
			gearSolenoid2 = 1;
		}
	}
	
	
	//RobotType robot = RobotType.PRACTICE;

	
	
	
}

