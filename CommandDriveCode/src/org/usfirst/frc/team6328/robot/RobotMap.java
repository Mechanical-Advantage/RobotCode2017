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
	public static int leftMaster;
	public static int leftSlave;
	public static int minVelocity = 40; // lower values will be treated as this value
	public static int maxVelocity = 950; // maximum velocity when sticks are fully forward (value of 1)
	public boolean PracticeRobot= true;
	public RobotMap(){
		if(PracticeRobot == true){
				rightMaster = 1;
				rightSlave = 2;
				leftMaster= 3;
				leftSlave = 4;
		}
	}
	
	
	//RobotType robot = RobotType.PRACTICE;

	
	
	
}

