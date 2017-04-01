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
	public static int shooterSensor;
	public static int topGearSolenoid1;
	public static int topGearSolenoid2;
	public static int gearExpellerSolenoid1;
	public static int gearExpellerSolenoid2;
	public static int gearExpellerSensor1;
	public static int gearExpellerSensor2;
	public static int minVelocity; // lower values will be treated as this value
	public static int maxVelocity; // maximum velocity when sticks are fully forward (value of 1)
	public static final boolean practiceRobot = false;
	public static final boolean tuningMode = false;
	public static ShooterControlType shooterControlType;
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
			shooterSensor = 9; // DIO
			topGearSolenoid1 = 2;
			topGearSolenoid2 = 3;
			gearExpellerSolenoid1 = 0;
			gearExpellerSolenoid2 = 1;
			gearExpellerSensor1 = 7; // DIO
			gearExpellerSensor2 = 8; // DIO
			maxVelocity = 525;
			minVelocity = 20;
			shooterControlType = ShooterControlType.FAST_BANG_BANG;
		}
	}

	public enum ShooterControlType {
		SIMPLE_BANG_BANG, // bang-bang control in the 20ms execute period
		FAST_BANG_BANG, // bang-bang in a separate thread with a definable speed
		PID // PID wheel control in the 20ms execute period with adjustable PID thread speed
	}
}

