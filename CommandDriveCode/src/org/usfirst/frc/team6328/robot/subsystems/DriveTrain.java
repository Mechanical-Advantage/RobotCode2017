package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private CANTalon rightTalonMaster = new CANTalon(RobotMap.rightMaster);
	private CANTalon rightTalonSlave = new CANTalon(RobotMap.rightSlave);
	private CANTalon leftTalonMaster = new CANTalon(RobotMap.leftMaster);
	private CANTalon leftTalonSlave = new CANTalon(RobotMap.leftSlave);
	private FeedbackDevice encoderType = FeedbackDevice.QuadEncoder;
	private int ticksPerRotation = 1440; // getEncPosition values in one turn
	private double wheelDiameter = 6; // inches
	
	public DriveTrain() {
		rightTalonMaster.setFeedbackDevice(encoderType);
		rightTalonMaster.reverseSensor(true); // right side is reversed
		rightTalonMaster.reverseOutput(true);
		rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		rightTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftTalonMaster.setFeedbackDevice(encoderType);
		leftTalonMaster.reverseSensor(false);
		leftTalonMaster.reverseOutput(false);
		leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		leftTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		setupVelocityClosedLoop(2,0,40,1.07); // PID tuning P,I,D,F
		rightTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightTalonSlave.set(RobotMap.rightMaster);
		leftTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftTalonSlave.set(RobotMap.leftMaster);
		enableBrakeMode(true);
	}
	
	private void setupVelocityClosedLoop(double p, double i, double d, double f) {
		rightTalonMaster.changeControlMode(TalonControlMode.Speed);
		leftTalonMaster.changeControlMode(TalonControlMode.Speed);
		//rightTalonMaster.setProfile(0);
		rightTalonMaster.setF(f);
		rightTalonMaster.setP(p);
		rightTalonMaster.setI(i);
		rightTalonMaster.setD(d);
		//leftTalonMaster.setProfile(0);
		leftTalonMaster.setF(f);
		leftTalonMaster.setP(p);
		leftTalonMaster.setI(i);
		leftTalonMaster.setD(d);
		resetPosition();
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    public void drive(int right, int left) {
    	drive((double)right, (double)left);
    }
    
    private double calcActualVelocity(double input) {
    	if (input>=-0.1 && input<=0.1) {
    		return 0;
    	}
    	else if (input>0.1 && input<RobotMap.minVelocity) {
    		return RobotMap.minVelocity;
    	}
    	else if (input<-0.1 && input>RobotMap.minVelocity*-1) {
    		return RobotMap.minVelocity*-1;
    	}
    	else {
    		return input;
    	}
    }
    
    public void drive(double right, double left) {
    	enable();
    	rightTalonMaster.set(calcActualVelocity(right));
    	leftTalonMaster.set(calcActualVelocity(left));
    	//System.out.println("Distance: Right: " + getDistanceRight() + " Left: " + getDistanceLeft());
    }
    
    public void stop() {
    	rightTalonMaster.set(0);
    	leftTalonMaster.set(0);
    }
    
    public void enable() {
    	rightTalonMaster.enable();
    	leftTalonMaster.enable();
    	rightTalonSlave.set(RobotMap.rightMaster); // reset slave master, talons lose their master when disabled
    	leftTalonMaster.set(RobotMap.leftMaster);
    }
    
    public void disable() {
    	rightTalonMaster.disable();
    	leftTalonMaster.disable();
    }
    
    public void enableBrakeMode(boolean enable) {
    	rightTalonMaster.enableBrakeMode(enable);
		leftTalonMaster.enableBrakeMode(enable);
		rightTalonSlave.enableBrakeMode(enable);
		leftTalonSlave.enableBrakeMode(enable);
    }
    
    public void resetPosition() {
    	rightTalonMaster.setEncPosition(0);
		leftTalonMaster.setEncPosition(0);
    }
    
    // values are cast to doubles to prevent integer division
    public double getRotationsLeft() {
    	return (double)leftTalonMaster.getEncPosition()/(double)ticksPerRotation;
    }
    
    public double getRotationsRight() {
    	return (double)rightTalonMaster.getEncPosition()/(double)ticksPerRotation*-1; // even though sensor is reversed on talon, getEncPosition returns real value so needs to be reversed here
    }
    
    // diameter times pi equals circumference times rotations equals distance
    public double getDistanceRight() {
    	return wheelDiameter*Math.PI*getRotationsRight();
    }
    
    public double getDistanceLeft() {
    	return wheelDiameter*Math.PI*getRotationsLeft();
    }
}

