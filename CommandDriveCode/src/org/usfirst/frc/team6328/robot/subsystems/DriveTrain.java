package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

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
	
	public DriveTrain() {
		rightTalonMaster.setFeedbackDevice(encoderType);
		rightTalonMaster.reverseSensor(false);
		rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		rightTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftTalonMaster.setFeedbackDevice(encoderType);
		leftTalonMaster.reverseSensor(false);
		leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
		leftTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		setupVelocityClosedLoop(1,0,0,1);
		rightTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightTalonSlave.set(RobotMap.rightMaster);
		leftTalonSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftTalonSlave.set(RobotMap.leftMaster);
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
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

