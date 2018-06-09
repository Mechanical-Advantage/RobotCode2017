package org.usfirst.frc.team6328.robot.CANTalon;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.GadgeteerUartClient;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDInterface;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/*
 *  Based on CANTalon sandbox from CTRE github.
 *  MP support would require class definitions of all the data objects to work and modifications to the pushMotionProfileTrajectoryPoint function to accept old TrajectoryPoints
 */

public class CANTalon
		implements MotorSafety, PIDOutput, PIDSource, SpeedController, PIDInterface, Sendable, GadgeteerUartClient {
	private MotorSafetyHelper m_safetyHelper;
	protected PIDSourceType m_pidSource = PIDSourceType.kDisplacement;

	private com.ctre.phoenix.motorcontrol.Faults _faults = new com.ctre.phoenix.motorcontrol.Faults();
	private com.ctre.phoenix.motorcontrol.StickyFaults _stickyFaults = new com.ctre.phoenix.motorcontrol.StickyFaults();

	private int _setTimeoutMs = 0;
	private int _getTimeoutMs = 4;

	private TalonSRX _srx;

	private int _deviceNumber;

	private boolean _inverted = false;
	private boolean _reverseSensor = false;
	private boolean _reverseOutput = false;

	/** ensures we've set the saturation voltage for vcomp */
	private boolean _hasSetSaturationVoltage = false;

	private com.ctre.phoenix.motion.TrajectoryPoint _trajPt = new com.ctre.phoenix.motion.TrajectoryPoint();
	
	/**
	 * Number of adc engineering units per 0 to 3.3V sweep. This is necessary
	 * for scaling Analog Position in rotations/RPM.
	 */
	private static final int kNativeAdcUnitsPerRotation = 1024;
	/**
	 * Number of pulse width engineering units per full rotation. This is
	 * necessary for scaling Pulse Width Decoded Position in rotations/RPM.
	 */
	private static final double kNativePwdUnitsPerRotation = 4096.0;
	/**
	 * Number of minutes per 100ms unit. Useful for scaling velocities measured
	 * by Talon's 100ms time base to rotations per minute.
	 */
	private static final double kMinutesPer100msUnit = 1.0 / 600.0;

	NeutralMode _neutralMode = NeutralMode.EEPROMSetting;

	public enum TalonControlMode {
		PercentVbus(0), Position(1), Speed(2), Current(3), Voltage(4), Follower(5), MotionProfile(6), MotionMagic(
				7), Disabled(15);

		public final int value;

		public static TalonControlMode valueOf(int value) {
			for (TalonControlMode mode : values()) {
				if (mode.value == value) {
					return mode;
				}
			}

			return null;
		}

		TalonControlMode(int value) {
			this.value = value;
		}

		public boolean isPID() {
			return this == Current || this == Speed || this == Position;
		}

		public int getValue() {
			return value;
		}
	}

	public enum FeedbackDevice {
		QuadEncoder(0), AnalogPot(2), AnalogEncoder(3), EncRising(4), EncFalling(5), CtreMagEncoder_Relative(
				6), CtreMagEncoder_Absolute(7), PulseWidth(8);

		public int value;

		public static FeedbackDevice valueOf(int value) {
			for (FeedbackDevice mode : values()) {
				if (mode.value == value) {
					return mode;
				}
			}

			return null;
		}

		FeedbackDevice(int value) {
			this.value = value;
		}
	}

	/**
	 * Depending on the sensor type, Talon can determine if sensor is plugged in
	 * ot not.
	 */
	public enum FeedbackDeviceStatus {
		FeedbackStatusUnknown(0), FeedbackStatusPresent(1), FeedbackStatusNotPresent(2);

		public int value;

		public static FeedbackDeviceStatus valueOf(int value) {
			for (FeedbackDeviceStatus mode : values()) {
				if (mode.value == value) {
					return mode;
				}
			}
			return null;
		}

		FeedbackDeviceStatus(int value) {
			this.value = value;
		}
	}

	/**
	 * Enumerated types for frame rate ms.
	 */
	public enum StatusFrameRate {
		General(0), Feedback(1), QuadEncoder(2), AnalogTempVbat(3), PulseWidth(4);

		public int value;

		public static StatusFrameRate valueOf(int value) {
			for (StatusFrameRate mode : values()) {
				if (mode.value == value) {
					return mode;
				}
			}
			return null;
		}

		StatusFrameRate(int value) {
			this.value = value;
		}
	}

	void EnsureSaturationVoltageIsSet() {
		if (_hasSetSaturationVoltage == false) {
			/* use nonzero timeout to ensure this gets set properly */
			if (_srx.configVoltageCompSaturation(15.0, 10) == ErrorCode.OK) {
				_hasSetSaturationVoltage = true;
			}
		} else {
			/* already set to 15V */
		}
	}


//	/**
//	 * Motion Profile Trajectory Point This is simply a data transer object.
//	 */
//	@SuppressWarnings("MemberName")
//	public static class TrajectoryPoint {
//		public double position; // !< The position to servo to.
//		public double velocity; // !< The velocity to feed-forward.
//		/**
//		 * Time in milliseconds to process this point. Value should be between
//		 * 1ms and 255ms. If value is zero then Talon will default to 1ms. If
//		 * value exceeds 255ms API will cap it.
//		 */
//		public int timeDurMs;
//		/**
//		 * Which slot to get PIDF gains. PID is used for position servo. F is
//		 * used as the Kv constant for velocity feed-forward. Typically this is
//		 * hardcoded to the a particular slot, but you are free gain schedule if
//		 * need be.
//		 */
//		public int profileSlotSelect;
//		/**
//		 * Set to true to only perform the velocity feed-forward and not perform
//		 * position servo. This is useful when learning how the position servo
//		 * changes the motor response. The same could be accomplish by clearing
//		 * the PID gains, however this is synchronous the streaming, and doesn't
//		 * require restoing gains when finished.
//		 *
//		 * <p>
//		 * Additionaly setting this basically gives you direct control of the
//		 * motor output since motor output = targetVelocity X Kv, where Kv is
//		 * our Fgain. This means you can also scheduling straight-throttle
//		 * curves without relying on a sensor.
//		 */
//		public boolean velocityOnly;
//		/**
//		 * Set to true to signal Talon that this is the final point, so do not
//		 * attempt to pop another trajectory point from out of the Talon buffer.
//		 * Instead continue processing this way point. Typically the velocity
//		 * member variable should be zero so that the motor doesn't spin
//		 * indefinitely.
//		 */
//		public boolean isLastPoint;
//		/**
//		 * Set to true to signal Talon to zero the selected sensor. When
//		 * generating MPs, one simple method is to make the first target
//		 * position zero, and the final target position the target distance from
//		 * the current position. Then when you fire the MP, the current position
//		 * gets set to zero. If this is the intent, you can set zeroPos on the
//		 * first trajectory point.
//		 *
//		 * <p>
//		 * Otherwise you can leave this false for all points, and offset the
//		 * positions of all trajectory points so they are correct.
//		 */
//		public boolean zeroPos;
//	}

//	/**
//	 * Motion Profile Status This is simply a data transer object.
//	 */
//	@SuppressWarnings("MemberName")
//	public static class MotionProfileStatus {
//		/**
//		 * The available empty slots in the trajectory buffer.
//		 *
//		 * <p>
//		 * The robot API holds a "top buffer" of trajectory points, so your
//		 * applicaion can dump several points at once. The API will then stream
//		 * them into the Talon's low-level buffer, allowing the Talon to act on
//		 * them.
//		 */
//		public int topBufferRem;
//		/**
//		 * The number of points in the top trajectory buffer.
//		 */
//		public int topBufferCnt;
//		/**
//		 * The number of points in the low level Talon buffer.
//		 */
//		public int btmBufferCnt;
//		/**
//		 * Set if isUnderrun ever gets set. Only is cleared by
//		 * clearMotionProfileHasUnderrun() to ensure robot logic can react or
//		 * instrument it.
//		 *
//		 * @see clearMotionProfileHasUnderrun()
//		 */
//		public boolean hasUnderrun;
//		/**
//		 * This is set if Talon needs to shift a point from its buffer into the
//		 * active trajectory point however the buffer is empty. This gets
//		 * cleared automatically when is resolved.
//		 */
//		public boolean isUnderrun;
//		/**
//		 * True if the active trajectory point has not empty, false otherwise.
//		 * The members in activePoint are only valid if this signal is set.
//		 */
//		public boolean activePointValid;
//		/**
//		 * The number of points in the low level Talon buffer.
//		 */
//		public TrajectoryPoint activePoint = new TrajectoryPoint();
//		/**
//		 * The current output mode of the motion profile executer (disabled,
//		 * enabled, or hold). When changing the set() value in MP mode, it's
//		 * important to check this signal to confirm the change takes effect
//		 * before interacting with the top buffer.
//		 */
//		public SetValueMotionProfile outputEnable;
//	}

	private TalonControlMode m_controlMode = TalonControlMode.PercentVbus;

	private boolean m_controlEnabled;
	private boolean m_stopped = false;
	private int m_profile;

	double m_setPoint;
	/**
	 * Encoder CPR, counts per rotations, also called codes per revoluion.
	 * Default value of zero means the API behaves as it did during the 2015
	 * season, each position unit is a single pulse and there are four pulses
	 * per count (4X). Caller can use configEncoderCodesPerRev to set the
	 * quadrature encoder CPR.
	 */
	int m_codesPerRev;
	/**
	 * Number of turns per rotation. For example, a 10-turn pot spins ten full
	 * rotations from a wiper voltage of zero to 3.3 volts. Therefore knowing
	 * the number of turns a full voltage sweep represents is necessary for
	 * calculating rotations and velocity. A default value of zero means the API
	 * behaves as it did during the 2015 season, there are 1024 position units
	 * from zero to 3.3V.
	 */
	int m_numPotTurns;
	/**
	 * Although the Talon handles feedback selection, caching the feedback
	 * selection is helpful at the API level for scaling into rotations and RPM.
	 */
	FeedbackDevice m_feedbackDevice;

	/**
	 * Constructor for the CANTalon device.
	 *
	 * @param deviceNumber
	 *            The CAN ID of the Talon SRX
	 */
	public CANTalon(int deviceNumber) {
		_deviceNumber = deviceNumber;
		_srx = new TalonSRX(deviceNumber);
	}

	/**
	 * Constructor for the CANTalon device.
	 *
	 * @param deviceNumber
	 *            The CAN ID of the Talon SRX
	 * @param controlPeriodMs
	 *            The period in ms to send the CAN control frame. Period is
	 *            bounded to [1ms,95ms].
	 */
	public CANTalon(int deviceNumber, int controlPeriodMs) {
		_deviceNumber = deviceNumber;
		_srx = new TalonSRX(deviceNumber);
		_srx.setControlFramePeriod(ControlFrame.Control_3_General, controlPeriodMs);
	}

	/**
	 * Constructor for the CANTalon device.
	 *
	 * @param deviceNumber
	 *            The CAN ID of the Talon SRX
	 * @param controlPeriodMs
	 *            The period in ms to send the CAN control frame. Period is
	 *            bounded to [1ms,95ms].
	 * @param enablePeriodMs
	 *            The period in ms to send the enable control frame. Period is
	 *            bounded to [1ms,95ms]. This typically is not required to
	 *            specify, however this could be used to minimize the time
	 *            between robot-enable and talon-motor-drive.
	 */
	public CANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs) {

		_deviceNumber = deviceNumber;
		_srx = new TalonSRX(deviceNumber);
		_srx.setControlFramePeriod(ControlFrame.Control_3_General, controlPeriodMs);
		/* enable frame is gone */
	}

	@Override
	public void pidWrite(double output) {
		if (getControlMode() == TalonControlMode.PercentVbus) {
			set(output);
		} else {
			throw new IllegalStateException("PID only supported in PercentVbus mode");
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}

	@Override
	public double pidGet() {
		return getPosition();
	}

	public void delete() {
		disable();
		/* unsupported */
	}

	/**
	 * Sets the appropriate output on the talon, depending on the mode.
	 *
	 * <p>
	 * In PercentVbus, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Follower mode, the output is the integer device ID of the talon to
	 * duplicate. In Voltage mode, outputValue is in volts. In Current mode,
	 * outputValue is in amperes. In Speed mode, outputValue is in position
	 * change / 10ms. In Position mode, outputValue is in encoder ticks or an
	 * analog value, depending on the sensor.
	 *
	 * @param outputValue
	 *            The setpoint value, as described above.
	 */
	public void set(double outputValue) {
		/* feed safety helper since caller just updated our output */
//		m_safetyHelper.feed(); // This never gets instantiated so disable it
		if (m_stopped) {
			enableControl();
			m_stopped = false;
		}
		if (m_controlEnabled) {
			m_setPoint = outputValue; /* cache set point for getSetpoint() */
			double scalar = _inverted ? -1.0 : +1.0;
			switch (m_controlMode) {
			case Current:
				_srx.set(ControlMode.Current, outputValue * scalar);
				break;
			case Disabled:
				_srx.set(ControlMode.Disabled, outputValue);
				break;
			case Follower:
				_srx.set(ControlMode.Follower, outputValue);
				break;
			case MotionMagic:
				_srx.set(ControlMode.MotionMagic, outputValue * scalar);
				break;
			case MotionProfile:
				_srx.set(ControlMode.MotionProfile, outputValue);
				break;
			case PercentVbus:
				_srx.set(ControlMode.PercentOutput, outputValue * scalar);
				break;
			case Position:
				_srx.set(ControlMode.Position, scaleRotationsToNativeUnits(m_feedbackDevice, outputValue) * scalar);
				break;
			case Speed:
				_srx.set(ControlMode.Velocity, scaleVelocityToNativeUnits(m_feedbackDevice, outputValue) * scalar);
				break;
			case Voltage:
				EnsureSaturationVoltageIsSet();
				_srx.set(ControlMode.PercentOutput, outputValue / 15.0 * scalar);
				break;
			}
		}
	}

	/**
	 * Inverts the direction of the motor's rotation. Only works in PercentVbus
	 * mode.
	 *
	 * @param isInverted
	 *            The state of inversion, true is inverted.
	 */
	@Override
	public void setInverted(boolean isInverted) {
		_inverted = isInverted;
		_srx.setInverted(false); /* leave M+ as positive, and M- for reverse */
	}

	/**
	 * Common interface for the inverting direction of a speed controller.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	@Override
	public boolean getInverted() {
		return _inverted;
	}

	/**
	 * Resets the accumulated integral error and disables the controller.
	 *
	 * <p>
	 * The only difference between this and {@link PIDController#reset} is that
	 * the PIDController also resets the previous error for the D term, but the
	 * difference should have minimal effect as it will only last one cycle.
	 */
	public void reset() {
		disable();
		clearIAccum();
	}

	/**
	 * Return true if Talon is enabled.
	 *
	 * @return true if the Talon is enabled and may be applying power to the
	 *         motor
	 */
	public boolean isEnabled() {
		return isControlEnabled();
	}

	/**
	 * Returns the difference between the setpoint and the current position.
	 *
	 * @return The error in units corresponding to whichever mode we are in.
	 * @see #set(double) set() for a detailed description of the various units.
	 */
	public double getError() {
		return getClosedLoopError();
	}

	/**
	 * Calls {@link #set(double)}.
	 */
	public void setSetpoint(double setpoint) {
		set(setpoint);
	}

	/**
	 * Flips the sign (multiplies by negative one) the sensor values going into
	 * the talon.
	 *
	 * <p>
	 * This only affects position and velocity closed loop control. Allows for
	 * situations where you may have a sensor flipped and going in the wrong
	 * direction.
	 *
	 * @param flip
	 *            True if sensor input should be flipped; False if not.
	 */
	public void reverseSensor(boolean flip) {
		_reverseSensor = flip;
		updateDirectionParams();
	}

	private void updateDirectionParams() {
		_srx.setInverted(_reverseOutput); /* flip the hbridge, this will not affect sensor phase */
		_srx.setSensorPhase(_reverseSensor); /* flip the sensor */
	}

	/**
	 * Flips the sign (multiplies by negative one) the throttle values going
	 * into the motor on the talon in closed loop modes.
	 *
	 * @param flip
	 *            True if motor output should be flipped; False if not.
	 */
	public void reverseOutput(boolean flip) {
		_reverseOutput = flip;
		updateDirectionParams();
	}

	/**
	 * Gets the current status of the Talon (usually a sensor value).
	 *
	 * <p>
	 * In Current mode: returns output current. In Speed mode: returns current
	 * speed. In Position mode: returns current sensor position. In PercentVbus
	 * and Follower modes: returns current applied throttle.
	 *
	 * @return The current sensor value of the Talon.
	 */
	public double get() {
		switch (m_controlMode) {
		case Voltage:
			return getOutputVoltage();
		case Current:
			return getOutputCurrent();
		case Speed:
			return scaleNativeUnitsToRpm(m_feedbackDevice, _srx.getSelectedSensorVelocity(0));
		case Position:
			return scaleNativeUnitsToRotations(m_feedbackDevice, _srx.getSelectedSensorPosition(0));
		case PercentVbus:
		default:
			return (double) _srx.getMotorOutputPercent();
		}
	}

	/**
	 * Get the current encoder position, regardless of whether it is the current
	 * feedback device.
	 *
	 * @return The current position of the encoder.
	 */
	public int getEncPosition() {
		return _srx.getSensorCollection().getQuadraturePosition();
	}

	public void setEncPosition(int newPosition) {
		_srx.getSensorCollection().setQuadraturePosition(newPosition, _setTimeoutMs);
	}

	/**
	 * Get the current encoder velocity, regardless of whether it is the current
	 * feedback device.
	 *
	 * @return The current speed of the encoder.
	 */
	public int getEncVelocity() {
		return _srx.getSensorCollection().getQuadratureVelocity();
	}

	public int getPulseWidthPosition() {
		return _srx.getSensorCollection().getPulseWidthPosition();
	}

	public void setPulseWidthPosition(int newPosition) {
		_srx.getSensorCollection().setPulseWidthPosition(newPosition, _setTimeoutMs);
	}

	public int getPulseWidthVelocity() {
		return _srx.getSensorCollection().getPulseWidthVelocity();
	}

	public int getPulseWidthRiseToFallUs() {
		return _srx.getSensorCollection().getPulseWidthRiseToFallUs();
	}

	public int getPulseWidthRiseToRiseUs() {
		return _srx.getSensorCollection().getPulseWidthRiseToRiseUs();
	}

	/**
	 * @param feedbackDevice
	 *            which feedback sensor to check it if is connected.
	 * @return status of caller's specified sensor type.
	 */
	public FeedbackDeviceStatus isSensorPresent(FeedbackDevice feedbackDevice) {
		FeedbackDeviceStatus retval = FeedbackDeviceStatus.FeedbackStatusUnknown;
		/* detecting sensor health depends on which sensor caller cares about */
		switch (feedbackDevice) {
		case QuadEncoder:
		case AnalogPot:
		case AnalogEncoder:
		case EncRising:
		case EncFalling:
			/*
			 * no real good way to tell if these sensor are actually present so
			 * return status unknown.
			 */
			break;
		case PulseWidth:
		case CtreMagEncoder_Relative:
		case CtreMagEncoder_Absolute:

			/* all of these require pulse width signal to be present. */
			if (_srx.getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
				/* Talon not getting a signal */
				retval = FeedbackDeviceStatus.FeedbackStatusNotPresent;
			} else {
				/* getting good signal */
				retval = FeedbackDeviceStatus.FeedbackStatusPresent;
			}
			break;
		default:
			break;
		}
		return retval;
	}

	/**
	 * Get the number of of rising edges seen on the index pin.
	 *
	 * @return number of rising edges on idx pin.
	 */
	public int getNumberOfQuadIdxRises() {
		return 0;
	}

	/**
	 * @return IO level of QUADA pin.
	 */
	public int getPinStateQuadA() {
		return _srx.getSensorCollection().getPinStateQuadA() ? 1 : 0;
	}

	/**
	 * @return IO level of QUADB pin.
	 */
	public int getPinStateQuadB() {
		return _srx.getSensorCollection().getPinStateQuadB() ? 1 : 0;
	}

	/**
	 * @return IO level of QUAD Index pin.
	 */
	public int getPinStateQuadIdx() {
		return _srx.getSensorCollection().getPinStateQuadIdx() ? 1 : 0;
	}

	public void setAnalogPosition(int newPosition) {
		_srx.getSensorCollection().setAnalogPosition(newPosition, _setTimeoutMs);
	}

	/**
	 * Get the current analog in position, regardless of whether it is the
	 * current feedback device.
	 *
	 * <p>
	 * The bottom ten bits is the ADC (0 - 1023) on the analog pin of the Talon.
	 * The upper 14 bits tracks the overflows and underflows (continuous
	 * sensor).
	 *
	 * @return The 24bit analog position.
	 */
	public int getAnalogInPosition() {
		return _srx.getSensorCollection().getAnalogIn();
	}

	/**
	 * Get the current analog in position, regardless of whether it is the
	 * current feedback device.
	 *
	 * @return The ADC (0 - 1023) on analog pin of the Talon.
	 */
	public int getAnalogInRaw() {
		return _srx.getSensorCollection().getAnalogInRaw();
	}

	/**
	 * Get the current encoder velocity, regardless of whether it is the current
	 * feedback device.
	 *
	 * @return The current speed of the analog in device.
	 */
	public int getAnalogInVelocity() {
		return _srx.getSensorCollection().getAnalogInVel();
	}

	/**
	 * Get the current difference between the setpoint and the sensor value.
	 *
	 * @return The error, in whatever units are appropriate.
	 */
	public int getClosedLoopError() {
		/* retrieve the closed loop error in native units */
		return _srx.getClosedLoopError(0);
	}

	/**
	 * Set the allowable closed loop error.
	 *
	 * @param allowableCloseLoopError
	 *            allowable closed loop error for selected profile. mA for
	 *            Curent closed loop. Talon Native Units for position and
	 *            velocity.
	 */
	public void setAllowableClosedLoopErr(int allowableCloseLoopError) {
		_srx.configAllowableClosedloopError(allowableCloseLoopError, 0, _setTimeoutMs);
	}

	// Returns true if limit switch is closed. false if open.
	public boolean isFwdLimitSwitchClosed() {
		return _srx.getSensorCollection().isFwdLimitSwitchClosed();
	}

	// Returns true if limit switch is closed. false if open.
	public boolean isRevLimitSwitchClosed() {
		return _srx.getSensorCollection().isRevLimitSwitchClosed();
	}

	// Returns true if Index is set to Zero Sensor Position
	public boolean isZeroSensorPosOnIndexEnabled() {
		return false;
	}

	// Returns true if Reverse Limit Switch is set to Zero Sensor Position
	public boolean isZeroSensorPosOnRevLimitEnabled() {
		return false;
	}

	// Returns true if Reverse Limit Switch is set to Zero Sensor Position
	public boolean isZeroSensorPosOnFwdLimitEnabled() {
		return false;
	}

	// Returns true if break is enabled during neutral. false if coast.
	public boolean getBrakeEnableDuringNeutral() {
		if (_neutralMode == NeutralMode.EEPROMSetting) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		}
		if (_neutralMode == NeutralMode.Brake)
			return true;
		return false;
	}

	/**
	 * Configure how many codes per revolution are generated by your encoder.
	 *
	 * @param codesPerRev
	 *            The number of counts per revolution.
	 */
	public void configEncoderCodesPerRev(int codesPerRev) {
		/*
		 * first save the scalar so that all getters/setter work as the user
		 * expects
		 */
		m_codesPerRev = codesPerRev;
	}

	/**
	 * Configure the number of turns on the potentiometer.
	 *
	 * @param turns
	 *            The number of turns of the potentiometer.
	 */
	public void configPotentiometerTurns(int turns) {
		/*
		 * first save the scalar so that all getters/setter work as the user
		 * expects
		 */
		m_numPotTurns = turns;
	}

	/**
	 * Returns temperature of Talon, in degrees Celsius.
	 */
	public double getTemperature() {
		return _srx.getTemperature();
	}

	/**
	 * Returns the current going through the Talon, in Amperes.
	 */
	public double getOutputCurrent() {
		return _srx.getOutputCurrent();
	}

	/**
	 * @return The voltage being output by the Talon, in Volts.
	 */
	public double getOutputVoltage() {
		return _srx.getMotorOutputVoltage();
	}

	/**
	 * @return The voltage at the battery terminals of the Talon, in Volts.
	 */
	public double getBusVoltage() {
		return _srx.getBusVoltage();
	}

	/**
	 * When using analog sensors, 0 units corresponds to 0V, 1023 units
	 * corresponds to 3.3V When using an analog encoder (wrapping around 1023 to
	 * 0 is possible) the units are still 3.3V per 1023 units. When using
	 * quadrature, each unit is a quadrature edge (4X) mode.
	 *
	 * @return The position of the sensor currently providing feedback.
	 */
	public double getPosition() {
		return scaleNativeUnitsToRotations(m_feedbackDevice, _srx.getSelectedSensorPosition(0));
	}

	public void setPosition(double pos) {
		int nativePos = scaleRotationsToNativeUnits(m_feedbackDevice, pos);
		_srx.setSelectedSensorPosition(nativePos, 0, _setTimeoutMs);
	}

	/**
	 * The speed units will be in the sensor's native ticks per 100ms.
	 *
	 * <p>
	 * For analog sensors, 3.3V corresponds to 1023 units. So a speed of 200
	 * equates to ~0.645 dV per 100ms or 6.451 dV per second. If this is an
	 * analog encoder, that likely means 1.9548 rotations per sec. For
	 * quadrature encoders, each unit corresponds a quadrature edge (4X). So a
	 * 250 count encoder will produce 1000 edge events per rotation. An example
	 * speed of 200 would then equate to 20% of a rotation per 100ms, or 10
	 * rotations per second.
	 *
	 * @return The speed of the sensor currently providing feedback.
	 */
	public double getSpeed() {
		return scaleNativeUnitsToRpm(m_feedbackDevice, _srx.getSelectedSensorVelocity(0));
	}

	public TalonControlMode getControlMode() {
		return m_controlMode;
	}

	public void setControlMode(int mode) {
		TalonControlMode tcm = TalonControlMode.valueOf(mode);
		if (tcm != null) {
			changeControlMode(tcm);
		}
	}

	/**
	 * Fixup the m_controlMode so set() serializes the correct demand value.
	 * Also fills the modeSelecet in the control frame to disabled.
	 *
	 * @param controlMode
	 *            Control mode to ultimately enter once user calls set().
	 * @see #set
	 */
	private void applyControlMode(TalonControlMode controlMode) {
		m_controlMode = controlMode;
		if (controlMode == TalonControlMode.Disabled) {
			m_controlEnabled = false;
		}
		// Disable until set() is called.
		_srx.set(ControlMode.Disabled, 0);

		HAL.report(tResourceType.kResourceType_CANTalonSRX, _deviceNumber + 1, controlMode.value);
	}

	public void changeControlMode(TalonControlMode controlMode) {
		if (m_controlMode == controlMode) {
			/* we already are in this mode, don't perform disable workaround */
		} else {
			applyControlMode(controlMode);
		}
	}

	public void setFeedbackDevice(FeedbackDevice device) {

		m_feedbackDevice = device;
		
		com.ctre.phoenix.motorcontrol.FeedbackDevice toApply = com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder;
		switch (device){
			case AnalogEncoder:
			case AnalogPot:
				toApply = com.ctre.phoenix.motorcontrol.FeedbackDevice.Analog;
				break;
			case CtreMagEncoder_Absolute:
			case PulseWidth:
				toApply = com.ctre.phoenix.motorcontrol.FeedbackDevice.PulseWidthEncodedPosition;
				break;
			case CtreMagEncoder_Relative:
			case EncFalling:
			case EncRising:
			case QuadEncoder:
				toApply = com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder;
				break;
		}
		_srx.configSelectedFeedbackSensor(toApply, 0, _setTimeoutMs);
	}

	public void setStatusFrameRateMs(StatusFrameRate stateFrame, int periodMs) {
		StatusFrameEnhanced talonFrame;
		switch(stateFrame) {
			case AnalogTempVbat:
				talonFrame = StatusFrameEnhanced.Status_4_AinTempVbat;
				break;
			case Feedback:
				talonFrame = StatusFrameEnhanced.Status_2_Feedback0;
				break;
			case General:
				talonFrame = StatusFrameEnhanced.Status_1_General;
				break;
			case PulseWidth:
				talonFrame = StatusFrameEnhanced.Status_8_PulseWidth;
				break;
			case QuadEncoder:
				talonFrame = StatusFrameEnhanced.Status_3_Quadrature;
				break;
			default:
				return;
		}
		_srx.setStatusFramePeriod(talonFrame, periodMs, _setTimeoutMs);
	}

	public void enableControl() {
		changeControlMode(m_controlMode);
		m_controlEnabled = true;
	}

	public void enable() {
		enableControl();
	}

	public void disableControl() {
		_srx.set(ControlMode.Disabled, 0);
		m_controlEnabled = false;
	}

	public boolean isControlEnabled() {
		return m_controlEnabled;
	}

	/**
	 * Get the current proportional constant.
	 *
	 * @return double proportional constant for current profile.
	 */
	public double getP() {
		return _srx.configGetParameter(ParamEnum.eProfileParamSlot_P, m_profile, _getTimeoutMs);
	}

	public double getI() {
		return _srx.configGetParameter(ParamEnum.eProfileParamSlot_I, m_profile, _getTimeoutMs);
	}

	public double getD() {
		return _srx.configGetParameter(ParamEnum.eProfileParamSlot_D, m_profile, _getTimeoutMs);
	}

	public double getF() {
		return _srx.configGetParameter(ParamEnum.eProfileParamSlot_F, m_profile, _getTimeoutMs);
	}

	public double getIZone() {
		return _srx.configGetParameter(ParamEnum.eProfileParamSlot_IZone, m_profile, _getTimeoutMs);
	}

	/**
	 * Get the closed loop ramp rate.
	 *
	 * <p>
	 * Limits the rate at which the throttle will change. Only affects position
	 * and speed closed loop modes.
	 *
	 * @return rampRate Maximum change in voltage, in volts / sec.
	 * @see #setProfile For selecting a certain profile.
	 */
	public double getCloseLoopRampRate() {
		double retval = _srx.configGetParameter(ParamEnum.eClosedloopRamp, 0, _getTimeoutMs);
		retval = retval / 1023.0 * 12.0 * 100.0;
		return retval;
	}

	/**
	 * Firmware version running on the Talon.
	 *
	 * @return The version of the firmware running on the Talon
	 */
	public long GetFirmwareVersion() {

		return _srx.getFirmwareVersion();
	}

	public long GetIaccum() {
		return (long)_srx.getIntegralAccumulator(0);
	}

	/**
	 * Set the proportional value of the currently selected profile.
	 *
	 * @param p
	 *            Proportional constant for the currently selected PID profile.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setP(double p) {
		_srx.config_kP(m_profile, p, _setTimeoutMs);
	}

	/**
	 * Set the integration constant of the currently selected profile.
	 *
	 * @param i
	 *            Integration constant for the currently selected PID profile.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setI(double i) {
		_srx.config_kI(m_profile, i, _setTimeoutMs);
	}

	/**
	 * Set the derivative constant of the currently selected profile.
	 *
	 * @param d
	 *            Derivative constant for the currently selected PID profile.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setD(double d) {
		_srx.config_kD(m_profile, d, _setTimeoutMs);
	}

	/**
	 * Set the feedforward value of the currently selected profile.
	 *
	 * @param f
	 *            Feedforward constant for the currently selected PID profile.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setF(double f) {
		_srx.config_kF(m_profile, f, _setTimeoutMs);
	}

	/**
	 * Set the integration zone of the current Closed Loop profile.
	 *
	 * <p>
	 * Whenever the error is larger than the izone value, the accumulated
	 * integration error is cleared so that high errors aren't racked up when at
	 * high errors. An izone value of 0 means no difference from a standard PIDF
	 * loop.
	 *
	 * @param izone
	 *            Width of the integration zone.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setIZone(int izone) {
		_srx.config_IntegralZone(m_profile, izone, _setTimeoutMs);
	}

	/**
	 * Set the closed loop ramp rate for the current profile.
	 *
	 * <p>
	 * Limits the rate at which the throttle will change. Only affects position
	 * and speed closed loop modes.
	 *
	 * @param rampRate
	 *            Maximum change in voltage, in volts / sec.
	 * @see #setProfile For selecting a certain profile.
	 */
	public void setCloseLoopRampRate(double rampRate) {
		if (rampRate == 0) {
			/* user wants to disable ramp */
			_srx.configClosedloopRamp(0, _setTimeoutMs);
		} else {
			double work;
			work = rampRate; /* throttle per 1ms */
			work /= 1023.0; /* percent per 1ms */
			work = 1.0 / work; /* ms per percent */
			work /= 1000.0; /* seconds per percent */
			work *= 100.0; /* sec from neutral to full */

			_srx.configClosedloopRamp(work, _setTimeoutMs);
		}
	}

	/**
	 * Set the voltage ramp rate for the current profile. This only affects open
	 * loop.
	 * <p>
	 * Limits the rate at which the throttle will change. Affects all modes.
	 *
	 * @param rampRate
	 *            Maximum change in voltage, in volts / sec.
	 */
	public void setVoltageRampRate(double rampRate) {
		if (rampRate == 0) {
			/* user wants to disable ramp */
			_srx.configClosedloopRamp(0, _setTimeoutMs);
		} else {
			double work;
			work = rampRate; /* volt per 1ms */
			work /= 12.0; /* percent per 1ms */
			work = 1.0 / work; /* ms per percent */
			work /= 1000.0; /* seconds per percent */
			work *= 100.0; /* sec from neutral to full */

			_srx.configOpenloopRamp(work, _setTimeoutMs);
		}
	}

	/**
	 * Same as setVoltageRampRate
	 * 
	 * @param rampRate
	 *            Maximum change in voltage, in volts / sec.
	 */
	public void setVoltageCompensationRampRate(double rampRate) {
		/* in prev season, this was effectively just another ramp rate */
		setVoltageRampRate(rampRate);
	}

	/**
	 * Clear the accumulator for I gain.
	 */
	public void ClearIaccum() {
		_srx.setIntegralAccumulator(0, 0, _setTimeoutMs);
	}

	/**
	 * Sets control values for closed loop control.
	 *
	 * @param p
	 *            Proportional constant.
	 * @param i
	 *            Integration constant.
	 * @param d
	 *            Differential constant.
	 * @param f
	 *            Feedforward constant.
	 * @param izone
	 *            Integration zone -- prevents accumulation of integration error
	 *            with large errors. Setting this to zero will ignore any izone
	 *            stuff.
	 * @param closeLoopRampRate
	 *            Closed loop ramp rate. Maximum change in voltage, in volts /
	 *            sec.
	 * @param profile
	 *            which profile to set the pid constants for. You can have two
	 *            profiles, with values of 0 or 1, allowing you to keep a second
	 *            set of values on hand in the talon. In order to switch
	 *            profiles without recalling setPID, you must call setProfile().
	 */
	public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		}
		m_profile = profile;
		setProfile(profile);
		setP(p);
		setI(i);
		setD(d);
		setF(f);
		setIZone(izone);
		setCloseLoopRampRate(closeLoopRampRate);
	}

	public void setPID(double p, double i, double d) {
		setPID(p, i, d, 0, 0, 0, m_profile);
	}

	/**
	 * @return The latest value set using set().
	 */
	public double getSetpoint() {
		return m_setPoint;
	}

	/**
	 * Select which closed loop profile to use, and uses whatever PIDF gains and
	 * the such that are already there.
	 */
	public void setProfile(int profile) {
		if (profile != 0 && profile != 1) {
			throw new IllegalArgumentException("Talon PID profile must be 0 or 1.");
		}
		m_profile = profile;
		_srx.selectProfileSlot(m_profile, 0);
	}

	/**
	 * Common interface for stopping a motor.
	 *
	 * @deprecated Use disableControl instead.
	 */
	@Override
	@Deprecated
	public void stopMotor() {
		disableControl();
		m_stopped = true;
	}

	public void disable() {
		disableControl();
	}

	public int getDeviceID() {
		return _deviceNumber;
	}

	public void clearIAccum() {
		_srx.setIntegralAccumulator(0, 0, _setTimeoutMs);
	}

	// ---------- soft limits ----------------------//
	public void setForwardSoftLimit(double forwardLimit) {
		int nativeLimitPos = scaleRotationsToNativeUnits(m_feedbackDevice, forwardLimit);
		_srx.configForwardSoftLimitThreshold(nativeLimitPos, _setTimeoutMs);
	}

	public int getForwardSoftLimit() {
		/* this appears to return raw sensors */
		return (int) _srx.configGetParameter(ParamEnum.eForwardSoftLimitThreshold, 0, _setTimeoutMs);
	}

	public void enableForwardSoftLimit(boolean enable) {
		_srx.configForwardSoftLimitEnable(enable, _setTimeoutMs);
	}

	public boolean isForwardSoftLimitEnabled() {
		double value = _srx.configGetParameter(ParamEnum.eForwardSoftLimitEnable, 0, _setTimeoutMs);
		return (value != 0);
	}

	public void setReverseSoftLimit(double reverseLimit) {
		int nativeLimitPos = scaleRotationsToNativeUnits(m_feedbackDevice, reverseLimit);
		_srx.configReverseSoftLimitThreshold(nativeLimitPos, _setTimeoutMs);
	}

	public int getReverseSoftLimit() {
		/* this appears to return raw sensors */
		return (int) _srx.configGetParameter(ParamEnum.eReverseSoftLimitThreshold, 0, _setTimeoutMs);
	}

	public void enableReverseSoftLimit(boolean enable) {
		_srx.configReverseSoftLimitEnable(enable, _setTimeoutMs);
	}

	public boolean isReverseSoftLimitEnabled() {
		double value = _srx.configGetParameter(ParamEnum.eReverseSoftLimitEnable, 0, _setTimeoutMs);
		return (value != 0);
	}

	// ---------- peak/nom ----------------------//
	/**
	 * Configure the maximum voltage that the Jaguar will ever output.
	 *
	 * <p>
	 * This can be used to limit the maximum output voltage in all modes so that
	 * motors which cannot withstand full bus voltage can be used safely.
	 *
	 * @param voltage
	 *            The maximum voltage output by the Jaguar.
	 */
	public void configMaxOutputVoltage(double voltage) {
		/* convert to percent */
		voltage /= 12.0;

		_srx.configPeakOutputForward(+voltage, _setTimeoutMs);
		_srx.configPeakOutputReverse(-voltage, _setTimeoutMs);
	}

	public void configPeakOutputVoltage(double forwardVoltage, double reverseVoltage) {
		/* bounds checking */
		if (forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if (forwardVoltage < 0) {
			forwardVoltage = 0;
		}
		if (reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if (reverseVoltage < -12) {
			reverseVoltage = -12;
		}
		/* config calls */
		_srx.configPeakOutputForward(forwardVoltage / 12.0, _setTimeoutMs);
		_srx.configPeakOutputReverse(reverseVoltage / 12.0, _setTimeoutMs);
	}

	public void configNominalOutputVoltage(double forwardVoltage, double reverseVoltage) {
		/* bounds checking */
		if (forwardVoltage > 12) {
			forwardVoltage = 12;
		} else if (forwardVoltage < 0) {
			forwardVoltage = 0;
		}
		if (reverseVoltage > 0) {
			reverseVoltage = 0;
		} else if (reverseVoltage < -12) {
			reverseVoltage = -12;
		}
		/* config calls */
		_srx.configNominalOutputForward(forwardVoltage / 12.0, _setTimeoutMs);
		_srx.configNominalOutputReverse(reverseVoltage / 12.0, _setTimeoutMs);
	}

	/**
	 * General set frame. Since the parameter is a general integral type, this
	 * can be used for testing future features.
	 */
	public void setParameter(ParamEnum paramEnum, double value) {
		_srx.configSetParameter(paramEnum.value, value, 0, 0, _setTimeoutMs);
	}

	/**
	 * General get frame. Since the parameter is a general integral type, this
	 * can be used for testing future features.
	 */
	public double getParameter(ParamEnum paramEnum) {

		double retval = _srx.configGetParameter(paramEnum.value, 0, _setTimeoutMs);
		return retval;
	}

	public void clearStickyFaults() {
		_srx.clearStickyFaults(_setTimeoutMs);
	}

	public void enableLimitSwitch(boolean forward, boolean reverse) {

		_srx.configSetParameter(ParamEnum.eForwardSoftLimitEnable.value, forward ? 1 : 0, 0x00, 0, _setTimeoutMs);
		_srx.configSetParameter(ParamEnum.eReverseSoftLimitEnable.value, reverse ? 1 : 0, 0x00, 0, _setTimeoutMs);
	}

	/**
	 * Configure the fwd limit switch to be normally open or normally closed.
	 * Talon will disable momentarilly if the Talon's current setting is
	 * dissimilar to the caller's requested setting.
	 *
	 * <p>
	 * Since Talon saves setting to flash this should only affect a given Talon
	 * initially during robot install.
	 *
	 * @param normallyOpen
	 *            true for normally open. false for normally closed.
	 */
	public void ConfigFwdLimitSwitchNormallyOpen(boolean normallyOpen) {
		int value;
		int subValue = 0;
		int ordinal = 0;
		// open=0, closed=1, disabled = 2
		if (normallyOpen)
			value = 0;
		else
			value = 1;
		_srx.configSetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, value, subValue, ordinal, _setTimeoutMs);
	}

	/**
	 * Configure the rev limit switch to be normally open or normally closed.
	 * Talon will disable momentarilly if the Talon's current setting is
	 * dissimilar to the caller's requested setting.
	 *
	 * <p>
	 * Since Talon saves setting to flash this should only affect a given Talon
	 * initially during robot install.
	 *
	 * @param normallyOpen
	 *            true for normally open. false for normally closed.
	 */
	public void ConfigRevLimitSwitchNormallyOpen(boolean normallyOpen) {
		int value;
		int subValue = 0;
		int ordinal = 1;
		// open=0, closed=1, disabled = 2
		if (normallyOpen)
			value = 0;
		else
			value = 1;
		_srx.configSetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, value, subValue, ordinal, _setTimeoutMs);
	}

	public void enableBrakeMode(boolean brake) {

		if (brake) {
			_neutralMode = NeutralMode.Brake;
		} else {
			_neutralMode = NeutralMode.Coast;
		}
		_srx.setNeutralMode(_neutralMode);
	}

	public int getFaultOverTemp() {
		return 0;
	}

	public int getFaultUnderVoltage() {
		_srx.getFaults(_faults);
		return _faults.UnderVoltage ? 1 : 0;
	}

	public int getFaultForLim() {
		_srx.getFaults(_faults);
		return _faults.ForwardLimitSwitch ? 1 : 0;
	}

	public int getFaultRevLim() {
		_srx.getFaults(_faults);
		return _faults.ReverseLimitSwitch ? 1 : 0;
	}

	public int getFaultHardwareFailure() {
		_srx.getFaults(_faults);
		return _faults.HardwareFailure ? 1 : 0;
	}

	public int getFaultForSoftLim() {
		_srx.getFaults(_faults);
		return _faults.ForwardSoftLimit ? 1 : 0;
	}

	public int getFaultRevSoftLim() {
		_srx.getFaults(_faults);
		return _faults.ReverseSoftLimit ? 1 : 0;
	}

	public int getStickyFaultOverTemp() {
		return 0;
	}

	public int getStickyFaultUnderVoltage() {
		_srx.getStickyFaults(_stickyFaults);
		return _stickyFaults.UnderVoltage ? 1 : 0;
	}

	public int getStickyFaultForLim() {
		_srx.getStickyFaults(_stickyFaults);
		return _stickyFaults.ForwardLimitSwitch ? 1 : 0;
	}

	public int getStickyFaultRevLim() {
		_srx.getStickyFaults(_stickyFaults);
		return _stickyFaults.ReverseLimitSwitch ? 1 : 0;
	}

	public int getStickyFaultForSoftLim() {
		_srx.getStickyFaults(_stickyFaults);
		return _stickyFaults.ForwardSoftLimit ? 1 : 0;
	}

	public int getStickyFaultRevSoftLim() {
		_srx.getStickyFaults(_stickyFaults);
		return _stickyFaults.ReverseSoftLimit ? 1 : 0;
	}

	/**
	 * Number of native units per rotation if scaling info is available. Zero if
	 * scaling information is not available.
	 *
	 * @return Number of native units per rotation.
	 */
	private double getNativeUnitsPerRotationScalar(FeedbackDevice devToLookup) {
		double retval = 0;
		boolean scalingAvail = false;
		switch (devToLookup) {
		case QuadEncoder: {
			/*
			 * When caller wants to lookup Quadrature, the QEI may be in 1x if
			 * the selected feedback is edge counter. Additionally if the
			 * quadrature source is the CTRE Mag encoder, then the CPR is known.
			 * This is nice in that the calling app does not require knowing the
			 * CPR at all. So do both checks here.
			 */
			int qeiPulsePerCount = 4; /* default to 4x */
			switch (m_feedbackDevice) {
			case CtreMagEncoder_Relative:
			case CtreMagEncoder_Absolute:
				/*
				 * we assume the quadrature signal comes from the MagEnc, of
				 * which we know the CPR already
				 */
				retval = kNativePwdUnitsPerRotation;
				scalingAvail = true;
				break;
			case EncRising: /*
							 * Talon's QEI is setup for 1x, so perform 1x math
							 */
			case EncFalling:
				qeiPulsePerCount = 1;
				break;
			case QuadEncoder: /* Talon's QEI is 4x */
			default: /*
						 * pulse width and everything else, assume its regular
						 * quad use.
						 */
				break;
			}
			if (scalingAvail) {
				/* already deduced the scalar above, we're done. */
			} else {
				/* we couldn't deduce the scalar just based on the selection */
				if (0 == m_codesPerRev) {
					/*
					 * caller has never set the CPR. Most likely caller is just
					 * using engineering units so fall to the bottom of this
					 * func.
					 */
				} else {
					/* Talon expects PPR units */
					retval = qeiPulsePerCount * m_codesPerRev;
					scalingAvail = true;
				}
			}
		}
			break;
		case EncRising:
		case EncFalling:
			if (0 == m_codesPerRev) {
				/*
				 * caller has never set the CPR. Most likely caller is just
				 * using engineering units so fall to the bottom of this func.
				 */
			} else {
				/* Talon expects PPR units */
				retval = 1 * m_codesPerRev;
				scalingAvail = true;
			}
			break;
		case AnalogPot:
		case AnalogEncoder:
			if (0 == m_numPotTurns) {
				/*
				 * caller has never set the CPR. Most likely caller is just
				 * using engineering units so fall to the bottom of this func.
				 */
			} else {
				retval = (double) kNativeAdcUnitsPerRotation / m_numPotTurns;
				scalingAvail = true;
			}
			break;
		case CtreMagEncoder_Relative:
		case CtreMagEncoder_Absolute:
		case PulseWidth:
			retval = kNativePwdUnitsPerRotation;
			scalingAvail = true;
			break;
		default:
			break;
		}
		/* if scaling info is not available give caller zero */
		if (false == scalingAvail) {
			return 0;
		}
		return retval;
	}

	/**
	 * @param fullRotations
	 *            double precision value representing number of rotations of
	 *            selected feedback sensor. If user has never called the config
	 *            routine for the selected sensor, then the caller is likely
	 *            passing rotations in engineering units already, in which case
	 *            it is returned as is.
	 * @return fullRotations in native engineering units of the Talon SRX
	 *         firmware.
	 * @see configPotentiometerTurns
	 * @see configEncoderCodesPerRev
	 */
	private int scaleRotationsToNativeUnits(FeedbackDevice devToLookup, double fullRotations) {
		/* first assume we don't have config info, prep the default return */
		int retval = (int) fullRotations;
		/* retrieve scaling info */
		double scalar = getNativeUnitsPerRotationScalar(devToLookup);
		/* apply scalar if its available */
		if (scalar > 0) {
			retval = (int) (fullRotations * scalar);
		}
		return retval;
	}

	/**
	 * @param rpm
	 *            double precision value representing number of rotations per
	 *            minute of selected feedback sensor. If user has never called
	 *            the config routine for the selected sensor, then the caller is
	 *            likely passing rotations in engineering units already, in
	 *            which case it is returned as is.
	 * @return sensor velocity in native engineering units of the Talon SRX
	 *         firmware.
	 * @see configPotentiometerTurns
	 * @see configEncoderCodesPerRev
	 */
	private int scaleVelocityToNativeUnits(FeedbackDevice devToLookup, double rpm) {
		/* first assume we don't have config info, prep the default return */
		int retval = (int) rpm;
		/* retrieve scaling info */
		double scalar = getNativeUnitsPerRotationScalar(devToLookup);
		/* apply scalar if its available */
		if (scalar > 0) {
			retval = (int) (rpm * kMinutesPer100msUnit * scalar);
		}
		return retval;
	}

	/**
	 * @param nativePos
	 *            integral position of the feedback sensor in native Talon SRX
	 *            units. If user has never called the config routine for the
	 *            selected sensor, then the return will be in TALON SRX units as
	 *            well to match the behavior in the 2015 season.
	 * @return double precision number of rotations, unless config was never
	 *         performed.
	 * @see configPotentiometerTurns
	 * @see configEncoderCodesPerRev
	 */
	private double scaleNativeUnitsToRotations(FeedbackDevice devToLookup, int nativePos) {
		/* first assume we don't have config info, prep the default return */
		double retval = (double) nativePos;
		/* retrieve scaling info */
		double scalar = getNativeUnitsPerRotationScalar(devToLookup);
		/* apply scalar if its available */
		if (scalar > 0) {
			retval = ((double) nativePos) / scalar;
		}
		return retval;
	}

	/**
	 * @param nativeVel
	 *            integral velocity of the feedback sensor in native Talon SRX
	 *            units. If user has never called the config routine for the
	 *            selected sensor, then the return will be in TALON SRX units as
	 *            well to match the behavior in the 2015 season.
	 * @return double precision of sensor velocity in RPM, unless config was
	 *         never performed.
	 * @see configPotentiometerTurns
	 * @see configEncoderCodesPerRev
	 */
	private double scaleNativeUnitsToRpm(FeedbackDevice devToLookup, long nativeVel) {
		/* first assume we don't have config info, prep the default return */
		double retval = (double) nativeVel;
		/* retrieve scaling info */
		double scalar = getNativeUnitsPerRotationScalar(devToLookup);
		/* apply scalar if its available */
		if (scalar > 0) {
			retval = (double) (nativeVel) / (scalar * kMinutesPer100msUnit);
		}
		return retval;
	}

	/**
	 * Enables Talon SRX to automatically zero the Sensor Position whenever an
	 * edge is detected on the index signal.
	 *
	 * @param enable
	 *            boolean input, pass true to enable feature or false to
	 *            disable.
	 * @param risingEdge
	 *            boolean input, pass true to clear the position on rising edge,
	 *            pass false to clear the position on falling edge.
	 */
	public void enableZeroSensorPositionOnIndex(boolean enable, boolean risingEdge) {
		if (enable) {
			/*
			 * enable the feature, update the edge polarity first to ensure it
			 * is correct before the feature is enabled.
			 */
			setParameter(ParamEnum.eQuadIdxPolarity, risingEdge ? 1 : 0);
			setParameter(ParamEnum.eClearPositionOnIdx, 1);
		} else {
			/* disable the feature first, then update the edge polarity. */
			setParameter(ParamEnum.eClearPositionOnIdx, 0);
			setParameter(ParamEnum.eQuadIdxPolarity, risingEdge ? 1 : 0);
		}
	}

	/**
	 * Enables Talon SRX to automatically zero the Sensor Position whenever an
	 * edge is detected on the Forward Limit Switch signal.
	 *
	 * @param enable
	 *            boolean input, pass true to enable feature or false to
	 *            disable.
	 */
	public void enableZeroSensorPositionOnForwardLimit(boolean enable) {
		setParameter(ParamEnum.eClearPositionOnLimitF, enable ? 1 : 0);
	}

	/**
	 * Enables Talon SRX to automatically zero the Sensor Position whenever an
	 * edge is detected on the Reverse Limit Switch signal.
	 *
	 * @param enable
	 *            boolean input, pass true to enable feature or false to
	 *            disable.
	 */
	public void enableZeroSensorPositionOnReverseLimit(boolean enable) {
		setParameter(ParamEnum.eClearPositionOnLimitR, enable ? 1 : 0);
	}

	/**
	 * Calling application can opt to speed up the handshaking between the robot
	 * API and the Talon to increase the download rate of the Talon's Motion
	 * Profile. Ideally the period should be no more than half the period of a
	 * trajectory point.
	 */
	public void changeMotionControlFramePeriod(int periodMs) {
		_srx.changeMotionControlFramePeriod(periodMs);
	}

	/**
	 * Clear the buffered motion profile in both Talon RAM (bottom), and in the
	 * API (top). Be sure to check getMotionProfileStatus() to know when the
	 * buffer is actually cleared.
	 */
	public void clearMotionProfileTrajectories() {
		_srx.clearMotionProfileTrajectories();
	}

	/**
	 * Retrieve just the buffer count for the api-level (top) buffer. This
	 * routine performs no CAN or data structure lookups, so its fast and ideal
	 * if caller needs to quickly poll the progress of trajectory points being
	 * emptied into Talon's RAM. Otherwise just use GetMotionProfileStatus.
	 *
	 * @return number of trajectory points in the top buffer.
	 */
	public int getMotionProfileTopLevelBufferCount() {
		return _srx.getMotionProfileTopLevelBufferCount();
	}

	/**
	 * Push another trajectory point into the top level buffer (which is emptied
	 * into the Talon's bottom buffer as room allows).
	 *
	 * <p>
	 * Will return CTR_OKAY if trajectory point push ok. CTR_BufferFull if
	 * buffer is full due to kMotionProfileTopBufferCapacity.
	 *
	 * @param trajPt
	 *            {@link TrajectoryPoint}
	 * @return CTR_OKAY or CTR_BufferFull.
	 */
	public boolean pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
		/* check if there is room */
		if (isMotionProfileTopLevelBufferFull()) {
			return false;
		}
		/* convert position and velocity to native units */
		int targPos = scaleRotationsToNativeUnits(m_feedbackDevice, trajPt.position);
		int targVel = scaleVelocityToNativeUnits(m_feedbackDevice, trajPt.velocity);
		/* bounds check signals that require it */
		// profileSlotSelect0 should just be profileSlotSelect for compatibility with old library, but was changed so this would build without definition of old TrajectoryPoint
		int profileSlotSelect = (trajPt.profileSlotSelect0 > 0) ? 1 : 0;
		//int timeDurMs = 10;//trajPt.timeDurMs;
		///* cap time to [0ms, 255ms], 0 and 1 are both interpreted as 1ms. */
		//if (timeDurMs > 255) {
		//	timeDurMs = 255;
		//}
		//if (timeDurMs < 0) {
		//	timeDurMs = 0;
		//}
		/* send it to the top level buffer */
		_trajPt.headingDeg = 0;
		_trajPt.position = targPos;
		_trajPt.velocity = targVel;
		_trajPt.profileSlotSelect0 = profileSlotSelect;
		_trajPt.zeroPos = trajPt.zeroPos;
		_trajPt.isLastPoint = trajPt.isLastPoint;
		// trajPt.velocityOnly is not supported
		// timeDurMs is not supported
		_srx.pushMotionProfileTrajectory(_trajPt);
		return true;
	}

	/**
	 * @return true if api-level (top) buffer is full.
	 */
	public boolean isMotionProfileTopLevelBufferFull() {
		return _srx.isMotionProfileTopLevelBufferFull();
	}

	/**
	 * This must be called periodically to funnel the trajectory points from the
	 * API's top level buffer to the Talon's bottom level buffer. Recommendation
	 * is to call this twice as fast as the executation rate of the motion
	 * profile. So if MP is running with 20ms trajectory points, try calling
	 * this routine every 10ms. All motion profile functions are thread-safe
	 * through the use of a mutex, so there is no harm in having the caller
	 * utilize threading.
	 */
	public void processMotionProfileBuffer() {
		_srx.processMotionProfileBuffer();
	}

	/**
	 * Retrieve all Motion Profile status information. Since this all comes from
	 * one CAN frame, its ideal to have one routine to retrieve the frame once
	 * and decode everything.
	 *
	 * @param motionProfileStatus
	 *            [out] contains all progress information on the currently
	 *            running MP. Caller should must instantiate the
	 *            motionProfileStatus object first then pass into this routine
	 *            to be filled.
	 */
	public void getMotionProfileStatus(MotionProfileStatus motionProfileStatus) {
		_srx.getMotionProfileStatus(motionProfileStatus);
	}

	/**
	 * Internal method to set the contents.
	 */
	protected void setMotionProfileStatusFromJNI(MotionProfileStatus motionProfileStatus, int flags,
			int profileSlotSelect, int targPos, int targVel, int topBufferRem, int topBufferCnt, int btmBufferCnt,
			int outputEnable) {
		motionProfileStatus.topBufferRem = topBufferRem;
		motionProfileStatus.topBufferCnt = topBufferCnt;
		motionProfileStatus.btmBufferCnt = btmBufferCnt;
		motionProfileStatus.hasUnderrun = ((flags & CanTalonJNI.kMotionProfileFlag_HasUnderrun) > 0);
		motionProfileStatus.isUnderrun = ((flags & CanTalonJNI.kMotionProfileFlag_IsUnderrun) > 0);
		motionProfileStatus.activePointValid = ((flags & CanTalonJNI.kMotionProfileFlag_ActTraj_IsValid) > 0);
		motionProfileStatus.isLast = ((flags & CanTalonJNI.kMotionProfileFlag_ActTraj_IsLast) > 0);
		//motionProfileStatus.velocityOnly = ((flags & CanTalonJNI.kMotionProfileFlag_ActTraj_VelOnly) > 0);
		//motionProfileStatus.position = scaleNativeUnitsToRotations(m_feedbackDevice, targPos);
		//motionProfileStatus.velocity = scaleNativeUnitsToRpm(m_feedbackDevice, targVel);
		motionProfileStatus.profileSlotSelect = profileSlotSelect;
		//motionProfileStatus.outputEnable = SetValueMotionProfile.valueOf(outputEnable);
		//motionProfileStatus.zeroPos = false; // this signal is only
															// used sending pts
															// to
		// Talon
		//motionProfileStatus.activePoint.timeDurMs = 0; // this signal is only
														// used sending pts to
	}

	/**
	 * Clear the hasUnderrun flag in Talon's Motion Profile Executer when MPE is
	 * ready for another point, but the low level buffer is empty.
	 *
	 * <p>
	 * Once the Motion Profile Executer sets the hasUnderrun flag, it stays set
	 * until Robot Application clears it with this routine, which ensures Robot
	 * Application gets a chance to instrument or react. Caller could also check
	 * the isUnderrun flag which automatically clears when fault condition is
	 * removed.
	 */
	public void clearMotionProfileHasUnderrun() {
		setParameter(ParamEnum.eMotionProfileHasUnderrunErr, 0);
	}

	/**
	 * Set the Cruise Velocity used in Motion Magic Control Mode.
	 * 
	 * @param motmagicCruiseVeloc
	 *            Cruise(peak) velocity in RPM.
	 */
	public void setMotionMagicCruiseVelocity(double motMagicCruiseVeloc) {
		int sensorUnitsPer100ms = scaleVelocityToNativeUnits(m_feedbackDevice, motMagicCruiseVeloc);
		_srx.configMotionCruiseVelocity(sensorUnitsPer100ms, _setTimeoutMs);
	}

	/**
	 * Set the Acceleration used in Motion Magic Control Mode.
	 * 
	 * @param motMagicAccel
	 *            Accerleration in RPM per second.
	 */
	public void setMotionMagicAcceleration(double motMagicAccel) {
		int sensorUnitsPer100msPerSec = scaleVelocityToNativeUnits(m_feedbackDevice, motMagicAccel);
		_srx.configMotionAcceleration(sensorUnitsPer100msPerSec, _setTimeoutMs);
	}

	/**
	 * @return polled motion magic cruise velocity setting from Talon. RPM if
	 *         units are configured, velocity native units otherwise.
	 */
	public double getMotionMagicCruiseVelocity() {
		// Update the info in m_impl.
		double retval = getParameter(ParamEnum.eMotMag_VelCruise);

		return scaleNativeUnitsToRpm(m_feedbackDevice, (int) retval);
	}

	/**
	 * @return polled motion magic acceleration setting from Talon. RPM per
	 *         second if units are configured, velocity native units per second
	 *         otherwise.
	 */
	public double getMotionMagicAcceleration() {
		/* get the last received update */
		double retval = getParameter(ParamEnum.eMotMag_Accel);

		// TODO: Add Error Checking

		return scaleNativeUnitsToRpm(m_feedbackDevice, (int) retval);
	}

	public void setCurrentLimit(int amps) {
		_srx.configContinuousCurrentLimit(amps, _setTimeoutMs);
		_srx.configPeakCurrentDuration(0,
				_setTimeoutMs); /*
								 * clear Peak, firmware will then just use
								 * continuous
								 */
	}

	public void EnableCurrentLimit(boolean enable) {
		_srx.enableCurrentLimit(enable);
	}

	public int getGadgeteerStatus(GadgeteerUartStatus status) {
		/* unsupported in the CANTalon wrapper */
		status.type = GadgeteerProxyType.Unknown;
		status.conn = GadgeteerConnection.Unknown;
		status.resetCount = 0;
		status.bitrate = 0;
		return -1;
	}

	public String getLastError() {
		ErrorCode error = _srx.getLastError();
		return error.toString();
	}

	@Override
	public void setExpiration(double timeout) {
//		m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
//		return m_safetyHelper.getExpiration();
		return 1;
	}

	@Override
	public boolean isAlive() {
//		return m_safetyHelper.isAlive();
		return true;
	}

	@Override
	public boolean isSafetyEnabled() {
//		return m_safetyHelper.isSafetyEnabled();
		return false;
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
//		m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public String getDescription() {
		return "CANTalon ID " + _deviceNumber;
	}

	// ---- essentially a copy of SendableBase -------//
	private String m_name = "";
	private String m_subsystem = "Ungrouped";

	/**
	 * Free the resources used by this object.
	 */
	public void free() {
		LiveWindow.remove(this);
	}

	@Override
	public final synchronized String getName() {
		return m_name;
	}

	@Override
	public final synchronized void setName(String name) {
		m_name = name;
	}

	/**
	 * Sets the name of the sensor with a channel number.
	 *
	 * @param moduleType
	 *            A string that defines the module name in the label for the
	 *            value
	 * @param channel
	 *            The channel number the device is plugged into
	 */
	protected final void setName(String moduleType, int channel) {
		setName(moduleType + "[" + channel + "]");
	}

	/**
	 * Sets the name of the sensor with a module and channel number.
	 *
	 * @param moduleType
	 *            A string that defines the module name in the label for the
	 *            value
	 * @param moduleNumber
	 *            The number of the particular module type
	 * @param channel
	 *            The channel number the device is plugged into (usually PWM)
	 */
	protected final void setName(String moduleType, int moduleNumber, int channel) {
		setName(moduleType + "[" + moduleNumber + "," + channel + "]");
	}

	@Override
	public final synchronized String getSubsystem() {
		return m_subsystem;
	}

	@Override
	public final synchronized void setSubsystem(String subsystem) {
		m_subsystem = subsystem;
	}

	/**
	 * Add a child component.
	 *
	 * @param child
	 *            child component
	 */
	protected final void addChild(Object child) {
		LiveWindow.addChild(this, child);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Speed Controller");
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);
	}

	//--------------Extensions to mitigate difference between legacy CANTalon and latest TalonSRX
	void changeSetTimeout(int timeMs)
	{
		_setTimeoutMs = timeMs;
	}
	void changeGetTimeout(int timeMs)
	{
		_setTimeoutMs = timeMs;
	}
	TalonSRX GetTalonSRX()
	{
		return _srx;
	}
}

