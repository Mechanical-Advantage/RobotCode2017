/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6328.robot.CANTalon;

import com.ctre.phoenix.motorcontrol.NeutralMode;

@SuppressWarnings("MethodName")
public class CanTalonJNI  {
  // Motion Profile status bits
  public static final int kMotionProfileFlag_ActTraj_IsValid = 0x1;
  public static final int kMotionProfileFlag_HasUnderrun = 0x2;
  public static final int kMotionProfileFlag_IsUnderrun = 0x4;
  public static final int kMotionProfileFlag_ActTraj_IsLast = 0x8;
  public static final int kMotionProfileFlag_ActTraj_VelOnly = 0x10;

  public static String ERR_CANSessionMux_InvalidBuffer_MESSAGE = "CAN: Invalid Buffer";
  public static String ERR_CANSessionMux_MessageNotFound_MESSAGE = "CAN: Message not found";
  public static String WARN_CANSessionMux_NoToken_MESSAGE = "CAN: No token";
  public static String ERR_CANSessionMux_NotAllowed_MESSAGE = "CAN: Not allowed";
  public static String ERR_CANSessionMux_NotInitialized_MESSAGE = "CAN: Not initialized";
  public static String CTR_RxTimeout_MESSAGE = "CTRE CAN Receive Timeout";
  public static String CTR_TxTimeout_MESSAGE = "CTRE CAN Transmit Timeout";
  public static String CTR_InvalidParamValue_MESSAGE = "CTRE CAN Invalid Parameter";
  public static String CTR_UnexpectedArbId_MESSAGE = "CTRE Unexpected Arbitration ID (CAN Node ID)";
  public static String CTR_TxFailed_MESSAGE = "CTRE CAN Transmit Error";
  public static String CTR_SigNotUpdated_MESSAGE = "CTRE CAN Signal Not Updated";
  
	public final static int CTR_RxTimeout = 1;
	public final static int CTR_TxTimeout = 2;
	public final static int CTR_InvalidParamValue = 3;
	public final static int CTR_UnexpectedArbId = 4;
	public final static int CTR_TxFailed = 5;
	public final static int CTR_SigNotUpdated = 6;
	public final static int ERR_CANSessionMux_InvalidBuffer = -44086;
	public final static int ERR_CANSessionMux_MessageNotFound = -44087;
	public final static int WARN_CANSessionMux_NoToken = 44087;
	public final static int ERR_CANSessionMux_NotAllowed = -44088;


  

}
