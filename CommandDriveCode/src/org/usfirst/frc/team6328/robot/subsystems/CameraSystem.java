package org.usfirst.frc.team6328.robot.subsystems;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls front and back cameras
 */
public class CameraSystem extends Subsystem {

	private UsbCamera frontCamera = new UsbCamera("Front Camera", 0);
	private UsbCamera rearCamera = new UsbCamera("Rear Camera", 2);
	private MjpegServer mjpegServer;
	private boolean serverCreated = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CameraSystem() {
    	frontCamera.setResolution(320, 240);
        frontCamera.setFPS(30);
        rearCamera.setResolution(320, 240);
        rearCamera.setFPS(30);
        //mjpegServer.setSource(frontCamera);
    }
    
    public void useFrontCamera() {
    	System.out.println("Switching to front camera");
    	if (!serverCreated) {
    		mjpegServer = CameraServer.getInstance().addServer("Server", 1181);
    		serverCreated = true;
    	}
    	mjpegServer.setSource(frontCamera);
    }
    
    public void useRearCamera() {
    	System.out.println("Switching to rear camera");
    	if (!serverCreated) {
    		mjpegServer = CameraServer.getInstance().addServer("Server", 1181);
    		serverCreated = true;
    	}
    	mjpegServer.setSource(rearCamera);
    }
}

