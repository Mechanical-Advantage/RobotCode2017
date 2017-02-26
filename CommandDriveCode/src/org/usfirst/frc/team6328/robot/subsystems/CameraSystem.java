package org.usfirst.frc.team6328.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls front and back cameras
 */
public class CameraSystem extends Subsystem {

	private UsbCamera frontCamera;
	private UsbCamera rearCamera/* = new UsbCamera("Rear Camera", 0)*/;
	//private MjpegServer mjpegServer;
	private boolean serverCreated = false;
	private boolean frontCameraAdded = false;
	private boolean rearCameraAdded = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public CameraSystem() {
    	/*frontCamera.setResolution(320, 240);
        frontCamera.setFPS(15);
        /*rearCamera.setResolution(320, 240);
        rearCamera.setFPS(30);*/
        //mjpegServer.setSource(frontCamera);
        //CameraServer.getInstance().addCamera(frontCamera);
        //CameraServer.getInstance().addCamera(rearCamera);
    }
    
    // To get network tables publishing, must use startAutomaticCapture and have it create the UsbCamera.
    // This should not have to work this way, but CameraServer is finicky, and this was the only way it worked
    // Written for WPILib 2017.3.1
    // Note: Second Camera is just internal, selecting on dashboard will have no effect
    private UsbCamera setupServer(int id) {
    	serverCreated = true;
    	return CameraServer.getInstance().startAutomaticCapture("Video Feed", id);
    }
    
    public void useFrontCamera() {
    	if (!frontCameraAdded) {
    		if (!serverCreated) {
        		frontCamera = setupServer(2);
        	} else {
        		frontCamera = new UsbCamera("Second Camera", 2);
        	}
    		frontCamera.setResolution(320, 240);
            frontCamera.setFPS(15);
    		frontCameraAdded = true;
    	}
    	CameraServer.getInstance().getServer().setSource(frontCamera);
    	System.out.println("Switching to front camera");
    	/*if (!serverCreated) {
    		mjpegServer = CameraServer.getInstance().addServer("serve_video", 1181);
    		serverCreated = true;
    	}
    	if (!frontCameraAdded) {
    		CameraServer.getInstance().addCamera(frontCamera);
    		frontCameraAdded = true;
    	}
    	mjpegServer.setSource(frontCamera);*/
    	//CameraServer.getInstance().removeCamera("Rear Camera");
    	//CameraServer.getInstance().addCamera(frontCamera);
    }
    
    public void useRearCamera() {
    	if (!rearCameraAdded) {
    		if (!serverCreated) {
        		rearCamera = setupServer(0);
        	} else {
        		rearCamera = new UsbCamera("Second Camera", 0);
        	}
    		rearCamera.setResolution(320, 240);
    		rearCamera.setFPS(15);
    		rearCameraAdded = true;
    	}
    	CameraServer.getInstance().getServer().setSource(rearCamera);
    	System.out.println("Switching to rear camera");
    	/*if (!serverCreated) {
    		mjpegServer = CameraServer.getInstance().addServer("serve_video", 1181);
    		serverCreated = true;
    	}
    	if (!rearCameraAdded) {
    		CameraServer.getInstance().addCamera(rearCamera);
    		rearCameraAdded = true;
    	}
    	mjpegServer.setSource(rearCamera);*/
    	//CameraServer.getInstance().removeCamera("Front Camera");
    	//CameraServer.getInstance().addCamera(rearCamera);
    }
}

