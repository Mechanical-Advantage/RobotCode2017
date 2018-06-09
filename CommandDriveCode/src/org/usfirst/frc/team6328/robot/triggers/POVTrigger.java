package org.usfirst.frc.team6328.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Trigger so d-pad can be used as buttons
 */
public class POVTrigger extends Button {
	
	private Joystick joystick;
	private int angle;
	
	public POVTrigger(Joystick joystick, int angle) {
		this.joystick = joystick;
		this.angle = angle;
	}

    public boolean get() {
        return joystick.getPOV() == angle;
    }
}
