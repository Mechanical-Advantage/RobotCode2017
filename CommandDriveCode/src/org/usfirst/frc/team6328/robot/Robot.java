
package org.usfirst.frc.team6328.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
//import edu.wpi.first.wpilibj.CameraServer;

import org.usfirst.frc.team6328.robot.commands.DriveDistance;
import org.usfirst.frc.team6328.robot.commands.DriveDistanceOnHeading;
import org.usfirst.frc.team6328.robot.commands.DriveSquare;
import org.usfirst.frc.team6328.robot.commands.DriveToBoilerShootBallsCrossLine;
import org.usfirst.frc.team6328.robot.commands.MotionProfileTest;
import org.usfirst.frc.team6328.robot.commands.PlaceGearCenter;
import org.usfirst.frc.team6328.robot.commands.PlaceGearSideOfAirship;
import org.usfirst.frc.team6328.robot.commands.TurnAndDriveDistance;
import org.usfirst.frc.team6328.robot.commands.TurnToAngle;
import org.usfirst.frc.team6328.robot.subsystems.BallTrigger;
import org.usfirst.frc.team6328.robot.subsystems.CameraSystem;
import org.usfirst.frc.team6328.robot.subsystems.Climber;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6328.robot.subsystems.GearExpeller;
import org.usfirst.frc.team6328.robot.subsystems.TopGear;
import org.usfirst.frc.team6328.robot.subsystems.Intake;
import org.usfirst.frc.team6328.robot.subsystems.Loader;
import org.usfirst.frc.team6328.robot.subsystems.Shooter;

import com.kauailabs.navx.frc.AHRS;

//import org.usfirst.frc.team6328.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static final RobotMap robotMap = new RobotMap();
	
	public static final DriveTrain driveSubsystem = new DriveTrain();
	public static final Intake intakeSubsystem = new Intake();
	public static final Loader loaderSubsystem = new Loader();
	public static final TopGear topGearSubsystem = new TopGear();
	public static final GearExpeller expelGearSubsystem = new GearExpeller();
	public static final BallTrigger triggerSubsystem = new BallTrigger();
	public static final Climber climberSubsystem = new Climber();
	public static final Shooter shooterSubsystem = new Shooter();
	
	public static OI oi;
	public static AHRS ahrs = new AHRS(SPI.Port.kMXP);
	public static final CameraSystem cameraSubsystem = new CameraSystem();

    Command autonomousCommand;
    SendableChooser<Command> chooser;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
		oi = new OI();
        chooser = new SendableChooser<Command>();
        //chooser.addDefault("Default Auto", new ExampleCommand());
//        chooser.addObject("My Auto", new MyAutoCommand());
        if (RobotMap.tuningMode) {
        	chooser.addObject("Rotate 90 degrees", new TurnToAngle(90));
            chooser.addObject("Rotate 180 degrees", new TurnToAngle(180));
            chooser.addObject("Rotate 24 degrees", new TurnToAngle(24));
            chooser.addObject("Drive 24 inches with gyro correction", new DriveDistanceOnHeading(24));
            chooser.addObject("Drive 6 inches with gyro correction", new DriveDistanceOnHeading(6));
            chooser.addObject("Drive 10 feet", new DriveDistance(120));
            chooser.addObject("Drive backwards 10 feet", new DriveDistance(-120));
            chooser.addObject("Drive backwards 24 inches", new DriveDistance(-24));
            chooser.addObject("Drive 5 feet with gyro correction", new DriveDistanceOnHeading(60));
            chooser.addObject("Drive backwards 60 inches", new DriveDistance(-60));
            chooser.addObject("Drive 6 feet with gyro correction", new DriveDistanceOnHeading(72));
            chooser.addObject("Drive 10 feet with gyro correction", new DriveDistanceOnHeading(120));
            chooser.addObject("Turn around and drive 5 feet", new TurnAndDriveDistance(60, 180));
            chooser.addObject("Turn 45 degrees and drive 2 feet", new TurnAndDriveDistance(24, 45));
            chooser.addObject("Drive square 5 feet length", new DriveSquare(60, true));
            chooser.addObject("Drive square 5 feet length backwards", new DriveSquare(-60, true));
            chooser.addObject("Drive backwards 5 feet with gyro correction", new DriveDistanceOnHeading(-60));
            chooser.addObject("Drive 20 feet with gyro correction", new DriveDistanceOnHeading(240));
            chooser.addObject("Motion Profile Test", new MotionProfileTest());
        } else {
        	chooser.addDefault("Do Nothing", null);
        	chooser.addObject("Cross Line not behind airship facing backward", new DriveDistanceOnHeading(-110));
        	chooser.addObject("Place gear centered behind airship", new PlaceGearCenter());
        	chooser.addObject("Place gear left of airship", new PlaceGearSideOfAirship(false));
        	chooser.addObject("Place gear right of airship", new PlaceGearSideOfAirship(true));
        	chooser.addObject("Shoot balls cross line red", new DriveToBoilerShootBallsCrossLine(false));
        	chooser.addObject("Shoot balls cross line blue", new DriveToBoilerShootBallsCrossLine(true));
        	//chooser.addObject("Shoot balls and place gear red", new DriveToBoilerShootBallsPlaceGear(false)); // not fully implemented
        	//chooser.addObject("Shoot balls and place gear blue", new DriveToBoilerShootBallsPlaceGear(true));
        }
        SmartDashboard.putData("Auto mode", chooser);
        System.out.println("NavX firmware version " + ahrs.getFirmwareVersion());
    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit(){
    	driveSubsystem.enableBrakeMode(false);
    	if (RobotMap.tuningMode) {
    		SmartDashboard.putNumber("Final Yaw", ahrs.getYaw());
        	SmartDashboard.putNumber("Right Distance", driveSubsystem.getDistanceRight());
        	SmartDashboard.putNumber("Left Distance", driveSubsystem.getDistanceLeft());
    	}
    	driveSubsystem.stopMotionProfile();
    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() {
    	driveSubsystem.enableBrakeMode(true);
        autonomousCommand = (Command) chooser.getSelected();
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        if (!RobotMap.practiceRobot) {
        	SmartDashboard.putBoolean("Gear Spring Sensor", expelGearSubsystem.getSpringSensor());
        }
    }

    public void teleopInit() {
    	if (RobotMap.practiceRobot) {
    		driveSubsystem.enableBrakeMode(true);
    	}
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        
        ahrs.reset();
        driveSubsystem.resetPosition();
        oi.initShooter();
        if (!RobotMap.practiceRobot) {
        	if (expelGearSubsystem.getCurrentCommand() != null) {
        		expelGearSubsystem.getCurrentCommand().cancel(); // make sure the system is closed and ExpelGear is not running
        	}
        	expelGearSubsystem.close();
        }
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        if (!RobotMap.practiceRobot) {
        	SmartDashboard.putBoolean("Gear Spring Sensor", expelGearSubsystem.getSpringSensor());
        }
        //System.out.println("Count: " + shooterSubsystem.getCount() + " Period: " + shooterSubsystem.getPeriod() + " Speed: " + shooterSubsystem.getSpeed());
        //System.out.println(driveSubsystem.getCurrent());
        //System.out.println(oi.getSniperLevel());
        //System.out.println("Right: " + driveSubsystem.getRotationsRight() + " Left: " + driveSubsystem.getRotationsLeft());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
