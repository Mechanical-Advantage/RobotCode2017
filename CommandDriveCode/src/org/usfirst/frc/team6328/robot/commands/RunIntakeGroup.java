package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Runs the intake, loader, climber as intake, and trigger backwards
 */
public class RunIntakeGroup extends CommandGroup {

    public RunIntakeGroup() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addParallel(new RunIntake(false));
    	addParallel(new RunLoader(false));
    	addParallel(new RunTrigger(true));
    	addParallel(new RunClimberForIntake());
    }
}
