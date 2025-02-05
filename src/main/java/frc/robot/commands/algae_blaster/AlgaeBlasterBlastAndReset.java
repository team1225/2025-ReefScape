package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.AlgaeBlaster;
import frc.robot.subsystems.AlgaeBlaster.Position;

public class AlgaeBlasterBlastAndReset extends Command {
    
    private AlgaeBlaster algaeBlaster;
    private Timer timer;
    private static final double RESET_TIME = 2.0; // Time in seconds before reset

    public AlgaeBlasterBlastAndReset(AlgaeBlaster algaeBlaster) {    
        this.algaeBlaster = algaeBlaster;
        this.timer = new Timer();
        addRequirements(algaeBlaster);
    }

    @Override
    public void initialize() {
        System.out.println("algaeBlasterBlast: initialize");
        algaeBlaster.setPosition(Position.BLAST);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // This will run repeatedly until isFinished returns true
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(RESET_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        algaeBlaster.setPosition(Position.RESET);
        timer.stop();
    }
}