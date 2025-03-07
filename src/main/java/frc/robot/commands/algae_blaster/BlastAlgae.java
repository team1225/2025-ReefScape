package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TelescopingArmConstants;
import frc.robot.subsystems.AlgaeBlaster;
import frc.robot.subsystems.TelescopingArm;

public class BlastAlgae extends Command {
    
    private AlgaeBlaster algaeBlaster;
    private TelescopingArm telescopingArm;
    private Timer timer;
    private static final double KICK_IT_TIME = 2.0; // Time in seconds before kicking the algae

    public BlastAlgae(AlgaeBlaster algaeBlaster, TelescopingArm telescopingArm) {    
        this.algaeBlaster = algaeBlaster;
        this.telescopingArm = telescopingArm;
        this.timer = new Timer();
        addRequirements(algaeBlaster, telescopingArm);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        algaeBlaster.Eject();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(KICK_IT_TIME)) {
            telescopingArm.setDesiredState(TelescopingArmConstants.KICK_IT_STATE);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(KICK_IT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        algaeBlaster.Stop();
    }
}
