package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeBlaster;
import frc.robot.subsystems.TelescopingArm;


public class IntakeAlgae extends Command {
	private AlgaeBlaster algaeBlaster;
	private TelescopingArm telescopingArm;

	public IntakeAlgae(AlgaeBlaster algaeBlaster, TelescopingArm telescopingArm) {	
		this.algaeBlaster = algaeBlaster;
		this.telescopingArm = telescopingArm;
		addRequirements(algaeBlaster, telescopingArm);
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		telescopingArm.setDesiredState(0.0);
		algaeBlaster.Intake();
	}
}
