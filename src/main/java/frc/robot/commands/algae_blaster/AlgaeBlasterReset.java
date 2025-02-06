package frc.robot.commands.algae_blaster;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeBlaster;
import frc.robot.subsystems.AlgaeBlaster.Position;

// To reset the AlgaeBlaster
/*public*/ class AlgaeBlasterReset extends Command {

	
	private AlgaeBlaster algaeBlaster;

	public AlgaeBlasterReset(AlgaeBlaster algaeBlaster) {	
		this.algaeBlaster = algaeBlaster;
		addRequirements(algaeBlaster);
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		System.out.println("AlgaeBlasterReset: initialize");
		algaeBlaster.setPosition(Position.RESET);
	}

}