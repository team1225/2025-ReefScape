package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;


/**
 * The {@code Mouth} class contains fields and methods pertaining to the function of the mouth.
 */
public class AlgaeBlaster extends SubsystemBase {
	
	static final int WAIT_MS = 1000;
	
	DoubleSolenoid blastSolenoid;
	
	public enum Position {
		RESET,
		BLAST, 
		UNKNOWN;
	}

	public AlgaeBlaster() {
		// the double solenoid valve will send compressed air from the tank wherever needed
		blastSolenoid = new DoubleSolenoid(Ports.CAN.PCM, PneumaticsModuleType.REVPH, Ports.PCM.ALGAE_BLASTER_BLAST, Ports.PCM.ALGAE_BLASTER_RESET); // make sure ports are properly set in Ports.java	
	}
	
	/*@Override
	public void initDefaultCommand() {

	}*/

	@Override
	public void periodic() {
		// Put code here to be run every loop

	}

	public void setPosition(Position pos) //Telling the piston to be in the selected position 
	{
		switch(pos)
		{
			case BLAST: //Telling the solenoid to have the piston go up
			{
				blastSolenoid.set(DoubleSolenoid.Value.kReverse); // adjust direction if needed
				break;
			}
			case RESET: //Telling the solenoid to have the piston go down
			{
				blastSolenoid.set(DoubleSolenoid.Value.kForward); // adjust direction if needed
				break;
			}
			default:
			{
				// do nothing
			}
		}
	}

	public Position getPosition() //Getting the current gear
	{
		DoubleSolenoid.Value value = blastSolenoid.get();
		
		switch(value)
		{
			case kReverse: //Checking if the piston is in the kReverse position (high)
			{
				return Position.BLAST;
			}
			case kForward: //Checking if the piston is in the kFoward position (low)
			{
				return Position.RESET;
			}
			default: //gear unknown
			{
				return Position.UNKNOWN;
			}
		}
	}

	public boolean isDangerous() {
		return getPosition() == Position.RESET;
	}

}