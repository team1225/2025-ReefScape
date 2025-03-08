package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Constants.AlgaeBlasterConstants;
import frc.robot.Constants.CoralatorConstants;
import frc.robot.Ports;

/**
 * The {@code Mouth} class contains fields and methods pertaining to the
 * function of the mouth.
 */
public class AlgaeBlaster extends SubsystemBase {
    private final SparkMax leader;
    private final SparkMax follower;
    private final SparkMaxConfig leaderMotorConfig;
    private final SparkMaxConfig followerMotorConfig;

    public AlgaeBlaster() {
        leader = new SparkMax(Ports.CAN.ALGAE_BLASTER_LEADER, MotorType.kBrushed);
        follower = new SparkMax(Ports.CAN.ALGAE_BLASTER_FOLLOWER, MotorType.kBrushed); //CHANGE THIS!

        leaderMotorConfig = new SparkMaxConfig();
        leaderMotorConfig
            .inverted(false)
            .idleMode(AlgaeBlasterConstants.MOTOR_IDLE_MODE)
            .smartCurrentLimit(AlgaeBlasterConstants.CURRENT_LIMIT_AMPS);
        leader.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerMotorConfig = new SparkMaxConfig();
        followerMotorConfig
            .follow(leader, true)
            .idleMode(AlgaeBlasterConstants.MOTOR_IDLE_MODE)
            .smartCurrentLimit(AlgaeBlasterConstants.CURRENT_LIMIT_AMPS);
        follower.configure(followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }	

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

   
    public void Intake() {
        leader.set(CoralatorConstants.INTAKE_SPEED);
    }

    public void Eject() {
        leader.setVoltage(CoralatorConstants.EJECT_SPEED);
    }

    public void Stop() {
        leader.set(0);
    } 
}