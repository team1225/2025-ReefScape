// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports;

public class Arm extends SubsystemBase {
  private final SparkMax m_turningSparkMax;
	private final SparkMaxConfig turningConfig;
  private final SparkAbsoluteEncoder m_turningAbsoluteEncoder;
  private Rotation2d m_desiredState;
  private final SparkClosedLoopController m_TurningClosedLoopController;


  /** Creates a new Arm. */
  public Arm() {
    m_turningSparkMax = new SparkMax(Ports.CAN.ARM_TURNING, MotorType.kBrushless);
		turningConfig = new SparkMaxConfig();
    m_turningAbsoluteEncoder = m_turningSparkMax.getAbsoluteEncoder();
		m_TurningClosedLoopController = m_turningSparkMax.getClosedLoopController();

    turningConfig
			.inverted(false)
      .idleMode(ArmConstants.TURNING_MOTOR_IDLE_MODE)
      .smartCurrentLimit(ArmConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
    turningConfig.absoluteEncoder
      .positionConversionFactor(ArmConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION) // radians
      .velocityConversionFactor(ArmConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM); // radians per second
    turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // These are example gains you may need to them for your own robot!
      .pid(ArmConstants.TURNING_P, ArmConstants.TURNING_I, ArmConstants.TURNING_D)
      .outputRange(ArmConstants.TURNING_MIN_OUTPUT_NORMALIZED, ArmConstants.TURNING_MAX_OUTPUT_NORMALIZED)
			.velocityFF(ArmConstants.TURNING_FF)
            // Disable PID wrap around for the ARM. We do not want a Ferris Wheel.
      .positionWrappingEnabled(false)
      .positionWrappingInputRange(0, 2);
    turningConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(ArmConstants.MAX_VELOCITY)
        .maxAcceleration(ArmConstants.MAX_VELOCITY * ArmConstants.MAX_ACCELERATION_FACTOR)
        .allowedClosedLoopError(ArmConstants.ALLOWED_ERROR);
		m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_desiredState = new Rotation2d(m_turningAbsoluteEncoder.getPosition());

    //sets the desired state to the current state so the arm does not move during robot startup
    setDesiredState(m_desiredState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(Rotation2d desiredState) {
			m_TurningClosedLoopController.setReference(desiredState.getRadians(), 
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
      m_desiredState = Rotation2d.fromRadians(desiredState.getRadians());
  }

  public SparkAbsoluteEncoder getTurningAbsoluteEncoder()
	{
		return m_turningAbsoluteEncoder;
	}

  public Rotation2d getDesiredState() {
    return m_desiredState;
  }

  public Rotation2d getPosition() {
    return new Rotation2d(m_turningAbsoluteEncoder.getPosition());
  }

}
