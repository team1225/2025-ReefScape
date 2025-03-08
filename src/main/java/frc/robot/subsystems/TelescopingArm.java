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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopingArmConstants;
import frc.robot.Ports;

public class TelescopingArm extends SubsystemBase {
  private final SparkMax motor;
	private final SparkMaxConfig motorConfig;
  private final SparkAbsoluteEncoder absoluteEncoder;
  private Double desiredState;
  private final SparkClosedLoopController closedLoopController;
  private boolean atUpperLimit;
  private boolean atLowerLimit;
  /** Creates a new Arm. */
  public TelescopingArm() {
    motor = new SparkMax(Ports.CAN.TELESCOPING_ARM, MotorType.kBrushless);
		motorConfig = new SparkMaxConfig();
    absoluteEncoder = motor.getAbsoluteEncoder();
		closedLoopController = motor.getClosedLoopController();

    motorConfig
			.inverted(true)
      .idleMode(TelescopingArmConstants.IDLE_MODE)
      .smartCurrentLimit(TelescopingArmConstants.CURRENT_LIMIT_AMPS);
    motorConfig.absoluteEncoder
      .positionConversionFactor(1) // Rotations
      .velocityConversionFactor(1); // RPMs
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // These are example gains you may need to them for your own robot!
      .pid(TelescopingArmConstants.TURNING_P, TelescopingArmConstants.TURNING_I, TelescopingArmConstants.TURNING_D)
      .outputRange(TelescopingArmConstants.TURNING_MIN_OUTPUT_NORMALIZED, TelescopingArmConstants.TURNING_MAX_OUTPUT_NORMALIZED)
			.velocityFF(TelescopingArmConstants.TURNING_FF)
            // Disable PID wrap around for the ARM. We do not want a Ferris Wheel.
      .positionWrappingEnabled(false);
    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(TelescopingArmConstants.MAX_VELOCITY)
        .maxAcceleration(TelescopingArmConstants.MAX_VELOCITY * TelescopingArmConstants.MAX_ACCELERATION_FACTOR)
        .allowedClosedLoopError(TelescopingArmConstants.ALLOWED_ERROR);
		motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		desiredState = absoluteEncoder.getPosition();

    //sets the desired state to the current state so the arm does not move during robot startup
    setDesiredState(desiredState);
  }

  @Override
  public void periodic() {
      atUpperLimit = getPosition() >= TelescopingArmConstants.SOFT_LIMIT_FORWARD;
      atLowerLimit = getPosition() <= TelescopingArmConstants.SOFT_LIMIT_REVERSE;
  }

  // ******************************************************************************************
  // Setters
  // ******************************************************************************************

  /**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(Double desiredState) {
        closedLoopController.setReference(desiredState, 
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0);
      this.desiredState = desiredState;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // ******************************************************************************************
  // Commands
  // ******************************************************************************************

  public Command setPositionCommand(double targetPosition) {
    return Commands.runOnce(() -> setDesiredState(targetPosition));
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> motor.clearFaults());
  }

  public Command setVoltageCommand(double voltage) {
    return Commands.runOnce(() -> setVoltage(voltage));
  }

  // ******************************************************************************************
  // Getters
  // ******************************************************************************************

  public SparkAbsoluteEncoder getTurningAbsoluteEncoder()
	{
		return absoluteEncoder;
	}

  public double getDesiredState() {
    return this.desiredState;
  }

  public double getPosition() {
    return absoluteEncoder.getPosition();
  }

  public boolean onPlusSoftwareLimit() {
    return absoluteEncoder.getPosition() >= motor.configAccessor.softLimit.getForwardSoftLimit();
  }

  public boolean onMinusSoftwareLimit() {
    return motor.getEncoder().getPosition() <= motor.configAccessor.softLimit.getReverseSoftLimit();
  }

  public boolean onPlusHardwareLimit() {
    return motor.getForwardLimitSwitch().isPressed();
  }

  public boolean onMinusHardwareLimit() {
    return motor.getReverseLimitSwitch().isPressed();
  }

  public boolean onLimit() {
    return onPlusHardwareLimit() || onMinusHardwareLimit() ||
           onPlusSoftwareLimit() || onMinusSoftwareLimit();
  }

  public boolean isAtUpperLimit() {
    return atUpperLimit;
  }

  public boolean isAtLowerLimit() {
    return atLowerLimit;
  }

}
