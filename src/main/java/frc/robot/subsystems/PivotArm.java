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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotArmConstants;
import frc.robot.Ports;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;

public class PivotArm extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMaxConfig pivotMotorConfig;
    private final SparkAbsoluteEncoder pivotAbsoluteEncoder;
    private Rotation2d pivotDesiredState;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final SysIdRoutine sysIdRoutine;
    
    private boolean atUpperLimit;
    private boolean atLowerLimit;
    
    /** Creates a new Arm. */
    public PivotArm() {
        pivotMotor = new SparkMax(Ports.CAN.PIVOT_ARM, MotorType.kBrushless);
        pivotMotorConfig = new SparkMaxConfig();
        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
        pivotClosedLoopController = pivotMotor.getClosedLoopController();

        pivotMotorConfig
                .inverted(false)
                .idleMode(PivotArmConstants.TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(PivotArmConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
        pivotMotorConfig.absoluteEncoder
                .positionConversionFactor(PivotArmConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION) // radians
                .velocityConversionFactor(PivotArmConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM); // radians per second
        pivotMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(PivotArmConstants.TURNING_P, PivotArmConstants.TURNING_I, PivotArmConstants.TURNING_D)
                .outputRange(PivotArmConstants.TURNING_MIN_OUTPUT_NORMALIZED,
                        PivotArmConstants.TURNING_MAX_OUTPUT_NORMALIZED)
                .velocityFF(PivotArmConstants.TURNING_FF)
                // Disable PID wrap around for the ARM. We do not want a Ferris Wheel.
                .positionWrappingEnabled(false);
        pivotMotorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .maxVelocity(PivotArmConstants.MAX_VELOCITY)
                .maxAcceleration(PivotArmConstants.MAX_VELOCITY * PivotArmConstants.MAX_ACCELERATION_FACTOR)
                .allowedClosedLoopError(PivotArmConstants.ALLOWED_ERROR);
        pivotMotorConfig.softLimit
                .forwardSoftLimit(PivotArmConstants.SOFT_LIMIT_FORWARD)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(PivotArmConstants.SOFT_LIMIT_REVERSE)
                .reverseSoftLimitEnabled(true);
        pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotDesiredState = new Rotation2d(pivotAbsoluteEncoder.getPosition());

        // sets the desired state to the current state so the arm does not move during
        // robot startup
        setDesiredState(pivotDesiredState);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        pivotMotor.setVoltage(volts.in(Volts));
                    },
                    null,
                    this));
    }

    @Override
    public void periodic() {
        atUpperLimit = getPosition().getDegrees() >= (PivotArmConstants.SOFT_LIMIT_FORWARD);
        atLowerLimit = getPosition().getDegrees() <= (PivotArmConstants.SOFT_LIMIT_REVERSE);
    }

    // ******************************************************************************************
    // Setters
    // ******************************************************************************************
    
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(Rotation2d desiredState) {
        pivotClosedLoopController.setReference(desiredState.getRadians(),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0);
        pivotDesiredState = Rotation2d.fromRadians(desiredState.getRadians());
    }

    public void setGoalDegrees(double targetDegrees) {
        setDesiredState(Rotation2d.fromDegrees(targetDegrees));
    }

    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    // ******************************************************************************************
    // Commands
    // ******************************************************************************************
    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> setGoalDegrees(targetDegrees));
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> pivotMotor.clearFaults());
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.runOnce(() -> setVoltage(voltage));
    }
    
    // ******************************************************************************************
    // Getters
    // ******************************************************************************************
    public SparkAbsoluteEncoder getTurningAbsoluteEncoder() {
        return pivotAbsoluteEncoder;
    }

    public Rotation2d getDesiredState() {
        return pivotDesiredState;
    }

    public Rotation2d getPosition() {
        return new Rotation2d(pivotAbsoluteEncoder.getPosition());
    }

    public boolean onPlusSoftwareLimit() {
        return pivotAbsoluteEncoder.getPosition() >= pivotMotor.configAccessor.softLimit.getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return pivotMotor.getEncoder().getPosition() <= pivotMotor.configAccessor.softLimit.getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return pivotMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return pivotMotor.getReverseLimitSwitch().isPressed();
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

// ******************************************************************************************

 

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }
}
