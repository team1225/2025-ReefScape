// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot_arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PivotArm;

public class ManuallyAdjustPivotArm extends Command {
  /** Creates a new JogArm. */
  private PivotArm m_arm;
  private CommandXboxController m_controller;

  public ManuallyAdjustPivotArm(PivotArm arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    double yval = -m_controller.getLeftY() / 4; // Left stick Y axis
   
    if (yval > 0 && !m_arm.isAtUpperLimit() || yval < 0 && !m_arm.isAtLowerLimit()) {

      double appliedVolts = yval * RobotController.getBatteryVoltage();

      m_arm.setVoltage(appliedVolts);

    } else {
      m_arm.setVoltage(0);
    }
 
    m_arm.setDesiredState(Rotation2d.fromRadians(m_arm.getPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
