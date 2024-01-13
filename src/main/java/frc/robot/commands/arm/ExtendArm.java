// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabbyArm;

public class ExtendArm extends Command {
  private GrabbyArm m_arm;
  private Double m_power;
  /** Creates a new ExtendArm. */
  public ExtendArm(GrabbyArm arm, Double pow) {
    addRequirements(arm);
    m_arm = arm;
    m_power = pow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setNeo(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setNeo(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
