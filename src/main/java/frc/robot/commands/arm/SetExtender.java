// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabbyArm;

public class SetExtender extends Command {
  private GrabbyArm m_arm;
  private int m_position;
  /** Creates a new setExtender. */
  public SetExtender(GrabbyArm arm, int pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_position = pose;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setExtendPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setNeo(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getExtendInt() - m_position) < GrabbyArm.extendTolerance;
  }
}
