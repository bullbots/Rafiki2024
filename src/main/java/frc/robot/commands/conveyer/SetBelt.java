// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;

public class SetBelt extends Command {
  Conveyer m_belt;
  double m_power;
  /** Creates a new SetBelt. */
  public SetBelt(Conveyer belt, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(belt);
    m_belt = belt;
    m_power = power;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_belt.setPower(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_belt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
