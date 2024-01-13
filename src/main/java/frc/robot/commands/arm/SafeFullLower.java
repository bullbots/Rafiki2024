// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GrabbyArm;
import frc.robot.subsystems.GrabbyArm.ArmState;

public class SafeFullLower extends Command {
  GrabbyArm m_arm;
  /** Creates a new SafeFullLower. */
  public SafeFullLower(GrabbyArm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setLogicalState(ArmState.RETRACTED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setFalcon(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setFalcon(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getLiftingSwitch();
  }
}
