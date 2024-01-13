// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  private Intake m_intake;
  private Conveyer m_belt;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, Conveyer belt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_belt = belt;
    addRequirements(m_intake,m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.dropIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeMotors();
    m_belt.runForward();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPower(0.0,0.0);
    m_intake.liftIntake();
    m_belt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
