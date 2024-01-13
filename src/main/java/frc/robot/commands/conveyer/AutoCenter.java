// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyer;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Grabber;

public class AutoCenter extends Command {
  Conveyer m_belt;
  Grabber m_grab;
  double targetPosition = 30;
  double position = 0;
  ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0, new Constraints(1, 1));
  /** Creates a new AutoCenter. */
  public AutoCenter(Conveyer belt, Grabber grab) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grab, belt);
    m_belt = belt;
    m_grab = grab;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //NetworkTableInstance.getDefault().getTable("").getEntry("").getDoubleArray(new double[6]);
    
    boolean isCube = false;
    m_belt.setPower(pidController.calculate(position, targetPosition));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      m_grab.grab();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPosition-position) < 2;
  }
}
