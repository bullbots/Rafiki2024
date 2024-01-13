// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MatrixLEDs;
import frc.robot.utility.YamlLoader;
import frc.team1891.common.hardware.SimNavX;

public class ChargeStationPID extends Command {
  ProfiledPIDController pidController = new ProfiledPIDController(.03,.0001, 0, new TrapezoidProfile.Constraints(
    .0002,
    0.0002
  ));
  /** Creates a new ChargeStationPID. */
  private final DriveTrain m_driveTrain;
  // private final MatrixLEDs leds;
  private final SimNavX m_gyro;
  public ChargeStationPID(DriveTrain driveTrain, SimNavX gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    // this.leds = MatrixLEDs.getInstance();
    m_gyro = gyro;
    // addRequirements(driveTrain, leds);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_gyro.isConnected() || m_gyro.isCalibrating()) {
      // leds.setMat(YamlLoader.getImage("sad-face-frown-one-frame"));
      // leds.start();
      m_driveTrain.fromChassisSpeeds(new ChassisSpeeds(0,0,0));
      return;
    }
    double forwardSpeed = pidController.calculate(m_gyro.getPitch(), 0);
    System.out.println("PID forward Speed: " + forwardSpeed);
    m_driveTrain.fromChassisSpeeds(new ChassisSpeeds(forwardSpeed,0,0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_gyro.isConnected()) {
      System.err.println("Charge Station PID: Gyro Disconnected!");
    }
    // return !m_gyro.isConnected() || m_gyro.isCalibrating();
    return false;
  }
}
