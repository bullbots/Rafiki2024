// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.team1891.common.hardware.SimNavX;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationFull extends SequentialCommandGroup {
  /** Creates a new ChargeStationFull. */
  public ChargeStationFull(DriveTrain driveTrain, SimNavX gyro, boolean preFaceWheels) {
    SwerveModuleState[] states = {
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d())};
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new RunCommand(()->driveTrain.setSwerveModuleStates(states), driveTrain).raceWith(Commands.waitSeconds(.6)), 
        new InstantCommand(), 
        ()->preFaceWheels),
      new ChargeStation(driveTrain, gyro), 
      Commands.waitSeconds(.7),
      new ChargeStationPID(driveTrain, gyro));
  }
}
