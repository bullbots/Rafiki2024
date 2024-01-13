// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.RetractLiftExtend;
import frc.robot.commands.arm.SafeExtendingRetract;
import frc.robot.commands.arm.SafeFullLower;
import frc.robot.commands.drivetrain.ChargeStationFull;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Release;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabbyArm;
import frc.robot.subsystems.GrabbyArm.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAndChargeStation extends SequentialCommandGroup {
  /** Creates a new PlaceAndChargeStation. */
  public PlaceAndChargeStation(GrabbyArm arm, Grabber grab, Conveyer belt ,DriveTrain chassis) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SwerveModuleState[] states = {
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d()),
      new SwerveModuleState(0, new Rotation2d())};

    addCommands(new Grab(grab), 
    new RetractLiftExtend(arm, ArmState.TOP, belt),
    new Release(grab),Commands.waitSeconds(.6),
    new SafeExtendingRetract(arm).raceWith(new RunCommand(()->chassis.setSwerveModuleStates(states), chassis)), 
    new SafeFullLower(arm).alongWith(Commands.waitSeconds(1).andThen(
      new ChargeStationFull(chassis, DriveTrain._gyro,false)))
    );
  }
}
