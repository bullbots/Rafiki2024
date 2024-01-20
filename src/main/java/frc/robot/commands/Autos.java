// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Drivetrain;
import frc.robot.commands.drivetrain.WaitForNavX;
import frc.robot.subsystems.DriveTrain;
import frc.team1891.common.trajectory.HolonomicTrajectoryCommandGenerator;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static frc.robot.utility.MirrorPoses.mirror;

public final class Autos {
  // static DriveTrain m_DriveTrain;
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // This chooser holds two commands.  The commands should be variants for red and blue.  The only difference should be using mirror() on robot poses.
  private static SendableChooser<Pair<Command, Command>> commandChooser = new SendableChooser<>();

  public static void load() {
    // TODO: PID values.
    HolonomicTrajectoryCommandGenerator.setRotationalPID(1,0,0);
    HolonomicTrajectoryCommandGenerator.setTranslationalPID(1,0,0);

    // m_DriveTrain = DriveTrain.getInstance();
  

    // commandChooser.setDefaultOption("Tribute to Swervy from Stephen", new Pair<Command,Command>(
    //   HolonomicTrajectoryCommandGenerator.generate(DriveTrain.getInstance(),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3, 3.2, new Rotation2d(Math.PI/4.)), new Rotation2d(Math.PI/2.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3.5, 3.5, new Rotation2d()), new Rotation2d(Math.PI/2.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3.9, 2.7, new Rotation2d(-Math.PI/1.7)), new Rotation2d(Math.PI/2.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3.3, 1.4, new Rotation2d(-2*Math.PI/3.)), new Rotation2d(Math.PI/3.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3.1, 1, new Rotation2d(-2*Math.PI/3.)), new Rotation2d(Math.PI/2.1)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(2.9, 1, new Rotation2d(2*Math.PI/3.)), new Rotation2d(Math.PI - Math.PI/2.1)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(2.7, 1.4, new Rotation2d(2*Math.PI/3.)), new Rotation2d(2*Math.PI/3.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(2.1, 2.7, new Rotation2d(Math.PI/1.7)), new Rotation2d(Math.PI/2.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(2.5, 3.5, new Rotation2d(0)), new Rotation2d(Math.PI/2.)),
    //     new Pair<Pose2d, Rotation2d>(new Pose2d(3, 3.2, new Rotation2d(-Math.PI/4.)), new Rotation2d(Math.PI/2.)))
    //   , HolonomicTrajectoryCommandGenerator.generate(DriveTrain.getInstance(),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 3.2, new Rotation2d(Math.PI/4.)), new Rotation2d(Math.PI/2.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3.5, 3.5, new Rotation2d()), new Rotation2d(Math.PI/2.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3.9, 2.7, new Rotation2d(-Math.PI/1.7)), new Rotation2d(Math.PI/2.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3.3, 1.4, new Rotation2d(-2*Math.PI/3.)), new Rotation2d(Math.PI/3.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3.1, 1, new Rotation2d(-2*Math.PI/3.)), new Rotation2d(Math.PI/2.1))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2.9, 1, new Rotation2d(2*Math.PI/3.)), new Rotation2d(Math.PI - Math.PI/2.1))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2.7, 1.4, new Rotation2d(2*Math.PI/3.)), new Rotation2d(2*Math.PI/3.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2.1, 2.7, new Rotation2d(Math.PI/1.7)), new Rotation2d(Math.PI/2.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2.5, 3.5, new Rotation2d(0)), new Rotation2d(Math.PI/2.))),
    //     mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 3.2, new Rotation2d(-Math.PI/4.)), new Rotation2d(Math.PI/2.))))
    //   ));
      // commandChooser.setDefaultOption("Place High and Stop", new Pair<Command,Command>(
      //   new AutoPlace(m_arm, m_grab, ArmState.TOP),
      //   new AutoPlace(m_arm, m_grab, ArmState.TOP)
      // ));
      // commandChooser.addOption("Place High and drive", new Pair<Command,Command>(
      //   new AutoPlace(m_arm, m_grab, ArmState.TOP).andThen(new RunCommand(()->m_DriveTrain.fromChassisSpeeds(new ChassisSpeeds(1,0,0)), m_DriveTrain).withTimeout(4.75)),
      //   new AutoPlace(m_arm, m_grab, ArmState.TOP).andThen(new RunCommand(()->m_DriveTrain.fromChassisSpeeds(new ChassisSpeeds(1,0,0)), m_DriveTrain).withTimeout(4.75))
      // ));
      // commandChooser.addOption("Taxi Only", new Pair<Command,Command>(
      //   new RunCommand(()->m_DriveTrain.fromChassisSpeeds(new ChassisSpeeds(1,0,0)), m_DriveTrain).withTimeout(4.75),
      //   new RunCommand(()->m_DriveTrain.fromChassisSpeeds(new ChassisSpeeds(1,0,0)), m_DriveTrain).withTimeout(4.75)
      // ));
      // commandChooser.addOption("Auto 1", new Pair<Command,Command>(
    //   Commands.print("RED - Auto 1"), 
    //   Commands.print("BLUE - Auto 1")
    // ));

    SmartDashboard.putData("Autonomous Chooser", commandChooser);
  }

  public static Command getSelected() {
    // SmartDashboard.putBoolean("Autonomous Finished", false);
    // return commandChooser.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
        return commandChooser.getSelected().getFirst();
    }
    return commandChooser.getSelected().getSecond();
  }
}
