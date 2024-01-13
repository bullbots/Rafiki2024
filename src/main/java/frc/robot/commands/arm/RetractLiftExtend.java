// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.conveyer.BeltForward;
import frc.robot.commands.conveyer.SetBelt;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GrabbyArm;
import frc.team1891.common.drivetrains.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractLiftExtend extends SequentialCommandGroup {
  /** Creates a new RetractLiftExtend. */
  public RetractLiftExtend(GrabbyArm arm, GrabbyArm.ArmState state, Conveyer belt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetExtender(arm, 0),
      new ConditionalCommand(
        new InstantCommand(()-> DriveTrain.getInstance().setInputScale(1)), 
        new InstantCommand(()-> DriveTrain.getInstance().setInputScale(.5)), 
        ()->(state == GrabbyArm.ArmState.RETRACTED)),
      new SetLifter(arm, state).raceWith(new SetBelt(belt, .3)), 
      new SetExtender(arm, state.extend));
  }
}
