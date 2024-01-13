// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grabber.DropAndLower;
import frc.robot.commands.grabber.Grab;
import frc.robot.commands.grabber.Release;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabbyArm;
import frc.robot.subsystems.GrabbyArm.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlace extends SequentialCommandGroup {
  /** Creates a new AutoPlace. */
  public AutoPlace(GrabbyArm arm, Grabber grab, ArmState state) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetExtender(arm, 4), new Grab(grab), Commands.waitSeconds(.5),new SafeExtendingRetract(arm),new RetractLiftExtend(arm, state, Conveyer.getInstance()), new SetLifter(arm, state.lift-30000) ,new DropAndLower(grab, arm));
  }
  //new RetractLiftExtend(m_arm, GrabbyArm.ArmState.RETRACTED).andThen(new Grab(m_grab).andThen(new RetractLiftExtend(m_arm, GrabbyArm.ArmState.TOP).andThen(new DropAndLower(m_grab, m_arm)))),
}
