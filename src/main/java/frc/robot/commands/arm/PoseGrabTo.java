// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.GrabbyArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoseGrabTo extends ParallelCommandGroup {
  /** Creates a new PoseGrabTo. */
  public PoseGrabTo(GrabbyArm arm, GrabbyArm.ArmState state) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetLifter(arm, state), Commands.sequence(Commands.waitSeconds(1), new SetExtender(arm, state.extend)));
    //new setExtender(arm, state.extend)
  }
}
