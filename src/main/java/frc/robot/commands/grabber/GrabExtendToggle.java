// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SafeExtendingRetract;
import frc.robot.commands.arm.SetExtender;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabbyArm;
import frc.robot.subsystems.GrabbyArm.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabExtendToggle extends SequentialCommandGroup {
  /** Creates a new GrabExtendToggle. */
  public GrabExtendToggle(Grabber grab, GrabbyArm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ToggleGrab(grab), Commands.waitSeconds(.4), new ConditionalCommand(new SetExtender(arm, ArmState.RETRACTED.extend), new SafeExtendingRetract(arm) , ()->grab.isOpen()));
  }
}
