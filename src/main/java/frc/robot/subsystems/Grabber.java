// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private static DoubleSolenoid grabbyPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH,2,3);
  private static Grabber instance;
  public static Value OPEN = Value.kReverse;
  public static Value CLOSED = Value.kForward;

  private Grabber() {}
  public static Grabber getInstance(){
    if(instance == null){
      instance = new Grabber();
    }
    return instance;
  }
  public void grab(){
    grabbyPistons.set(CLOSED);
  }
  public void release(){
    grabbyPistons.set(OPEN);
  }
  public void toggle(){
    if(isOpen()){
      grabbyPistons.set(CLOSED);
    }else{
      grabbyPistons.set(OPEN);
    }
  }
  public Value getState(){
    return grabbyPistons.get();
  }
  public boolean isOpen(){
    return grabbyPistons.get().equals(OPEN);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
