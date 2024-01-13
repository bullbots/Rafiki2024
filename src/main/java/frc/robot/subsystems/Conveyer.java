// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyer extends SubsystemBase {
  private static Conveyer instance = null;
  public static Conveyer getInstance() {
      if (instance == null) {
          instance = new Conveyer();
      }
      return instance;
  }

  public final WPI_VictorSPX beltMotor = new WPI_VictorSPX(10);
  /** Creates a new Conveyer. */
  private Conveyer() {}

  public void runForward(){
    beltMotor.set(-1.0);
  }
  public void setPower(double pow){
    beltMotor.set(-pow);
  }
  public void runBack(){
    beltMotor.set(1.0);
  }
  public void stop(){
    beltMotor.stopMotor();;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
