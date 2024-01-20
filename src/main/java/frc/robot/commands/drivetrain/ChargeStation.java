// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MatrixLEDs;
import frc.robot.utility.YamlLoader;
import frc.team1891.common.hardware.NavX;
import frc.team1891.common.hardware.SimNavX;

public class ChargeStation extends Command {
  /** Creates a new ChargeStation. */
  int phase = 0;
  private final DriveTrain driveTrain;
  // private final MatrixLEDs leds;
  private final SimNavX gyro;
  private final double tolerance = 4;
  private final double phase1Tol = -10;
  private final double phase2tol = -6;
  int phaseTimer = 0;
  public ChargeStation(DriveTrain drive, SimNavX gyro) {
    // this.leds = MatrixLEDs.getInstance();
    // addRequirements(drive, leds);
    addRequirements(drive);
    this.driveTrain = drive;
    this.gyro = gyro;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    phase = 0;
    //gyro calibrates automatically
    System.out.println("Driving up charge station");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!gyro.isConnected() || gyro.isCalibrating()) {
      System.out.println("stupy");
      // leds.setMat(YamlLoader.getImage("sad-face-frown-one-frame"));
      // leds.start();
      driveTrain.fromChassisSpeeds(new ChassisSpeeds(0,0,0));
      return;
    }
    if(phase == 0){
      driveTrain.fromChassisSpeeds(new ChassisSpeeds(1.8,0,0));
      if(gyro.getPitch() < phase1Tol){
        phase = 1;
      }
    }
    if(phase == 1){
      driveTrain.fromChassisSpeeds(new ChassisSpeeds(.64,0,0));
      if(gyro.getPitch() > phase2tol){
        phase = 2;
      }
    }
    // if(phase == 2){
      
    //   driveTrain.fromChassisSpeeds(new ChassisSpeeds(-.5,0,0));
    //   if((gyro.getPitch() > -tolerance)||(gyro.getPitch() < tolerance)){
    //     phase = 3;
    //   }
    // }
    System.out.println("phase: " + phase);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    System.out.println("Charge ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!gyro.isConnected()) {
      System.err.println("Charge Station: Gyro Disconnected!");
    }
    return (phase == 2 || phase == 3);
  }
}
