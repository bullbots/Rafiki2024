// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.team1891.common.logger.BullLogger;

public class GrabbyArm extends SubsystemBase {
  public static int liftTolerance = 5000;
  public static int extendTolerance = 2;
  private static GrabbyArm instance = null;
  private static SparkMaxLimitSwitch extendingReverseLimitSwitch;

  BullLogger LimitStopLog = new BullLogger("Limit Switch Log", false, false);
  boolean loweringStopImpuls = false;
  boolean retractingStopImpuls = false;
  public static GrabbyArm getInstance() {
      if (instance == null) {
          instance = new GrabbyArm();
      }
      return instance;
  }

  public final WPI_TalonFX liftFalcon = new WPI_TalonFX(7);
  public final CANSparkMax extendingNeo = new CANSparkMax(8, MotorType.kBrushless);
  private ArmState m_desiredState = ArmState.RETRACTED;
  protected ProfiledPIDController extendingPID;
  protected ProfiledPIDController liftingPID;
  /** Creates a new GrabbyArm. */
  private GrabbyArm() {
    
    configFalcon(liftFalcon);
    configNeo(extendingNeo);
    extendingPID = new ProfiledPIDController(0.03, 0, 0, new TrapezoidProfile.Constraints(
      // Max angular velocity and acceleration of the module
      9000000,
      9000000
    ));
    liftingPID = new ProfiledPIDController(0.000002, 0.00001, 0,  new TrapezoidProfile.Constraints(
      // Max angular velocity and acceleration of the module
      110000,
      110000
    ));
    extendingReverseLimitSwitch = extendingNeo.getReverseLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen);
  }
  public void setFalcon(Double pow){
    liftFalcon.set(pow);
  }
  public void setNeo(Double pow){
    extendingNeo.set(pow);
  }
  public void setState(ArmState state) {
    m_desiredState = state;
    if (isLiftAtDesiredState()) {
      setExtendPosition(state.extend);
      setLiftPosition(state.lift);
    } else {
      setExtendPosition(ArmState.RETRACTED.extend);
      if (Math.abs(getExtendInt() - ArmState.RETRACTED.extend) < extendTolerance) {
        setLiftPosition(state.lift);
      }
    }
    
  }
  public void zeroAll(){
    liftFalcon.setSelectedSensorPosition(0);
    extendingNeo.getEncoder().setPosition(0);
  }
  public void setLiftPosition(int position){
    double pow = liftingPID.calculate(liftFalcon.getSelectedSensorPosition(), position);
    //System.out.printf("lifitng PID: %f", pow);
    liftFalcon.set(pow);
  }
  public void setExtendPosition(int position){
    double pow = extendingPID.calculate(extendingNeo.getEncoder().getPosition(), position);
    extendingNeo.set(pow);
    //System.out.println("extend power: " + pow);
  }
  private void configFalcon(WPI_TalonFX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.config_kP(0, 0.8);
    motor.config_kI(0, 0);
    motor.config_kP(0, 0);

    
  }
  private void configNeo(CANSparkMax motor){
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
  }
  public ArmState getState(){
    return m_desiredState;
  }
  public boolean atDesiredState() {
    return isLiftAtDesiredState() && isExtendAtDesiredState();
  }
  public int getLifterInt(){
    return (int)liftFalcon.getSelectedSensorPosition();
  }
  public boolean isLiftAtDesiredState() {
    return (Math.abs(getLifterInt() - m_desiredState.lift) < liftTolerance);
  }
  public int getExtendInt(){
    return (int)extendingNeo.getEncoder().getPosition();
  }
  public boolean isExtendAtDesiredState() {
    return (Math.abs(getExtendInt() - m_desiredState.extend) < extendTolerance);
  }
  public void resetPID(){
    liftingPID.reset(liftFalcon.getSelectedSensorPosition());
    extendingPID.reset(extendingNeo.getEncoder().getPosition());
  }
  public void setLogicalState(ArmState state){
    m_desiredState = state;
  }
  public boolean isRetracted(){
    if(m_desiredState == ArmState.RETRACTED){
      System.out.println("is retracted");
    }
    return m_desiredState == ArmState.RETRACTED;
  }
  public boolean getExteningSwitch(){
    return extendingReverseLimitSwitch.isPressed();
  }
  public boolean getLiftingSwitch(){
    return liftFalcon.isRevLimitSwitchClosed() == 1;
  }
  public void resetExtendingPose(){
    extendingNeo.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lift Position Ticks", liftFalcon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Extend Position rots", extendingNeo.getEncoder().getPosition());
    SmartDashboard.putNumber("Lifter Supply", liftFalcon.getSupplyCurrent());
    SmartDashboard.putNumber("Lifter Stator", liftFalcon.getStatorCurrent());
    SmartDashboard.putNumber("Extending Current", extendingNeo.getOutputCurrent());

    if(extendingReverseLimitSwitch.isPressed() && !DriverStation.isAutonomousEnabled()){
      double margin = extendingNeo.getEncoder().getPosition();
      extendingNeo.getEncoder().setPosition(0);
      if(retractingStopImpuls){
        LimitStopLog.logEntry("Exteding Encoder reset with an error of: " + margin);
        RobotContainer.m_driverController.setRumble(RumbleType.kLeftRumble, .8);
      }else{
        RobotContainer.m_driverController.setRumble(RumbleType.kLeftRumble, 0);
      }
    }else{
      retractingStopImpuls = false;
    }
    if(liftFalcon.isRevLimitSwitchClosed() == 1){
      double margin = liftFalcon.getSelectedSensorPosition();
      liftFalcon.setSelectedSensorPosition(0);
      if(loweringStopImpuls){
        LimitStopLog.logEntry("Lifting Encoder reset with an error of: " + margin);
        RobotContainer.m_driverController.setRumble(RumbleType.kRightRumble, .8);
      }else{
        RobotContainer.m_driverController.setRumble(RumbleType.kRightRumble, 0);
      }
    }else{
      retractingStopImpuls = false;
    }
  }
  public enum ArmState{
    RETRACTED(0,0),
    INTAKEING(30000,0),
    LOW(90000,12),
    MID(150000,22),
    TOP(190000,96),
    HUMAN(170000,60);

    public final int lift;
    public final int extend;
    private ArmState(int liftPose, int extendPose){
      lift = liftPose;
      extend = extendPose;
    }
  }
}
