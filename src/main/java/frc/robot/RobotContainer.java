// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.NorthUntilInterupt;
import frc.robot.commands.leds.RunMatrixImageCommand;
import frc.robot.commands.leds.RunMatrixVideoCommand;
import frc.robot.commands.leds.RunMatrixVideoCommand.RunType;
import frc.robot.subsystems.*;
import frc.robot.utility.YamlLoader;
import frc.team1891.common.control.AxisTrigger;
import frc.team1891.common.control.JoystickRotation2d;
import frc.team1891.common.control.POVTrigger;
import frc.team1891.common.control.POVTrigger.POV;
import frc.team1891.common.logger.BullLogger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveTrain m_DriveTrain = DriveTrain.getInstance();
  // public final Intake m_intake = Intake.getInstance();
  // public final GrabbyArm m_arm = GrabbyArm.getInstance();
  // public final Conveyer m_conveyer = Conveyer.getInstance();
  // public final Grabber m_Grabber = Grabber.getInstance();
  // public final PneumaticHub m_Hub = new PneumaticHub();

  // public final MatrixLEDs m_LEDSystem = MatrixLEDs.getInstance();

  // private static final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  // This is big bad, but I'm lazy -stephen
  // public static Compressor getCompressor() {
  //   return m_compressor;
  // }
    
  public static boolean scoringForCubes = false;
  public static double setAngle = 0;
  //private final AbsoluteAngleJoystickDrive m_absoluteDrive = new AbsoluteAngleJoystickDrive(m_DriveTrain, null, null, null);

  BullLogger logger = new BullLogger("Main log", false, false);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // public static final XboxController m_driverController =
  //   new XboxController(OperatorConstants.kDriverControllerPort) {
  //     public double getRawAxis(int axis) {
  //       return MathUtil.applyDeadband(super.getRawAxis(axis), .15);
  //     };
  // };
  public static final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick m_coBox = new Joystick(OperatorConstants.kDriverControllerPort + 1);

  // This is big bad, but I'm lazy -stephen
  public static XboxController getController() {
    return m_driverController;
  }
    
    
  private Limelight limelight = new Limelight();
  private JoystickRotation2d rightStickRotation2d = new JoystickRotation2d(() -> MathUtil.applyDeadband(-m_driverController.getRightY(), JoystickDrive.DEADBAND), () -> MathUtil.applyDeadband(-m_driverController.getRightX(), JoystickDrive.DEADBAND));
  private JoystickButton m_intakeButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private JoystickButton m_resetGyroJoystickButton = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
  private JoystickButton m_FaceForward = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
  private JoystickButton m_alignToPlaceButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  private JoystickButton m_moveLow = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private JoystickButton m_moveMed = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private JoystickButton m_moveHigh = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private JoystickButton m_drop = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  private JoystickButton m_xwheels = new JoystickButton(m_driverController, XboxController.Button.kStart.value);

  private AxisTrigger m_leftrightTrigger = new AxisTrigger(m_driverController, XboxController.Axis.kLeftX.value);
  private AxisTrigger m_forwardBack = new AxisTrigger(m_driverController, XboxController.Axis.kLeftY.value);
  private AxisTrigger m_rightStickTrig = new AxisTrigger(m_driverController, XboxController.Axis.kRightX.value,.13);
  private AxisTrigger m_leftTrigger = new AxisTrigger(m_driverController, 2);
  private AxisTrigger m_rightTrigger = new AxisTrigger(m_driverController, 3);
  
  private POVTrigger m_POVNorth = new POVTrigger(m_driverController, POV.NORTH);
  private POVTrigger m_POVEast = new POVTrigger(m_driverController, POV.EAST);
  private POVTrigger m_Retract = new POVTrigger(m_driverController, POV.SOUTH);
  private POVTrigger m_POVWest = new POVTrigger(m_driverController, POV.WEST);

  private Trigger m_anyArmMovement = m_moveLow.or(m_moveMed).or(m_moveHigh);
  //Co driver board
  private JoystickButton m_backDriveIntake = new JoystickButton(m_coBox, 2);
  public JoystickButton m_ConeCubeMode = new JoystickButton(m_coBox, 5);
  public JoystickButton m_ArmUpBtn = new JoystickButton(m_coBox, 1);
  private JoystickButton m_ArmDownBtn = new JoystickButton(m_coBox, 12);
  private JoystickButton m_ArmInBtn = new JoystickButton(m_coBox, 8);
  private JoystickButton m_ArmOutBtn = new JoystickButton(m_coBox, 4);
  private JoystickButton m_zeroArm = new JoystickButton(m_coBox, 10);
  private JoystickButton m_BelftForward = new JoystickButton(m_coBox, 9);
  private JoystickButton m_BeltBack = new JoystickButton(m_coBox, 11);
  private JoystickButton m_CoGrab = new JoystickButton(m_coBox, 3);
  // belt back and forth
  // grab and drop
  private RunMatrixVideoCommand offlineCommand;
  private RunMatrixVideoCommand firstLogo;
  private RunMatrixVideoCommand lowBattery;

    // DEMO BUTTON
    String number_image = "";
  //private JoystickButton m_autopilotToCommunity = new JoystickButton(m_coBox, 1);
  public void periodic() {
    limelight.getAll();
    // SmartDashboard.putNumber("Tank Pressure", m_Hub.getPressure(0));
    // int seconds = (int)DriverStation.getMatchTime();
    // if(seconds <= 30 && seconds > 0 && DriverStation.isTeleop()){
    //   if(number_image != String.format("number_%d.png", seconds)){
    // //    new RunMatrixImageCommand(m_LEDSystem, YamlLoader.getImage(String.format("number_%d.png", seconds))).schedule();
    //     number_image = String.format("number_%d.png", seconds);
    //   }
      
    // }else if(seconds <= 0 && !firstLogo.isScheduled() && DriverStation.isEnabled() && DriverStation.isDSAttached() && !CommandScheduler.getInstance().isScheduled(lowBattery)) {
    //   firstLogo.schedule();
    // }
    //System.out.println("matchTime: " + seconds);

    // if(!DriverStation.isDSAttached()){
    //   if(!CommandScheduler.getInstance().isScheduled(offlineCommand)){
    //     offlineCommand.schedule();
    //   }
    // } else if (RobotController.getBatteryVoltage() < 8) {
    //   if (!CommandScheduler.getInstance().isScheduled(lowBattery)) {
    //     lowBattery.schedule();
    //   }
    // } else{
    //   if(CommandScheduler.getInstance().isScheduled(offlineCommand)){
    //     firstLogo.schedule();
    //   }
    // }
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Autos.load();

    // offlineCommand = new RunMatrixVideoCommand(m_LEDSystem, YamlLoader.getVideo("offline"), 10, RunType.CONTINUOUS);
    // firstLogo = new RunMatrixVideoCommand(m_LEDSystem, YamlLoader.getVideo("first_pixels"),5,RunType.CONTINUOUS);
    // lowBattery = new RunMatrixVideoCommand(m_LEDSystem, YamlLoader.getVideo(("batterylow")), 20, RunType.CONTINUOUS);
    //offlineBack = new RunMatrixVideoCommand(m_LEDSystemBack, YamlLoader.getVideo("offline"), 10, RunType.CONTINUOUS);
    //m_absoluteAngleDrive = new AbsoluteAngleJoystickDrive(m_DriveTrain, m_driverController.getLeftY(), null, null);
    // Configure the trigger bindings
    configureBindings();
    
    m_DriveTrain.setDefaultCommand(
        new RunCommand(
            () -> {
                m_driverController.setRumble(RumbleType.kBothRumble, 0.0);
                final double DEADBAND = .15;
                double x = m_driverController.getLeftX();
                double y = m_driverController.getLeftY();
                double z = m_driverController.getRightX();
                if (Math.abs(x) > DEADBAND) {
                  y = MathUtil.applyDeadband(y, DEADBAND*.6);
                } else {
                  y = MathUtil.applyDeadband(y, DEADBAND);
                }
                 if (Math.abs(y) > DEADBAND) {
                  x = MathUtil.applyDeadband(x, DEADBAND*.6);
                } else {
                  x = MathUtil.applyDeadband(x, DEADBAND);
                }
                z = MathUtil.applyDeadband(z, DEADBAND);

                

                m_DriveTrain.holonomicDrive(
                    // I may be wrong, but I think all of these should be negative (not z), 
                    // since forward y is negative, and on the x axes left is 
                    // positive for the robot strafing and twisting.
                    // It checks out in the simulator..
                    //Z should not be negative, the simulator has reversed turning for some reason
                    -y,
                    -x,
                    z,
                    true);
            }, m_DriveTrain));
    // if(DriverStation.isDSAttached()) {
    //   firstLogo.schedule();
    // }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //m_intakeButton.whileTrue(new IntakeCommand(m_intake, m_conveyer));
    //m_intakeButton.onFalse(new Release(m_Grabber).andThen(Commands.waitSeconds(.4)).andThen(new FullRetract(m_arm)));
    m_FaceForward.onTrue(new NorthUntilInterupt(m_DriveTrain,()-> m_driverController.getLeftX(),() -> m_driverController.getLeftY(),() -> m_rightStickTrig.getAsBoolean()));
    
    //m_alignToPlaceButton.onTrue(new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickConeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()));
      //m_alignToPlaceButton.onTrue(new ConditionalCommand(
      //new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickCubeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()),
      //new DriveToPose(m_DriveTrain, ()-> m_DriveTrain.pickConeScoringArea().getPose2d(), () -> m_leftrightTrigger.or(m_forwardBack.or(m_rightStickTrig)).getAsBoolean()),
      //()-> scoringForCubes));

    m_POVNorth.onTrue(
      new InstantCommand(
            () ->
            {
              if(Robot.isRedAlliance()){
                m_DriveTrain.resetAngle(180);
              }else{
                m_DriveTrain.resetAngle(0);
              }
            },
                m_DriveTrain));

    //m_moveMed.onTrue(new PoseGrabTo(m_arm, ArmState.MID));
    //m_BelftForward.whileTrue(new BeltForward(m_conveyer));
    //m_BeltBack.whileTrue(new BeltBack(m_conveyer));
    //m_leftTrigger.whileTrue(new LiftArm(m_arm, 0.2));
    //m_rightTrigger.whileTrue(new LiftArm(m_arm, -0.2));

    //m_moveLow.onTrue(new RetractLiftExtend(m_arm, GrabbyArm.ArmState.LOW, m_conveyer));
    //m_moveMed.onTrue(new RetractLiftExtend(m_arm, GrabbyArm.ArmState.MID, m_conveyer));
    //m_moveHigh.onTrue(new RetractLiftExtend(m_arm, GrabbyArm.ArmState.TOP, m_conveyer));
    //m_rightTrigger.onTrue(new RetractLiftExtend(m_arm, GrabbyArm.ArmState.HUMAN, m_conveyer));
    //m_Retract.onTrue(new FullRetract(m_arm));
    m_xwheels.onTrue(
      new RunCommand(
        ()->{
          m_DriveTrain.moduleXConfiguration();
        }, m_DriveTrain)
        .withTimeout(2)
    );
    m_CoGrab.onTrue(new ButtonPress("pressed"));
    m_CoGrab.onFalse(new ButtonPress("not pressed"));
    //m_backDriveIntake.whileTrue(new ReverseIntake());
    
    //m_backDriveIntake.whileTrue(new Command(() -> m_intake.setPower(.5, .5)));
    //m_ArmUpBtn.whileTrue(new LiftArm(m_arm, 0.25));
    //m_ArmDownBtn.whileTrue(new LiftArm(m_arm, -0.25));
    //m_ArmOutBtn.whileTrue(new ExtendArm(m_arm, .3));
    //m_ArmInBtn.whileTrue(new ExtendArm(m_arm, -.3));
    //m_zeroArm.onTrue(new InstantCommand(()-> m_arm.zeroAll()));
    //m_ConeCubeMode.onTrue(new InstantCommand(() -> coneMode()).andThen(new RunMatrixImageCommand(m_LEDSystem, YamlLoader.getImage("cone256"))));
    //m_ConeCubeMode.onFalse(new InstantCommand(() -> cubeMode()).andThen(new RunMatrixImageCommand(m_LEDSystem, YamlLoader.getImage("cube256"))));
    //m_CoGrab.onTrue(new ToggleGrab(m_Grabber));

    //m_anyArmMovement.onTrue(new InstantCommand(() -> m_compressor.disable()));
    //m_anyArmMovement.onFalse(new InstantCommand(() -> m_compressor.enableDigital()));

    //if the arm is out open claw and retract otherwise just toggle the claw
    //m_drop.onTrue(new ConditionalCommand(
    //   new ToggleGrab(m_Grabber),
    //   new DropAndLower(m_Grabber, m_arm),
    //   ()-> m_arm.isRetracted()
    // ));
    
    SmartDashboard.putData("set to 90", new InstantCommand(){
      @Override
      public void initialize() {
        setAngle = 90;
      }
    });
    SmartDashboard.putData("set to 0", new InstantCommand(){
      @Override
      public void initialize() {
        setAngle = 30;
      }
    });
    // SmartDashboard.putData("reset Gyro", new InstantCommand(
    //   () ->
    //       {m_DriveTrain.resetGyro();
    //       System.out.print("reset Gyro");},
    //       m_DriveTrain));

    //m_autopilotToCommunity.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toCommunityZone(m_DriveTrain));
    // SmartDashboard.putData("Off", new RunMatrixImageCommand(m_LEDSystem, YamlLoader.getImage("Off")));
    // SmartDashboard.putData("Offline", new RunMatrixVideoCommand(m_LEDSystem,
    //         YamlLoader.getVideo("offline"),
    //         10,
    //         RunMatrixVideoCommand.RunType.CONTINUOUS));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void coneMode(){
    scoringForCubes = false;
  }
  private void cubeMode(){
    scoringForCubes = true;
  }
  public Command getAutonomousCommand() {
    return Autos.getSelected();
  }
}
