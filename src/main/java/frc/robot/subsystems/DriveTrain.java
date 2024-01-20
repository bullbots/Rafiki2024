package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utility.MirrorPoses;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.SwerveModule;
import frc.team1891.common.hardware.SimNavX;
import frc.team1891.common.hardware.WPI_CANSparkMax;
import frc.team1891.common.logger.BullLogger;

public class DriveTrain extends SwerveDrivetrain {
  private static DriveTrain instance = null;
  public static DriveTrain getInstance() {
      if (instance == null) {
          instance = new DriveTrain();
      }
      return instance;
  }
  BullLogger CameraLoging = new BullLogger("Camera logging", false, false);
  
  private static final double TRANSLATIONAL_TOLERANCE = .02;
  private static final double ROTATIONAL_TOLERANCE = Math.toRadians(1);

  /**
   * Returns a new PID controller that can be used to control the position of the robot chassis in one axis.
   * @return a new {@link PIDController}
   */
  public static PIDController getTunedTranslationalPIDController() {
    PIDController controller = new PIDController(
      3, 0.5, 0
    );
    controller.setTolerance(TRANSLATIONAL_TOLERANCE);
    return controller;
  }

  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be in radians per second, representing the desired angular velocity.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedRotationalPIDController() {
    ProfiledPIDController controller = new ProfiledPIDController(
      3, 1, 0,
      new TrapezoidProfile.Constraints(
        10,
        100
      )
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    controller.setTolerance(ROTATIONAL_TOLERANCE);
    return controller;
  }
  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be between -1 and 1, and is meant to be fed to {@link Drivetrain#holonomicDrive(double, double, double, boolean)}.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedRotationalPIDControllerForHolonomicDrive() {
    ProfiledPIDController controller = new ProfiledPIDController(
      1.2, 0, 0,
      new TrapezoidProfile.Constraints(2.5, 5)
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    controller.setTolerance(ROTATIONAL_TOLERANCE);
    return controller;
  }
  
  public static double maxMetersPerSecond = 2;

  private static final ShuffleboardTab _shuffuleboardTab = Shuffleboard.getTab("Drivetrain");
  public static final DrivetrainConfig _config = new DrivetrainConfig(maxMetersPerSecond, .5, 7, 2, Units.inchesToMeters(2), 6.75, 2048);
  // public static final SimNavX _gyro = new SimNavX(SPI.Port.kMXP);
  public static final SimNavX _gyro = new SimNavX(SerialPort.Port.kUSB2);

  private static final TalonFX frontLeftDriveFalcon = new TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_CHANNEL);
  private static final WPI_CANSparkMax frontLeftSteerFalcon = new WPI_CANSparkMax(Constants.Drivetrain.FRONT_LEFT_STEER_CHANNEL, MotorType.kBrushless);
  private static final CANcoder frontLeftEncoder = new CANcoder(Constants.Drivetrain.FRONT_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule frontLeft = SwerveModule.createFromDriveFalconAndSteeringNeo(frontLeftDriveFalcon, frontLeftSteerFalcon, frontLeftEncoder, _config, .1,0,0,.8,0,0,1);
  
  private static final TalonFX frontRightDriveFalcon = new TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CHANNEL);
  private static final WPI_CANSparkMax frontRightSteerFalcon = new WPI_CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_STEER_CHANNEL, MotorType.kBrushless);
  private static final CANcoder frontRightEncoder = new CANcoder(Constants.Drivetrain.FRONT_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule frontRight = SwerveModule.createFromDriveFalconAndSteeringNeo(frontRightDriveFalcon, frontRightSteerFalcon, frontRightEncoder, _config, .1,0,0,.8,0,0,3);
  
  private static final TalonFX backLeftDriveFalcon = new TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_CHANNEL);
  private static final WPI_CANSparkMax backLeftSteerFalcon = new WPI_CANSparkMax(Constants.Drivetrain.BACK_LEFT_STEER_CHANNEL, MotorType.kBrushless);
  private static final CANcoder backLeftEncoder = new CANcoder(Constants.Drivetrain.BACK_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule backLeft = SwerveModule.createFromDriveFalconAndSteeringNeo(backLeftDriveFalcon, backLeftSteerFalcon, backLeftEncoder, _config, .1,0,0,.8,0,0,2);
  
  private static final TalonFX backRightDriveFalcon = new TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_CHANNEL);
  private static final WPI_CANSparkMax backRightSteerFalcon = new WPI_CANSparkMax(Constants.Drivetrain.BACK_RIGHT_STEER_CHANNEL, MotorType.kBrushless);
  private static final CANcoder backRightEncoder = new CANcoder(Constants.Drivetrain.BACK_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule backRight = SwerveModule.createFromDriveFalconAndSteeringNeo(backRightDriveFalcon, backRightSteerFalcon, backRightEncoder, _config, .1,0,0,.8,0,0,4);
  
  // logging
  BullLogger m_CurrentBLLogger;

  final DoubleSubscriber[] moduleSub = new DoubleSubscriber[4];
  double[] prev = new double[4];
  CANcoder[] encoders;
  
  double savedX;
  CircularBuffer<Double> estimatePoseBuffer;
  int visionPeriodicTicker = 0;
  int staleVisionTicker = 0;

  NetworkTable limeNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
  private DriveTrain(){
    super(_shuffuleboardTab, _config, .501652, .62865, _gyro, frontLeft, frontRight, backLeft, backRight);
    encoders = new CANcoder[]{frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder};
    
    gyro.setAngleAdjustment(0);
    configDriveMotor(frontLeftDriveFalcon);
    configDriveMotor(frontRightDriveFalcon);
    configDriveMotor(backLeftDriveFalcon);
    configDriveMotor(backRightDriveFalcon);
    configSteerMotor(frontLeftSteerFalcon);
    configSteerMotor(frontRightSteerFalcon);
    configSteerMotor(backLeftSteerFalcon);
    configSteerMotor(backRightSteerFalcon);
    configCANCoder(frontLeftEncoder, Constants.Drivetrain.FRONT_LEFT_ENCODER_OFFSET);
    configCANCoder(frontRightEncoder, Constants.Drivetrain.FRONT_RIGHT_ENCODER_OFFSET);
    configCANCoder(backLeftEncoder, Constants.Drivetrain.BACK_LEFT_ENCODER_OFFSET);
    configCANCoder(backRightEncoder, Constants.Drivetrain.BACK_RIGHT_ENCODER_OFFSET);

    // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    SmartDashboard.putNumber("module1Offset", 0);
    SmartDashboard.putNumber("module2Offset", 0);
    SmartDashboard.putNumber("module3Offset", 0);
    SmartDashboard.putNumber("module4Offset", 0);
    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("SmartDashboard");

    // subscribe to the topic in "datatable" called "Y"
    moduleSub[0] = datatable.getDoubleTopic("module1Offset").subscribe(0.0);
    moduleSub[1] = datatable.getDoubleTopic("module2Offset").subscribe(0.0);
    moduleSub[2] = datatable.getDoubleTopic("module3Offset").subscribe(0.0);
    moduleSub[3] = datatable.getDoubleTopic("module4Offset").subscribe(0.0);

    _gyro.reset();
    if (Robot.isRedAlliance()) {
      resetOdometry(MirrorPoses.mirror(getPose2d()));
    } else {
      resetOdometry();
    }

    configureShuffleboard();
    estimatePoseBuffer = new CircularBuffer<Double>(8);

    // set up logging
    m_CurrentBLLogger = new BullLogger("DriveTrain current Drive BL", true, false);
    //m_CurrentBLLogger.setLogType(BullLogger.LogType.DOUBLE);
    //m_CurrentBLLogger.setLogLevel(BullLogger.LogLevel.DEBUG);
  }

  private static void configDriveMotor(TalonFX driveMotor) {
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private static void configSteerMotor(CANSparkMax steerMotor) {
    steerMotor.restoreFactoryDefaults();
    steerMotor.setIdleMode(IdleMode.kBrake);
    steerMotor.setInverted(true);
  }

  private static void configCANCoder(CANcoder encoder, double encoderOffset) {
    //encoder.configFactoryDefault();
    //encoder.configMagnetOffset(encoderOffset);
  }

  @Override
  public void periodic(){
    super.periodic();
    for(int i = 0;i < moduleSub.length;i++){
      DoubleSubscriber sub = moduleSub[i];
      double value = sub.get();
      if (value != prev[i]) {
        prev[i] = value;  // save previous value
        //configCANCoder(encoders[i], value);
      }
    }
    SmartDashboard.putNumber("robot pitch angle", gyro.getPitch());
    SmartDashboard.putNumber("SwerveModule1 angle", frontRightEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("SwerveModule2 angle", frontLeftEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("SwerveModule3 angle", backLeftEncoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("SwerveModule4 angle", backRightEncoder.getAbsolutePosition().getValue());
    //System.out.println("Gyro pitch: " + gyro.getPitch());
    //double[] visionPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    if(Robot.isReal()){
      boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
      if(hasTarget){
        double[] visionPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        //estimatePoseBuffer.add(new Pose2d(visionPose[0], visionPose[1], new Rotation2d(Math.toRadians(visionPose[5]))));
        double sum = 0;
        for(int i = 0;i < estimatePoseBuffer.size();i++){
          if(estimatePoseBuffer.get(i) != Double.MAX_VALUE)
          sum += estimatePoseBuffer.get(i);
        }
        //count number of used entries in buffer
        int bufferSize = estimatePoseBuffer.size();
        for(int i = 0;i < estimatePoseBuffer.size();i++){
          if(estimatePoseBuffer.get(i) == Double.MAX_VALUE){
            bufferSize--;
          }
        }
        //get average
        double averageY = sum/bufferSize;
        if(Math.abs(averageY - visionPose[1]) < 1 || bufferSize == 0){
          estimatePoseBuffer.addFirst(visionPose[1]);
        }else{
          CameraLoging.logEntry(String.format("rejected estimate dif of: %f buffer size: %d%n",(averageY - visionPose[1]), bufferSize));
          
          System.out.printf("rejected estimate dif of: %f buffer size: %d%n",(averageY - visionPose[1]), bufferSize);
        }
        //if the the buffer has enough entries send it
        if(bufferSize >= 6){
          poseEstimator.addVisionMeasurement(new Pose2d(visionPose[0], visionPose[1], getPose2d().getRotation()) ,Timer.getFPGATimestamp());
        }
        staleVisionTicker++;
        //System.out.println("buffer size: " + bufferSize);
      }else{
        //clear buffer if there are no targets for more than 10 frames
        visionPeriodicTicker++;
        if(visionPeriodicTicker > 10){
          visionPeriodicTicker = 0;
          //reset the buffer by setting it to max value
          for(int i = 0;i < estimatePoseBuffer.size();i++){
            estimatePoseBuffer.addFirst(Double.MAX_VALUE);
          }
          //System.out.println("cleared vision buffer");
        }
      }
      if(staleVisionTicker > 10){
        staleVisionTicker = 0;
        for(int i = 0;i < estimatePoseBuffer.size();i++){
          estimatePoseBuffer.addFirst(Double.MAX_VALUE);
        }
        //System.out.println("cleared vision buffer");
      }
      
    }
    
    // log the current
    //m_CurrentBLLogger.logEntry(backLeftDriveFalcon.getSupplyCurrent(), BullLogger.LogLevel.DEBUG);
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds simSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getDesiredState(),
      frontRight.getDesiredState(),
      backLeft.getDesiredState(),
      backRight.getDesiredState()
    );

    _gyro.setRadians(_gyro.getRadians() - simSpeeds.omegaRadiansPerSecond * .02);
    Pose2d newPose = poseEstimator.getEstimatedPosition().plus(
      new Transform2d(
        new Translation2d(
          simSpeeds.vxMetersPerSecond * .02,
          simSpeeds.vyMetersPerSecond * .02
        ),
        new Rotation2d()
      )
    );

    poseEstimator.resetPosition(_gyro.getRotation2d(), getSwerveModulePositions(), newPose);
  }
  public void resetAngle(double deg){
    poseEstimator.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d(getPose2d().getTranslation(), Rotation2d.fromDegrees(deg)));
    System.out.println("Reseting Gyro");
  }
  public ScoringArea pickCubeScoringArea(){
    ScoringArea[] cubes = {ScoringArea.CUBE1,ScoringArea.CUBE2,ScoringArea.CUBE3};
    //iterate
    ScoringArea selected = cubes[0];
    double selectedDistance = Double.MAX_VALUE;
    for (ScoringArea scoringArea : cubes) {
      double distance = scoringArea.getPose2d().getTranslation().getDistance(getPose2d().getTranslation());
      if(distance < selectedDistance){
        selected = scoringArea;
        selectedDistance = distance;
      }
    }
    //return
    if(selectedDistance < 2){
      return selected;
    }else{
      return ScoringArea.NONE;
    }
  }
  public ScoringArea pickConeScoringArea(){
    ScoringArea[] cubes = {ScoringArea.CONE1,ScoringArea.CONE2,ScoringArea.CONE3,ScoringArea.CONE4,ScoringArea.CONE5,ScoringArea.CONE6};
    //iterate
    ScoringArea selected = cubes[0];
    double selectedDistance = Double.MAX_VALUE;
    for (ScoringArea scoringArea : cubes) {
      double distance = scoringArea.getPose2d().getTranslation().getDistance(getPose2d().getTranslation());
      if(distance < selectedDistance){
        selected = scoringArea;
        selectedDistance = distance;
      }
    }
    //return
    if(selectedDistance < 2){
      return selected;
    }else{
      return ScoringArea.NONE;
    }
  }
  public static enum ScoringArea{
    CUBE1(new Pose2d(2,1.0668,new Rotation2d())),
    CUBE2(new Pose2d(2,2.7432,new Rotation2d())),
    CUBE3(new Pose2d(2,4.4196,new Rotation2d())),

    CONE1(new Pose2d(1.9,0.508,new Rotation2d())),
    CONE2(new Pose2d(1.9,1.6256,new Rotation2d())),
    CONE3(new Pose2d(1.9,2.1844,new Rotation2d())),
    CONE4(new Pose2d(1.9,3.302,new Rotation2d())),
    CONE5(new Pose2d(1.9,3.8608,new Rotation2d())),
    CONE6(new Pose2d(1.9,4.9784,new Rotation2d())),
    //HUMANSTATION(new Pose2d(4,4.9784, Rotation2d.fromDegrees(180))),
    NONE(null);

    private final Pose2d pose;
    private ScoringArea(Pose2d pose){
      this.pose = pose;
    }
    public Pose2d getPose2d(){
      if (this.equals(NONE)) {
        return pose;
      }
      if(DriverStation.getAlliance().equals(Alliance.Red)){
        return MirrorPoses.mirror(pose);
      }
      return pose;
    }
  }
  public void moduleXConfiguration(){
    setSwerveModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      }
    );
  }
}
