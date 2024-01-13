package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{
    Double bottomSpeed = -.7;
    Double topSpeed = -.8;
    final DoubleSubscriber bottomSub;
    final DoubleSubscriber topSub;

    int bottomListen;
    int topListen;

    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
    
    private static DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,1);
    private static final CANSparkMax neo1 = new CANSparkMax(5, MotorType.kBrushless);
    private static final CANSparkMax neo2 = new CANSparkMax(6, MotorType.kBrushless);
    
    private Intake(){
        configMotor(neo1);
        configMotor(neo2);
        liftIntake();
        //shuffleboard listener
        SmartDashboard.putNumber("UpperIntakeSpeed", -.5);
        SmartDashboard.putNumber("LowerIntakeSpeed", -.3);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable datatable = inst.getTable("SmartDashboard");

        topSub = datatable.getDoubleTopic("UpperIntakeSpeed").subscribe(-.5);
        bottomSub = datatable.getDoubleTopic("LowerIntakeSpeed").subscribe(-.3);
    }
    
    public void setPower(Double topPow,Double bottomPow){
        neo1.set(topPow);
        neo2.set(bottomPow);
    }
    public void intakeMotors(){
        neo1.set(topSub.get());
        neo2.set(bottomSub.get());
        if(RobotContainer.scoringForCubes){
            neo1.set(topSub.get()/2);
            neo2.set(bottomSub.get()/1.5);
        }
    }

    public void reverseIntakeMotors() {
        neo1.set(1);
        neo2.set(1);
    }
    public void dropIntake(){
        intakePistons.set(Value.kForward);
    }
    public void liftIntake(){
        intakePistons.set(Value.kReverse);
    }
    private void configMotor(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
    }
}
