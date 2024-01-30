package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class StrafeAndMoveForward extends Command {
    double forward;
    double sideways;
    DriveTrain m_driveTrain;
    boolean x;
    
    public StrafeAndMoveForward(double sideSpeed, double forwardSpeed, DriveTrain drivetrain){
        addRequirements(drivetrain);
        forward = forwardSpeed;
        sideways = sideSpeed;
        m_driveTrain = drivetrain;
    }

    @Override
    public void execute(){
       m_driveTrain.holonomicDrive(forward, sideways, 0, true);
    }

   

    @Override
    public boolean isFinished(){
        return true;
    }
}
