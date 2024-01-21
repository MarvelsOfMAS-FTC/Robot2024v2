package frc.robot.autos;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PoseEstimator;

public class exampleAuto extends SequentialCommandGroup {
    private Swerve swerve;
    private AutoBuilder autoBuilder;
    
    public exampleAuto(Swerve swerve){
        this.swerve = swerve;
        
        
    }
}