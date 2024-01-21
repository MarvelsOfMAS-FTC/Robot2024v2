package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANEncoder;
// import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import frc.robot.Commands.IntakeCommands.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkFlex m_intake = new CANSparkFlex(Constants.Intake.IntakeMotor, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);;
  private  double current_speed = 0.2;
  public Intake() {
    setDefaultCommand(intakeDefault());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intake.set(current_speed);
  }
  public void setIntakeSpeed(Double speed){
    current_speed = speed;
  }
  public Command intake() {
    return run(
        () -> {
          setIntakeSpeed(1.0);
        });
  }
  public Command outtake() {
    return run(
        () -> {
          setIntakeSpeed(-0.3);
        });
  }
  public Command intakeDefault() {
    return run(
        () -> {
          setIntakeSpeed(0.0);
        });
  }
}