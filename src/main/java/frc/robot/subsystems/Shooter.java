package frc.robot.subsystems;


import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
//import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Shooter extends SubsystemBase {

  private final CANSparkFlex m_shooterMotor =
      new CANSparkFlex(Constants.ShooterConstants.SHOOTER_LEFT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex m_shooterMotor2 =
      new CANSparkFlex(Constants.ShooterConstants.SHOOTER_RIGHT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex m_intake =
      new CANSparkFlex(Constants.ShooterConstants.SHOOTER_RIGHT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

 private final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();
  private final SimpleMotorFeedforward m_shooterFeedforward = 
      new SimpleMotorFeedforward(
          ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  private final PIDController m_shooterFeedback = new PIDController(ShooterConstants.kP, 0.0, 0.0);

  /** The shooter subsystem for the robot. */
  public Shooter() {
    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor.restoreFactoryDefaults();

    m_shooterMotor.setSmartCurrentLimit(30);
    m_shooterMotor2.restoreFactoryDefaults();

    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor2.setIdleMode(IdleMode.kCoast);

    m_shooterMotor.burnFlash();
    m_shooterMotor2.burnFlash();

    m_shooterMotor2.follow(m_shooterMotor, true);

    m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
    ((Encoder) m_shooterEncoder).setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    ((PIDController) m_shooterEncoder).reset();
    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () ->
                    m_shooterMotor.set(
                        m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                            + m_shooterFeedback.calculate(
                              m_shooterEncoder.getVelocity() / 60.0, setpointRotationsPerSecond))),
            // Wait until the shooter has reached the setpoint, and then run the feeder
            waitUntil(m_shooterFeedback::atSetpoint).andThen(() -> m_intake.set(1)))
        .withName("Shoot");
  
  }
  public void setPowerPercentage(double powerPercentage, double intp){
    //if(powerPercentage > Constants.Universal.voltageMin && powerPercentage < Constants.Universal.voltageMax){

    }
  }