package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Wrist extends SubsystemBase {
  private final CANSparkFlex wristMotor =
      new CANSparkFlex(Constants.WristConstants.wristMotorID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  private boolean isClosedLoop;
  private TrapezoidProfile.State goal;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.WristConstants.pGain,
          Constants.WristConstants.iGain,
          Constants.WristConstants.dGain,
          new TrapezoidProfile.Constraints(
              Constants.WristConstants.kMaxVelocityRadPerSecond,
              Constants.WristConstants.kMaxAccelerationRadPerSecSquared));
  private final SlewRateLimiter limiter = new SlewRateLimiter(4.0);

  public Wrist() {
    wristMotor.restoreFactoryDefaults();

    wristMotor.setSmartCurrentLimit(Constants.WristConstants.currentLimit);
    wristMotor.setIdleMode(IdleMode.kBrake);

    wristEncoder.setPositionConversionFactor(Constants.WristConstants.positionConversionFactor);

    wristMotor.burnFlash();

    enable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getAngle());
    if (DriverStation.isTeleopEnabled()) {
      goal = new State(Constants.WristConstants.stowValue, 0.0);
    }
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(new State());
  }

  // Arm a = new Arm();
  // new Trigger(Arm.setGoal(new State(45, 0)))
 
  // public CommandBase setGoal(TrapezoidProfile.State state) {
  //   return runOnce(
  //           () -> {
  //             goal = state;
  //           })
  //       .until(controller::atGoal);
  // } quick fix is wierd
    public Command setGoal(TrapezoidProfile.State state) {
    return runOnce(
            () -> {
              goal = state;
            })
        .until(controller::atGoal);
  }

  public Command runArm(DoubleSupplier joystickValue) {
    return run(
        () -> {
          if (!isClosedLoopEnabled()) {
            if (joystickValue.getAsDouble() == 0) {
              wristMotor.set(limiter.calculate(Constants.WristConstants.teleopSpeed));
            } else if (joystickValue.getAsDouble() == 180) {
              wristMotor.set(limiter.calculate(-Constants.WristConstants.teleopSpeed));
            } else {
              wristMotor.set(limiter.calculate(0.0));
            }
          }
        });
  }

  public void controllerPeriodic() {
    if (isClosedLoopEnabled()) {
      if (goal != null) {
        controller.setGoal(
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    goal.position,
                    Constants.WristConstants.minAngle,
                    Constants.WristConstants.maxAngle),
                goal.velocity));
      } else {
        controller.setGoal(new State(Constants.WristConstants.stowValue, 0.0));
      }

      wristMotor.setVoltage(controller.calculate(getAngle(), controller.getGoal()));
    }
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public double getAngle() {
    return wristEncoder.getPosition();
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    // positionEntry.set(getAngle());
    // currGoalEntry.set(controller.getGoal().position);
    // goalEntry.set(controller.atGoal());
  }
}