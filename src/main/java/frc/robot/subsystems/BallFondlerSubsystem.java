package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class BallFondlerSubsystem extends SubsystemBase {

  private final SparkMax shootingMotor;
  private final SparkMax intakeMotor;
  private final SparkMax loadingMotor;
  private final SparkClosedLoopController shootingController;

  public BallFondlerSubsystem() {
    intakeMotor   = new SparkMax(DriveConstants.kIntakeMotorCanId,   MotorType.kBrushless);
    loadingMotor  = new SparkMax(DriveConstants.kLoadingMotorCanId,  MotorType.kBrushless);
    shootingMotor = new SparkMax(DriveConstants.kShootingMotorCanId, MotorType.kBrushless);

    shootingController = shootingMotor.getClosedLoopController();

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD)
        .velocityFF(ShooterConstants.kFF)
        .outputRange(-1, 1);
    shootingMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    stopAll();
  }

  // ===== SHOOTER =====

  public void shooterOn() {
    shootingController.setSetpoint(ShooterConstants.kTargetRPM, ControlType.kVelocity);
  }

  public void shooterOff() {
    shootingController.setSetpoint(0, ControlType.kVelocity);
  }

  public boolean isShooterAtSpeed() {
    return Math.abs(shootingMotor.getEncoder().getVelocity() - ShooterConstants.kTargetRPM)
        < ShooterConstants.kRPMTolerance;
  }

  // ===== INTAKE =====

  public void intakeForward() {
    intakeMotor.set(1);
    loadingMotor.set(1);
  }

  public void intakeReverse() {
    intakeMotor.set(-1);
    loadingMotor.set(-1);
  }

  // ===== SHOOT FEED =====
  // Runs shooter to target RPM; loadingMotor runs in reverse because it feeds
  // toward the shooter from the opposite side.
  public void shootFeed() {
    shootingController.setSetpoint(ShooterConstants.kTargetRPM, ControlType.kVelocity);
    loadingMotor.set(-1);
    intakeMotor.set(1);
  }

  // ===== STOP =====

  // Stops only intake/loading motors — leaves shooter running if it was already on.
  public void stopIntake() {
    intakeMotor.set(0.0);
    loadingMotor.set(0.0);
  }

  public void stopAll() {
    shootingController.setSetpoint(0, ControlType.kVelocity);
    intakeMotor.set(0.0);
    loadingMotor.set(0.0);
  }

  public double getShootingMotorRPM() {
    return shootingMotor.getEncoder().getVelocity();
  }
}
