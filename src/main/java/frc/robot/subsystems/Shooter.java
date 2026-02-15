package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Shooter extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.LEFT_MOTOR_ID, "lil clanker");
  private final TalonFX right = new TalonFX(Constants.RIGHT_MOTOR_ID, "lil clanker");

  // Phoenix 6 request object (reuse it; don’t new it every loop)
  private final VelocityVoltage leftVelReq = new VelocityVoltage(0);

  // Right follows left. Set invert = true/false depending on your gearbox mounting.
  
private final Follower rightFollower =
    new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
     // Tune these from SysId or manual tuning
  // Units here are in VOLTS:
  // kS: volts, kV: volts per (rad/s), kA: volts per (rad/s^2)
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(0.252, 0.0208, 0.10);

  private boolean shooterEnabled = false;
  private double targetRPM = 0.0;

  public Shooter() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimit = 120;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    // Flywheels usually feel better on COAST, not BRAKE.
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Optional: set some starting slot gains on the Talon itself (closed-loop velocity P/I/D).
    // These are NOT the same as your WPILib PID values.
    cfg.Slot0.kP = 0.05;  // start small, tune
    cfg.Slot0.kI = 0.00;
    cfg.Slot0.kD = 0.00;

    left.getConfigurator().apply(cfg);
    right.getConfigurator().apply(cfg);

    // Make the right motor follow left
    right.setControl(rightFollower);
  }

  // Call this to spin up to a speed
  public void setShooterRPM(double rpm) {
    targetRPM = rpm;
    // shooterEnabled = rpm > 0.0;
  }

  public void shoot1(double rpm) {
    System.out.println("shooting");
    setShooterRPM(rpm); 
    shooterEnabled = true;
  }

  public void off() {
    shooterEnabled = false;
    targetRPM = 0.0;
    left.set(0);
    right.set(0);
  }

  @Override
  public void periodic() {
    double leftRPS = left.getVelocity().getValueAsDouble();     // motor rotations/sec
    double leftRPM = leftRPS * 60.0;

    if (!shooterEnabled) {
      SmartDashboard.putNumber("Shooter Left RPM", leftRPM);
      return;
    }

    // Convert target RPM -> target RPS (Phoenix 6 uses RPS)
    double targetRPS = targetRPM / 60.0;

    // Feedforward usually wants rad/s. Convert motor RPS -> rad/s
    double targetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);

    // If you don’t have accel, use the 1-arg calculate (kS + kV*w)
    double ffVolts = ff.calculate(targetRadPerSec);

    // Command velocity with arbitrary feedforward voltage
    left.setControl(
        leftVelReq
            .withVelocity(targetRPS)     // RPS
            .withFeedForward(ffVolts)    // volts
    );

    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putNumber("Shooter Left RPM", leftRPM);
    SmartDashboard.putNumber("Shooter FF Volts", ffVolts);

  }
}
