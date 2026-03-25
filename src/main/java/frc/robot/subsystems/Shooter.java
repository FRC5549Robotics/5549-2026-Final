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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.Constants;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


public class Shooter extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.LEFT_MOTOR_ID, "lil clanker");
  private final TalonFX right = new TalonFX(Constants.RIGHT_MOTOR_ID, "lil clanker");
  private final TalonFX middle = new TalonFX(Constants.MIDDLE_MOTOR_ID, "lil clanker");

  // Phoenix 6 request object (reuse it; don’t new it every loop)
  private final VelocityVoltage leftVelReq = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut charVoltage = new VoltageOut(0);

  // Right follows left. Set invert = true/false depending on your gearbox mounting.
  
  private final Follower rightFollower = new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
  private final Follower middleFollower = new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
     // Tune these from SysId or manual tuning
  // Units here are in VOLTS:
  // kS: volts, kV: volts per (rad/s), kA: volts per (rad/s^2)
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.28, 0.122);

  private boolean shooterEnabled = false;
  private double targetRPM = 0.0;

  private final LinearFilter rpmFilter = LinearFilter.movingAverage(8);

  public Shooter() { //shooter constructor
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimit = 80; //used to be 120
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30; //used to be 50
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Flywheels usually feel better on COAST, not BRAKE.
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Optional: set some starting slot gains on the Talon itself (closed-loop velocity P/I/D).
    // These are NOT the same as your WPILib PID values.
    cfg.Slot0.kP = 0.6;  // start small, tune
    cfg.Slot0.kI = 0.00;
    cfg.Slot0.kD = 0.00;


    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.00; //0.00 seconds to reach max velocity

    left.getConfigurator().apply(cfg);
    right.getConfigurator().apply(cfg);

    // Make the right motor follow left
    right.setControl(rightFollower);
    middle.setControl(middleFollower);
  }

  private double getVelocityRPS() {
    return left.getVelocity().getValueAsDouble();
  }

  // Call this to spin up to a speed
  public void setShooterRPM(double rpm) {
    targetRPM = rpm;
    // shooterEnabled = rpm > 0.0;
  }

  public void shoot(double rpm) {
    //System.out.println("shooting");
    setShooterRPM(rpm); 
    shooterEnabled = true;
  }

  public void off() {
    shooterEnabled = false;
    targetRPM = 0.0;
    left.stopMotor();
  }

  public boolean atSpeed() {
    double currentRPM = left.getVelocity().getValueAsDouble() * 60;
    if (targetRPM == 0.0) {
      return false;
    }
    return Math.abs(currentRPM - targetRPM) < 50; //Is flywheel rpm within tolerance?
  }

  @Override
  public void periodic() {
    double leftRPS = left.getVelocity().getValueAsDouble();     // motor rotations/sec
    double leftRPM = leftRPS * 60.0;
    double rpmFiltered = rpmFilter.calculate(leftRPM);
    double targetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);     // Feedforward usually wants rad/s. Convert motor RPS -> rad/s
    double targetRPS = targetRPM / 60.0; // Convert target RPM -> target RPS (Phoenix 6 uses RPS)
    double ffVolts = ff.calculate(targetRPS);
    double shooterVolts = left.getMotorVoltage().getValueAsDouble();

    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putNumber("Shooter Left RPM", leftRPM);
    SmartDashboard.putNumber("RPS", leftRPS);
    SmartDashboard.putNumber("Shooter Volts", shooterVolts);
    SmartDashboard.putNumber("Shooter FF Volts", ffVolts);
    SmartDashboard.putNumber("Shooter RPM Filtered", rpmFiltered);

    // Command velocity with arbitrary feedforward voltage

    if (shooterEnabled && targetRPM > 0.0) {
        left.setControl(
          leftVelReq
            .withVelocity(targetRPS)     // RPS
            .withFeedForward(ffVolts)    // volts
        );
    } else {
      left.stopMotor();
    }

  }

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.of(0.5).per(Second),
      Volts.of(6),
      null,
      (state) -> {}
    ),
    new SysIdRoutine.Mechanism(
      (Voltage volts) -> {
        left.setVoltage(volts.in(Volts));
        right.setControl(new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));
        middle.setControl(new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));
      },
      log -> {
        log.motor("shooter")
          .voltage(Volts.of(left.getMotorVoltage().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(getVelocityRPS()));
      },
      this
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
    return sysIdRoutine.quasistatic(dir);
  }

  public Command sysIdDynamic (SysIdRoutine.Direction dir) {
    return sysIdRoutine.dynamic(dir);
  }

  public void runCharacterization(double volts) {
    shooterEnabled = false;
    left.setControl(charVoltage.withOutput(volts));
  }
}