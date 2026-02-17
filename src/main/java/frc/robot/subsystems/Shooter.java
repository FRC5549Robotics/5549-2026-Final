package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX left = new TalonFX(Constants.LEFT_MOTOR_ID, "lil clanker"); //Create left shooter motor
  private final TalonFX right = new TalonFX(Constants.RIGHT_MOTOR_ID, "lil clanker"); //create right shooter motor

  private final VelocityVoltage leftVelReq = new VelocityVoltage(0); //create a velocity control request

  // Right follows left. Invert direction.
  private final Follower rightFollower = new Follower(Constants.LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);

  //Tuned feedforward
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.252, 0.0208);

  private double targetRPM = 0.0; //target rpm is 600 by default so robot idles at 600

  public Shooter() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.CurrentLimits.StatorCurrentLimit = 120; //shooter current limit
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast; //set default to coast mode

    // PID for minor corrections
    cfg.Slot0.kP = 0.05;  // start small, tune
    cfg.Slot0.kI = 0.00;
    cfg.Slot0.kD = 0.00;

    left.getConfigurator().apply(cfg);
    right.getConfigurator().apply(cfg);

    // Make the right motor follow left
    right.setControl(rightFollower);
  }

  public void shoot(double rpm) {
    targetRPM = rpm; //update targetRPM
  }

  public void off() {
    targetRPM = 0.0; //return to idle rpm
  }

  public boolean atSpeed() {
    double currentRPM = left.getVelocity().getValueAsDouble() * 60;
    return Math.abs(currentRPM - targetRPM) < 100; //Is flywheel rpm within tolerance?
  }

  @Override
  public void periodic() {

    double leftRPM = left.getVelocity().getValueAsDouble() * 60;

    SmartDashboard.putNumber("Shooter Left RPM", leftRPM);

    double targetRPS = targetRPM / 60.0;

    // Feedforward wants rad/s. Convert motor rot/s -> rad/s
    double targetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);

    double ffVolts = ff.calculate(targetRadPerSec); //determine voltage

    //send targetRPS and voltage
    left.setControl(
        leftVelReq
            .withVelocity(targetRPS)     
            .withFeedForward(ffVolts)    
    );
  }
}
