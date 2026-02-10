package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hood extends SubsystemBase{

    TalonFX HoodMotor;
    TalonFXConfiguration HoodMotorConfig;
    


    public Hood(){
        HoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID, "lil clanker");
        HoodMotorConfig = new TalonFXConfiguration();
        HoodMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        HoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        HoodMotor.getConfigurator().apply(HoodMotorConfig);
    }

    public void HoodUp(){
        double pos = HoodMotor.getPosition().getValueAsDouble();

        if (pos < 4.3) {
            HoodMotor.set(0.1);  // move up
            System.out.println("Hood going up, Hood go vroom vroom");
        } else {
            HoodMotor.set(0.0);  // stop at the limit
            System.out.println("Hood at upper limit!");
        }

        SmartDashboard.putNumber("Hood Position", pos);
    }

    public void HoodDown() {
        double pos = HoodMotor.getPosition().getValueAsDouble();

        if (pos > 1.0) {   // assuming 0 is your bottom
            HoodMotor.set(-0.1);
            System.out.println("Hood going down, Twinkle twinkle little star");
        } else {
            HoodMotor.set(0.0);
            System.out.println("Hood at lower limit!");
        }

        SmartDashboard.putNumber("Hood Position", pos);
    }

    
        public void HoodOff() {
        HoodMotor.set(0.0);
    }
    public void periodic(){
        double pos = HoodMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Hood Position", pos);
    }
    
    
    

}