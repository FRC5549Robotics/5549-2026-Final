package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final CANdle candle;

    private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
    

    public LED() {
        candle = new CANdle(19, "lil clanker"); // CHANGE to your CAN ID

        // Configure CANdle
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 0.5;

        candle.getConfigurator().apply(config);

        // Turn all LEDs solid green (0–399, adjust length as needed)
        candle.setControl(
            new SolidColor(0, 399).withColor(GREEN)
        );
    }
    public void setGreen() {
        candle.setControl( new SolidColor(0, 399).withColor(GREEN));
    };
}
