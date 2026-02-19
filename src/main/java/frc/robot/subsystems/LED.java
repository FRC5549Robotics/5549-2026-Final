package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.LEDState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED extends SubsystemBase {

    private Supplier<LEDState> stateSupplier;

    private final CANdle candle; //create candle

    //define colors
    private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
    private static final RGBWColor BLUE = new RGBWColor(0, 0, 255, 0);

    private LEDState currentState = null;

    public void setStateSupplier(Supplier<LEDState> supplier) {
        this.stateSupplier = supplier;
    }

    public LED() {
        candle = new CANdle(19, "lil clanker"); // Further define candle

        // Configure CANdle
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 0.5;

        candle.getConfigurator().apply(config);

        // Turn all LEDs solid green as their default state
        candle.setControl(
            new SolidColor(0, 399).withColor(GREEN)
        );
        currentState = LEDState.GREEN;
    }

    public void setState(LEDState state) {

        if (state == currentState) {
            return;
        }

        currentState = state;

        switch (state) {
            case RED:
                candle.setControl( new SolidColor(0, 399).withColor(RED));
                break;
            case BLUE: 
                candle.setControl( new SolidColor(0, 399).withColor(BLUE));
                break;
            case GREEN: 
                candle.setControl( new SolidColor(0, 399).withColor(GREEN));
                break;
        }
    }

    @Override
    public void periodic() {
        if (stateSupplier != null) {
            setState(stateSupplier.get());
        }

        SmartDashboard.putString("LED State", currentState.name());
    }
}