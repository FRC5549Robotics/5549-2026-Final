// package frc.robot.subsystems;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StripTypeValue;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;

// import frc.robot.subsystems.LEDState;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class LED extends SubsystemBase {

//     private Supplier<LEDState> stateSupplier;

//     private final CANdle candle; //create candle

//     //define colors
//     private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
//     private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
//     //private static final RGBWColor RED = new RGBWColor(0, 255, 0, 0); //red override to green
//     private static final RGBWColor PURPLE = new RGBWColor(185, 0, 255, 0);
//     //private static final RGBWColor PURPLE = new RGBWColor(0, 255, 0, 0); //purple override to green

//     private LEDState currentState = null;

//     private final Timer blinkTimer = new Timer();
//     private boolean blinking = false;
//     private double blinkPeriod = 1.0;
//     private static final double BLINK_START_TIME = 10.0;

//     public void setStateSupplier(Supplier<LEDState> supplier) {
//         this.stateSupplier = supplier;
//     }

//     public LED() {
//         candle = new CANdle(19, "lil clanker"); // Further define candle

//         // Configure CANdle
//         CANdleConfiguration config = new CANdleConfiguration();
//         config.LED.StripType = StripTypeValue.GRB;
//         config.LED.BrightnessScalar = 0.5;

//         candle.getConfigurator().apply(config);

//         // Turn all LEDs solid green as their default state
//         candle.setControl(
//             new SolidColor(0, 399).withColor(GREEN)
//         );
//         currentState = LEDState.GREEN;

//         blinkTimer.start();
//     }

//     public void setCountdown(double secondsRemaining) {
//         if (secondsRemaining < 0) {
//             blinking = false;
//             return;
//         }

//         if (secondsRemaining > BLINK_START_TIME) {
//             blinking = false;
//             return;
//         }

//         blinking = true;

//         blinkPeriod = MathUtil.clamp(secondsRemaining/4.0, 0.08, 0.6);
//     }

//     public void setState(LEDState state) {
//         if (state == currentState) {
//             return;
//         }

//         currentState = state;
//     }

//     private void applySolid(LEDState state) {
//         switch (state) {
//             case RED:
//                 candle.setControl( new SolidColor(0, 399).withColor(RED));
//                 break;
//             case PURPLE: 
//                 candle.setControl( new SolidColor(0, 399).withColor(PURPLE));
//                 break;
//             case GREEN: 
//                 candle.setControl( new SolidColor(0, 399).withColor(GREEN));
//                 break;
//         }
//     }

//     @Override
//     public void periodic() {
//         if (stateSupplier != null) {
//             setState(stateSupplier.get());
//         }

//         if (blinking) {
//             boolean on = (blinkTimer.get() % blinkPeriod) < (blinkPeriod/2.0);

//             if (on) {
//                 applySolid(currentState);
//             } else {
//                 candle.setControl(new SolidColor(0, 399).withColor(new RGBWColor(0,0,0,0)));
//             }
//         } else {
//             applySolid(currentState);
//         }

//         SmartDashboard.putString("LED State", currentState.name());
//     }
// }