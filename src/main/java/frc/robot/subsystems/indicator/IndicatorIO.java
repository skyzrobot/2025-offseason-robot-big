package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;
import frc.robot.drivers.led.patterns.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Seconds;

public interface IndicatorIO {
    default void updateInputs(IndicatorIOInputs inputs) {}

    default void setPattern(Patterns pattern) {}

    default void reset() {}

    enum Patterns {
        LOSS(new BlinkingPattern(Color.kOrange, 1.0)),
        RED_ALLIANCE(new ScannerPattern(Color.kRed, 8)),
        BLUE_ALLIANCE(new ScannerPattern(Color.kBlue, 8)),
        NORMAL(new BreathingPattern(Color.kBlue, 1.5)),
        EDGE_CASE(new BreathingPattern(Color.kLightSteelBlue, 1.5)),

        INTAKE(new BlinkingPattern(Color.kRed, 0.04)),
        ASSISTED_INTAKE(new BlinkingPattern(Color.kOrange, 0.04)),
        INDEXED_INTAKE(new BlinkingPattern(Color.kLightYellow, 0.04)),
        AFTER_INTAKE(new BlinkingPattern(Color.kGreen, 0.04)),

        RESET_ODOM(new BlinkingPattern(Color.kPurple, 0.1)),
        AIMING(new BlinkingPattern(Color.kBlue, 0.04)),
        AIMED(new BlinkingPattern(Color.kGreen, 0.04)),

        CLIMB_DEPLOYED(new BlinkingPattern(Color.kWhite, 0.2)),
        CLIMB_FINISHED(new RainbowingPattern());

        public final AddressableLEDPattern pattern;
        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern = Patterns.NORMAL;
    }
}
