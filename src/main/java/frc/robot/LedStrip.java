package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class LedStrip {
    private final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER = 9;
    private final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER = 3;
    private final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER = 1;
    private final int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
    private final int NUM_ROBORIO_ONBOARD_PWM_PINS = 10;
    private final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4;
    private AnalogOutput strip;

    private ShuffleboardTab LEDTab = Shuffleboard.getTab("LEDs");
    private GenericEntry lightMode;
    private GenericEntry voltage;
    private final SendableChooser<ColorChoices> colorChoicer = new SendableChooser<>();



    
    public LedStrip(int analogPort) {
        this.strip = new AnalogOutput(getChannelFromPin(PinType.AnalogOut, 0));

        boolean firstTry = true;
        for (ColorChoices choice: ColorChoices.values()) {
            if (firstTry) {
                colorChoicer.setDefaultOption(choice.toString(), choice);
                firstTry = false;
            }
            else {
                colorChoicer.addOption(choice.toString(), choice);
            }
        }
        LEDTab.add(colorChoicer).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 1);
        lightMode = LEDTab.add("LED Control Mode", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 1).getEntry();
        voltage = LEDTab.add("LED Voltage", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 5)).withPosition(1, 1).getEntry();
    }

    private enum PinType {
        DigitalIO, PWM, AnalogIn, AnalogOut
    };

    public enum ColorChoices {
        GreenStrobe (0), 
        YellowStrobe (.5), 
        RedStrobe (.2), 
        BlueStrobe (.35), 
        GreenFlash (.6), 
        RedFlash (.7), 
        BlueFlash (.8),
        YellowFlash (.91), 
        GreenSolid (1.1), 
        RedSolid (1.2), 
        BlueSolid (1.3), 
        YellowSolid (1.4), 
        BlueGreenStrobe (1.7), 
        RedBlueStrobe (2), 
        RedGreenStrobe (2.2), 
        White (2.4), 
        Off (5);

        private double value;

        ColorChoices(double value) {
            this.value = value;
        }
    }

    private int getChannelFromPin(PinType type, int io_pin_number) throws IllegalArgumentException {
        int roborio_channel = 0;
        if (io_pin_number < 0) {
            throw new IllegalArgumentException("Error:  navX MXP I/O Pin #");
        }
        switch (type) {
        case DigitalIO:
            if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
                throw new IllegalArgumentException("Error:  Invalid navX MXP Digital I/O Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS + (io_pin_number > 3 ? 4 : 0);
            break;
        case PWM:
            if (io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER) {
                throw new IllegalArgumentException("Error:  Invalid navX MXP Digital I/O Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
            break;
        case AnalogIn:
            if (io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER) {
                throw new IllegalArgumentException("Error:  Invalid navX MXP Analog Input Pin #");
            }
            roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
            break;
        case AnalogOut:
            if (io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER) {
                throw new IllegalArgumentException("Error:  Invalid navX MXP Analog Output Pin #");
            }
            roborio_channel = io_pin_number;
            break;
        }
        return roborio_channel;
    }

    public void displayColor (double voltage) {
        strip.setVoltage(voltage);
    }

    public void displayColor (ColorChoices color) {
        strip.setVoltage(color.value);
    }

    public void checkShuffleboard() {

        if (lightMode.getBoolean(true)) {
            this.displayColor(colorChoicer.getSelected());
        }
        else {
            this.displayColor(voltage.getDouble(0));
        }

        // if (lightMode.getBoolean(true)) {
        //     this.strip.displayColor(colorChoicer.getSelected());
        // }
        // else {
        //     this.strip.displayColor(voltage.getDouble(0));
        // }
    }
}
