package frc.robot;

import edu.wpi.first.wpilibj.AnalogOutput;

public class LedStrip {
    private final int MAX_NAVX_MXP_DIGIO_PIN_NUMBER = 9;
    private final int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER = 3;
    private final int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER = 1;
    private final int NUM_ROBORIO_ONBOARD_DIGIO_PINS = 10;
    private final int NUM_ROBORIO_ONBOARD_PWM_PINS = 10;
    private final int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS = 4;
    private AnalogOutput strip;
    
    public LedStrip(int analogPort) {
        this.strip = new AnalogOutput(getChannelFromPin(PinType.AnalogOut, 0));
    }

    private enum PinType {
        DigitalIO, PWM, AnalogIn, AnalogOut
    };

    public enum ColorChoices {
        GreenStrobe, YellowStrobe, RedStrobe, BlueStrobe, GreenFlash, RedFlash, BlueFlash, YellowFlash, GreenSolid, RedSolid, BlueSolid, YellowSolid, BlueGreenStrobe, RedBlueStrobe, RedGreenStrobe, White, Off
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
        this.strip.setVoltage(voltage);
    }

    public void displayColor (ColorChoices color) {
        this.strip.setVoltage(findVoltage(color));
    }

    public double findVoltage (ColorChoices color) {
        switch (color) {
            case GreenStrobe:
            default:{
                return 0;}
            case RedStrobe:{
                return .2;}
            case BlueStrobe:
                return .35;
            case YellowStrobe:{
                return .5;}
            case GreenFlash:{
                return .6;}
            case RedFlash:{
                return .7;}
            case BlueFlash:{
                return .8;}
            case YellowFlash:{
                return .91;}
            case GreenSolid:{
                return 1.1;}
            case RedSolid:{
                return 1.2;}
            case BlueSolid:{
                return 1.3;}
            case YellowSolid:{
                return 1.4;}
            case BlueGreenStrobe:{
                return 1.7;}
            case RedBlueStrobe:{
                return 2;}
            case RedGreenStrobe:{
                return 2.2;}
            case White:{
                return 2.4;}
            case Off:{
                return 5;}

                


        }
    }

    
}
