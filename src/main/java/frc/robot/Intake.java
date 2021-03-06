package frc.robot;

import java.util.Map;

import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Intake {
    private static Intake instance;
    private final WPI_VictorSPX leftIntake = new WPI_VictorSPX(7);
    private final WPI_VictorSPX rightIntake = new WPI_VictorSPX(8);
    private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);
    private final CANSparkMax intakeActuator = new CANSparkMax(5, MotorType.kBrushed);
    
    

    enum IntakeModes {
        intakeOn,
        intakeOff,
        intakeOut,
        intakeIn,
        intakeReverse
    }
    final Map<IntakeModes, Runnable> intakeModes = Map.ofEntries(
        entry(IntakeModes.intakeOff, this::executeOff),
        entry(IntakeModes.intakeOn, this::executeOn),
        entry(IntakeModes.intakeOut, this::executeOut),
        entry(IntakeModes.intakeIn, this::executeIn),
        entry(IntakeModes.intakeReverse, this::executeReverse)
    );

    private IntakeModes mode = IntakeModes.intakeOff;

    enum IntakeState {
        In,
        Out,
        movingIn,
        movingOut
    }
    private IntakeState state = IntakeState.In;

    public static Intake getInstance() {
        if(instance == null) { 
            instance = new Intake();
         }
        return instance;
    }

    public void update() {
        intakeModes.get(mode).run();

        if (state == IntakeState.movingIn) {
            executeIn();

            // if encoder, state = IntakeState.In;
        }

        else if (state == IntakeState.movingOut) {
            executeOut();

            // if encoder state = IntakeState.Out;
        }
    }

    public void off() {
        this.mode = IntakeModes.intakeOff;
    }

    public void on() {
        this.mode = IntakeModes.intakeOn;
    }

    public void reverse() {
        this.mode = IntakeModes.intakeReverse;
    }

    public void out() {
        this.mode = IntakeModes.intakeOut;
    }

    public void in () {
        this.mode = IntakeModes.intakeIn;
    }

    private void executeOff() {
        intakeMotors.set(0);
        intakeActuator.set(0);
    }

    private void executeOn() {
        intakeMotors.set(1);
        intakeActuator.set(0);
    }

    private void executeOut() {
        intakeMotors.set(0);
        intakeActuator.set(.5);
    }

    private void executeIn() {
        intakeMotors.set(0);
        intakeActuator.set(-.5);
    }

    private void executeReverse() {
        intakeActuator.set(0);
        intakeMotors.set(-.5);
    }

    public CANSparkMax getActuator() {
        return intakeActuator;
    }
}
