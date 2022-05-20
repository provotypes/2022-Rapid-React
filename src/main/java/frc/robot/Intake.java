package frc.robot;

import java.util.Map;

import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Intake {
    private static Intake instance;
    private final WPI_VictorSPX leftIntake = new WPI_VictorSPX(7);
    private final WPI_VictorSPX rightIntake = new WPI_VictorSPX(8);
    private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);
    private final CANSparkMax intakeActuator = new CANSparkMax(5, MotorType.kBrushed);
    private SparkMaxPIDController intakeController;
    private RelativeEncoder actuatorEncoder;
    
    
    private Intake() {
        intakeActuator.restoreFactoryDefaults();
        intakeActuator.setIdleMode(IdleMode.kBrake);
        intakeActuator.setOpenLoopRampRate(.4);
        intakeActuator.setInverted(true);
        actuatorEncoder = intakeActuator.getEncoder(Type.kQuadrature, 1024);
        actuatorEncoder.setPosition(0);
        intakeController = intakeActuator.getPIDController();
        // intakeActuator.setSecondaryCurrentLimit(20);


        intakeController.setP(1, 1); //0 to -65 at 4096 per rev
        intakeController.setI(0, 1);
        intakeController.setD(0, 1);
        intakeController.setIZone(0, 1);
        intakeController.setFF(0, 1);
        intakeController.setOutputRange(-.85, .85, 1);

        System.out.println("Did intake setup");
    }

    enum IntakeModes {
        intakeOn,
        intakeOff,
        intakeReverse
    }

    enum IntakeActuatorModes {
        intakeIn,
        intakeOut,
        intakeRest
    }
    final Map<IntakeModes, Runnable> intakeModes = Map.ofEntries(
        entry(IntakeModes.intakeOff, this::executeOff),
        entry(IntakeModes.intakeOn, this::executeOn),
        entry(IntakeModes.intakeReverse, this::executeReverse)
    );

    final Map<IntakeActuatorModes, Runnable> intakeActuatorModes = Map.ofEntries(
        entry(IntakeActuatorModes.intakeIn, this::executeIn),
        entry(IntakeActuatorModes.intakeOut, this::executeOut),
        entry(IntakeActuatorModes.intakeRest, this::executeRest)
    );

    private IntakeModes mode = IntakeModes.intakeOff;
    private IntakeActuatorModes actMode = IntakeActuatorModes.intakeIn;

    // enum IntakeState {
    //     In,
    //     Out,
    //     movingIn,
    //     movingOut
    // }
    // private IntakeState state = IntakeState.In;

    public static Intake getInstance() {
        if(instance == null) { 
            instance = new Intake();
        }
        return instance;
    }

    public void update() {
        intakeModes.get(mode).run();
        intakeActuatorModes.get(actMode).run();

        // if (state == IntakeState.movingIn) {
        //     executeIn();

        //     // if encoder, state = IntakeState.In;
        // }

        // else if (state == IntakeState.movingOut) {
        //     executeOut();

        //     // if encoder state = IntakeState.Out;
        // }
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
        this.actMode = IntakeActuatorModes.intakeOut;
    }

    public void in() {
        this.actMode = IntakeActuatorModes.intakeIn;
    }
    
    public void rest() {
        this.actMode = IntakeActuatorModes.intakeRest;
    }

    private void executeOff() {
        intakeMotors.set(0);
        // intakeActuator.set(0);
    }

    private void executeOn() {
        intakeMotors.set(1);
        // intakeActuator.set(0);
    }

    private void executeOut() {
        intakeController.setReference(260, CANSparkMax.ControlType.kPosition, 1);
        // intakeActuator.set(.75);
    }

    private void executeIn() {
        intakeController.setReference(0, CANSparkMax.ControlType.kPosition, 1);
        // intakeActuator.set(-.75);
    }

    private void executeReverse() {
        // intakeActuator.set(0);
        intakeMotors.set(-.5);
    }

    private void executeRest() {
        intakeActuator.set(0);
    }


}
