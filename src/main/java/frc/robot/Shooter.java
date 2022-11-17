package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.FollowerType;
// import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Robot.resetMotor;


// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Shooter {
    
    private static Shooter instance;
    private final WPI_TalonFX leftFlywheel = new WPI_TalonFX(9);
    private final WPI_TalonFX rightFlywheel = new WPI_TalonFX(10);
    // private MotorControllerGroup flywheelMotors = new MotorControllerGroup(leftFlywheel, rightFlywheel);

    /**
    * Convert 2000 RPM to units / 100ms.
    * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
    * velocity setpoint is in units/100ms
    */
    private final double defaultPower = 0.7;
    private double power = 0.7;

    enum ShooterModes {
        shooterOn,
        shooterOff
    }
    final Map<ShooterModes, Runnable> shooterModes = Map.ofEntries(
        entry(ShooterModes.shooterOff, this::_off),
        entry(ShooterModes.shooterOn, this::_on)
    );

    private ShooterModes mode = ShooterModes.shooterOff;

    private Shooter() {
        init();
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
            return instance;
    }

    private void init() {
        resetMotor(leftFlywheel, NeutralMode.Coast);
        resetMotor(rightFlywheel, NeutralMode.Coast);
        rightFlywheel.follow(leftFlywheel);
        rightFlywheel.setInverted(true);

        leftFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        leftFlywheel.configNominalOutputForward(0, 0);
        leftFlywheel.configNominalOutputReverse(0, 0);
        leftFlywheel.configPeakOutputForward(1, 0);
        leftFlywheel.configPeakOutputReverse(-1, 0);
        leftFlywheel.config_kF(0, 0.0489, 0); //1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output; at least, that's what the documentation says
        leftFlywheel.config_kP(0, 0.04, 0);
        leftFlywheel.config_kI(0, 0.0, 0);
        leftFlywheel.config_kD(0, 0.4, 0);
        leftFlywheel.selectProfileSlot(0, 0);
    }

    public void setPower(double v) {
        power = v;
    }

    public void resetPower() {
        power = defaultPower;
    }

    private void _on() {
        // flywheelMotors.set(power);
        leftFlywheel.set(TalonFXControlMode.Velocity, (int) 10800 * 0.95); //was 13900, not sure why it changed
    }

    private void _off() {
        // flywheelMotors.set(0);
        leftFlywheel.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void on() {
        this.mode = ShooterModes.shooterOn;
    }

    public void off() {
        this.mode = ShooterModes.shooterOff;
    }

    public void update() {

        shooterModes.get(mode).run();
    }

    public void name() {
        
    }
}
