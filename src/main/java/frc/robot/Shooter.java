package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Shooter {
    
    private static Shooter instance;
    private final WPI_TalonFX leftFlywheel = new WPI_TalonFX(9);
    private final WPI_TalonFX rightFlywheel = new WPI_TalonFX(10);
    private MotorControllerGroup flywheelMotors = new MotorControllerGroup(leftFlywheel, rightFlywheel);

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

    private Shooter() {}

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
            return instance;
    }

    public void setPower(double v) {
        power = v;
    }

    public void resetPower() {
        power = defaultPower;
    }

    private void _on() {
        flywheelMotors.set(power);
    }

    private void _off() {
        flywheelMotors.set(0);
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

}
