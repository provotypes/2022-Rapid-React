package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Auto {


    private PIDController turnController;
    double rotateToAngleRate;
    boolean turnControllerEnabled = false;
    
    private DifferentialDrive driveTrain;
    private AHRS gyro;
    private Intake intake = Intake.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private int state = 0;
    private int time = 0;
    private double turn_to = 0.0;
    private boolean finished_drive = false;
    private double target_angle = 90;
    private float initial_compass_heading = 0.0f;
    private float current_compass_heading = 0.0f;
    private boolean has_interference = false;
    private double total_angle = 0;
    private double current_angle = 0;

    static final double kToleranceDegrees = 2.0f;

    static final double kTargetAngleDegrees = 0.0f;

    private int task_status = 0;
    private RelativeEncoder leftEncoder1;
    private RelativeEncoder leftEncoder2;
    private RelativeEncoder rightEncoder1;
    private RelativeEncoder rightEncoder2;

    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0.00;
    
    public Auto(DifferentialDrive drivetrain, AHRS gyro, List<RelativeEncoder> encoders) {
        driveTrain = drivetrain;
        this.gyro = gyro;
        leftEncoder1 = encoders.get(0);
        leftEncoder2 = encoders.get(1);
        rightEncoder1 = encoders.get(2);
        rightEncoder2 = encoders.get(3);
        turnController = new PIDController(kP, kI, kD);
    }


    private void resetEncoders() {
        leftEncoder1.setPosition(0.0d);
        leftEncoder2.setPosition(0.0d);
        rightEncoder1.setPosition(0.0d);
        rightEncoder2.setPosition(0.0d);
      }
    
      public double getLeftEncoderDistance() {
        return Math.abs(((leftEncoder1.getPosition() + leftEncoder2.getPosition()) / 2.0d));
      }
    
      public double getRightEncoderDistance() {
        return Math.abs(((rightEncoder1.getPosition() + rightEncoder2.getPosition()) / 2.0d));
      }


    public void setStatus(int v) {
        task_status = v;
    }

    private boolean called = false;
    public void Auto2Ball() {
        switch (task_status) {
            case -1: {
                if (!called) {
                    time = 0;
                    called = true;
                    intake.out();
                    
                    gyro.zeroYaw();
                }
                intake.update();
                intake.getActuator().set(-0.5);
                time++;
                if (time >= 30) {
                    task_status++;
                    intake.getActuator().set(0);
                    resetEncoders();
                    time = 0;
                    called = false;
                }
                break; 
            }
            case 0: {
                if (!called) {
                    intake.getActuator().setIdleMode(IdleMode.kCoast);
                    intake.on();
                    called = true;
                }
                boolean reached_position = drive(6*12, -0.5);
                intake.update();
                if (reached_position) {
                    task_status++;
                    resetEncoders();
                    // intake.off();
                    time = 0;
                    intake.update();
                }
                break;
            }
            case 1: {
                task_status++;
                break;
            }
            case 2: {

                boolean reached_distance = drive(6*12, 0.5);
                time++;

                if (time >= 20) {
                    intake.off();
                    intake.update();
                    intake.getActuator().setIdleMode(IdleMode.kBrake);
                }
                if (reached_distance) {
                    task_status++;
                    resetEncoders();
                    gyro.zeroYaw();
                }
                break;
            }
            case 3: {
                boolean reached_angle = turn(170);

                if (reached_angle) {
                    task_status++;
                    time = 0;
                }
                break;
            }
            case 4: {
                time++;

                if (time >= 10) {
                    task_status++;
                    time = 0;
                }
                break;
            }
            case 5: {
                boolean reached_angle = slow_turn(180);

                if (reached_angle) {
                    task_status++;
                    resetEncoders();
                }

                break;
            }
            case 6: {
                boolean reached_distance = drive(3.5*12, -0.5);

                if (reached_distance) {
                    task_status++;
                    shooter.on();
                    shooter.update();
                }
                break;
            }
            case 7: {
                time++;
                shooter.update();
                if (time >= 100) {
                    shooter.off();
                    shooter.update();
                    task_status++;
                }
            }
            // add backup off tarmac here
        }
    }

    public void Auto1Ball() { // TODO: don't forget the breaks!!!
        switch (task_status) {
            case -1: {
                if (!called) {
                    time = 0;
                    called = true;
                }
                time++;

                if (time >= 300) {
                    task_status++;
                    called = false;
                    time = 0;
                }
                break;
            }
            case 0: {
                time++;
                if (!called) {
                    shooter.setPower(0.65);
                    shooter.on();
                    called = true;
                }
                shooter.update();

                if (time >= 150) {
                    time = 0;
                    task_status++;
                    shooter.off();
                    shooter.update();
                    resetEncoders();
                    called = false;
                }
                break;
            }
            case 1: {
                if (!called) {
                    called = true;
                    resetEncoders();
                }
                boolean reached_distance = drive(7.5*12, 0.5);

                if (reached_distance) {
                    task_status++;
                }
                break;
            }
            case 2: {
                if (!called) {
                    time = 0;
                    called = true;
                    intake.getActuator().setIdleMode(IdleMode.kCoast);
                    intake.out();
                }
                intake.update();
                intake.getActuator().set(-0.5);
                time++;
                if (time >= 30) {
                    task_status++;
                    intake.getActuator().set(0);
                    resetEncoders();
                    time = 0;
                    called = false;
                }
                break; 
            }
            case 3: {
                time++;

                if (time >= 10) {
                    time = 0;
                    task_status++;
                    intake.getActuator().setIdleMode(IdleMode.kBrake);
                    resetEncoders();
                }
                break;
            }
        }
    }


    public void task_1a() {

        switch (task_status) {
        case 0: {
            intake.on();
            //shooter.on();
            // enable flywheel
            boolean reached_position = drive(45, 0.6);
            if (reached_position) {
            task_status++;
            intake.off();
            resetEncoders();
            }
            break;
        }
        case 1: {
            //intake.on();
            //shooter.on();
            boolean reached_position = drive(25, -0.6);
            if (reached_position) {
            task_status++;
            //intake.off();
            time = 0;
            resetEncoders();
            }
            break;
        }
        case 2: {
            intake.on();
            shooter.on();
            boolean reached_angle = slow_turn(10);
            if (reached_angle) {
            task_status++;
            stop_motors();
            }
            break;
        }
        case 3: {
            //intake.on();
            shooter.on();
            // shooter.on();
            // intake.on();
            //shooter.ballFeederON();
            time += 1;
            if (time >= 30) {
            //shooter.ballFeederOFF();
            intake.off();
            shooter.off();
            task_status++;
            }
            break;
        }
        case 4: {
            boolean reached_angle = turn(86);
            if (reached_angle) {
            task_status+=2; // skip 5
            stop_motors();
            }
            break;
        }
        case 5: {
            boolean reached_angle = slow_turn(86);
            if (reached_angle) {
            task_status++;
            stop_motors();
            resetEncoders();
            
            }
            break;
        }
        case 6: {
            intake.on();
            boolean reached_distance = drive(135, 0.6);
            if (reached_distance) {
            task_status++;
            resetEncoders();
            // intake.off();
            }
            break;
        }
        case 7: {
            boolean reached_angle = turn(175);
            if (reached_angle) {
            task_status++;
            stop_motors();
            }

            break;
        }
        case 8: {

            boolean reached_angle = slow_turn(175);
            if (reached_angle) {
            task_status++;
            stop_motors();
            resetEncoders();
            }
            break;
        }
        case 9: {
            // intake.on();
            boolean reached_distance = drive(90, 0.6);

            if (reached_distance) {
            task_status++;
            }

            break;
        }
        case 10: {
            boolean reached_angle = turn(110);

            if (reached_angle) {
            task_status+= 2; // skip 11
            intake.off();
            resetEncoders();
            }

            break;
        }
        case 11: {
            boolean reached_angle = slow_turn(110);

            if (reached_angle) {
            task_status++;
            resetEncoders();
            }
            break;
        }
        case 12: {
            //intake.on();
            //shooter.on();
            boolean reached_distance = drive(98, -0.6);

            if (reached_distance) {
            task_status++;
            resetEncoders();
            }
            break;
        }
        case 13: {
            intake.on();
            shooter.on();
            boolean reached_angle = slow_turn(113);

            if (reached_angle) {
            task_status++;
            time = 0;
            }
            break;
        }
        case 14: {
            time++;
            intake.on();
            shooter.on();
            // shooter.on();
            //intake.on();
            //shooter.ballFeederON();
            if (time >= 400) {
            task_status++;
            shooter.off();
            intake.off();
            }
        }
        
        default: {} // leave this empty
        }
    }

    public void square_task() {
        if (state == 0) {

        finished_drive = drive(80, 0.4);

        if (finished_drive) {

            state = 1;
            // current_angle += target_angle;
        }
        } else if (state == 1) {
        // System.out.println(gyro.getAngle() + " -> " + angle);

        boolean reached_angle = turn(target_angle + current_angle);

        if (reached_angle) {
            state = 2;
            System.out.println("State 2");
            return;
        }

        // turn(target_angle + current_angle);
        } else if (state == 2) {

        // System.out.println(gyro.getAngle() + " -> " + angle);

        boolean reached_angle = slow_turn(target_angle + current_angle);

        if (reached_angle) {
            state = 0;
            time = 0;
            total_angle += gyro.getAngle();
            gyro.zeroYaw();
            return;
        }

        has_interference = gyro.isMagneticDisturbance();
        // System.out.println(has_interference);
        current_compass_heading = gyro.getCompassHeading();
        // System.out.println(current_angle + " " + total_angle + " " + target_angle + "
        // " + current_compass_heading);
        }
        // gyro.getAngle() >= -10 &&
    }

    private final double MAX_SPEED = 0.6;
    private int drive_state = 0;
    private double current_distance = 0;


    private void stop_motors() {
        driveTrain.stopMotor();
    }

    private boolean drive(double distance, double speed) {
        speed = MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);
        distance = Math.abs(distance);
        if (drive_state == 0) {
        resetEncoders();
        drive_state = 1;
        }
        if (drive_state == 1) {
        // driveTrain.arcadeDrive(speed, 0); // second arg can be used to counter robot
        // drifting left/right
        driveTrain.tankDrive(
            speed, // * 1.05,
            speed);

        current_distance = Math.min(getLeftEncoderDistance(), getRightEncoderDistance());
        if (current_distance >= distance - 1) {
            drive_state = 2;
        }
        }
        if (drive_state == 2) {
        double dir = (speed / Math.abs(speed)) * 2;
        driveTrain.tankDrive(
            MathUtil.clamp((getLeftEncoderDistance() - distance) * -dir, -0.2, 0.2),
            MathUtil.clamp((getRightEncoderDistance() - distance) * -dir, -0.2, 0.2));
        current_distance = Math.min(getLeftEncoderDistance(), getRightEncoderDistance());
        if (current_distance >= distance - 1 && current_distance <= distance + 1) {
            drive_state = 0;
            return true;
        }
        }
        return false;

    }

    private boolean turn(double angle) {

        double _angle = gyro.getAngle();
        if (_angle < -180.0) {
            while (_angle < -180.0) {
                _angle += 360.0;
            }
        } else if (_angle > 180.0) {
            while (_angle > 180.0) {
                _angle -= 360.0;
            }
        }

        if (angle < -180.0) {
            while (angle < -180.0) {
                angle += 360.0;
            }
        } else if (angle > 180.0) {
            while (angle > 180.0) {
                angle -= 360.0;
            }
        }

        if (_angle <= (-angle) + 6 && _angle >= (-angle) - 6) {

            return true;
        }

        if (!turnControllerEnabled) {
            turnController.setSetpoint(kTargetAngleDegrees);
            rotateToAngleRate = 0; // This value will be updated by the PID Controller
            turnControllerEnabled = true;
        }
        rotateToAngleRate = (turnController.calculate(gyro.getAngle() + angle));

        rotateToAngleRate *= 15; // increase scale of power
        rotateToAngleRate *= (rotateToAngleRate * rotateToAngleRate) + 5; // make difference more gradual
        rotateToAngleRate = (MathUtil.clamp(rotateToAngleRate / 10, -0.5, 0.5)); // put speed back in speed range
        driveTrain.tankDrive(-rotateToAngleRate, rotateToAngleRate);
        return false;
    }

    private boolean slow_turn(double angle) {

        double _angle = gyro.getAngle();
        if (_angle < -180.0) {
        while (_angle < -180.0) {
            _angle += 360.0;
        }
        } else if (_angle > 180.0) {
        while (_angle > 180.0) {
            _angle -= 360.0;
        }
        }

        if (angle < -180.0) {
        while (angle < -180.0) {
            angle += 360.0;
        }
        } else if (angle > 180.0) {
            while (angle > 180.0) {
                angle -= 360.0;
            }
        }

        if (_angle <= (-angle) + 4 && _angle >= (-angle) - 4) {
            return true;
        }

        if (!turnControllerEnabled) {
            turnController.setSetpoint(kTargetAngleDegrees);
            rotateToAngleRate = 0; // This value will be updated by the PID Controller
            turnControllerEnabled = true;
        }
        rotateToAngleRate = (turnController.calculate(gyro.getAngle() + angle));

        rotateToAngleRate *= 15;
        rotateToAngleRate *= (rotateToAngleRate * rotateToAngleRate) + 5;
        rotateToAngleRate = (MathUtil.clamp(rotateToAngleRate / 15, -0.325, 0.375));
        driveTrain.tankDrive(-rotateToAngleRate, rotateToAngleRate);
        return false;
    }
    
}
