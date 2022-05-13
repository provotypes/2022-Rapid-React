// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.LedStrip.ColorChoices;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Robot extends TimedRobot {
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private DifferentialDrive driveTrain;

  ////////////////////////////
  // ------- Motors ------- //
  ////////////////////////////
  private final CANSparkMax leftMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final double driveRampRate = 0.5;
  private final SlewRateLimiter filter = new SlewRateLimiter(0.8);

  // private final WPI_VictorSPX leftIntake = new WPI_VictorSPX(7);
  // private final WPI_VictorSPX rightIntake = new WPI_VictorSPX(8);
  // private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);
  // private final CANSparkMax intakeActuator = new CANSparkMax(5, MotorType.kBrushed);

  // private final WPI_TalonFX leftFlywheel = new WPI_TalonFX(9);
  // private final WPI_TalonFX rightFlywheel = new WPI_TalonFX(10);
  /**
 * Convert 2000 RPM to units / 100ms.
 * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
 * velocity setpoint is in units/100ms
 */
  private double shooterVelocity = 2000.0 * 2048.0 / 600.0;
  // private MotorControllerGroup flywheelMotors = new MotorControllerGroup(leftFlywheel, rightFlywheel);

  private final WPI_TalonFX leftClimber = new WPI_TalonFX(11);
  private final WPI_TalonFX rightClimber = new WPI_TalonFX(12);
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /////////////////////////////////
  // ------- Controllers ------- //
  /////////////////////////////////

  private final XboxController xboxController = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

  ////////////////////////////////
  // ------- Autonomous ------- //
  ////////////////////////////////

  String task = "None";

  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;
  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;
  private List<RelativeEncoder> encoders;
  private final AHRS gyro = new AHRS();

  private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // inches
  // 42 counts per revolution for the encoders

  private Auto autonomous;

  //////////////////////////////////
  // ------- Shuffleboard ------- //
  //////////////////////////////////

  private ShuffleboardTab dataTab = Shuffleboard.getTab("Data");
  private NetworkTableEntry leftEncoderPos;
  private NetworkTableEntry rightEncoderPos;

  // Flywheels
  private NetworkTableEntry flywheelSpeedSlider;
  public double flywheelSpeed;

  // private UsbCamera camera;  //Camera causes robot code to be deleted (I mean it wasn't actually that but we still hate the camera)
  // private NetworkTableEntry testingGraph;
  // private PowerDistribution PDP = new PowerDistribution();

  //////////////////////////////////
  // ------- Dependencies ------- //
  //////////////////////////////////

  private Intake intake = Intake.getInstance();
  private Shooter shooter = Shooter.getInstance();

  //////////////////////////////
  // ------- Lighting ------- //
  //////////////////////////////

  private LedStrip lightStrip;
  private ColorChoices defaultColor;

  ///////////////////////////
  // ------- Music ------- //
  ///////////////////////////

  // private WPI_TalonFX [] musicMotors = {leftFlywheel, rightFlywheel, leftClimber, rightClimber};
  private Playlist playlist;

  private void ResetSpark(CANSparkMax motor, Double rampRate) {
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(rampRate);
  }

  private void ResetTalon(WPI_TalonFX motor) {
    motor.configFactoryDefault();
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", "no task");
    m_chooser.addOption("Task 1-A", "task 1-A");
    m_chooser.addOption("2 Ball Auto", "task 1-B");
    m_chooser.addOption("1 Ball Auto", "task 2-A");
    m_chooser.addOption("Task 1-A", "task 1-A");
    m_chooser.addOption("Square Task", "square task");
    m_chooser.addOption("Test Task", "test task");
    // SmartDashboard.putData("Auto choices", m_chooser);
    dataTab.add("Auto Task", m_chooser).withSize(2, 1);

    // camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(320, 240);
    
    ResetSpark(leftMotor1, driveRampRate);
    ResetSpark(leftMotor2, driveRampRate);
    ResetSpark(rightMotor1, driveRampRate);
    ResetSpark(rightMotor2, driveRampRate);
    // leftIntake.configFactoryDefault();
    // rightIntake.configFactoryDefault();
    // intakeActuator.restoreFactoryDefaults();
    // ResetTalon(leftFlywheel);
    // ResetTalon(rightFlywheel);
    ResetTalon(leftClimber);
    ResetTalon(rightClimber);
    
    // intake.getActuator().setSmartCurrentLimit(30);
    // intake.getActuator().enableSoftLimit(SoftLimitDirection.kForward, true);
    // intake.getActuator().enableSoftLimit(SoftLimitDirection.kReverse, true);
    // intake.getActuator().setSoftLimit(SoftLimitDirection.kForward, -350);
    // intake.getActuator().setSoftLimit(SoftLimitDirection.kReverse, 10);

    //TODO
    // leftMotor1.setSmartCurrentLimit(0, 12, 0);
    // leftMotor2.setSmartCurrentLimit(0, 12, 0);
    // rightMotor1.setSmartCurrentLimit(0, 12, 0);
    // rightMotor2.setSmartCurrentLimit(0, 12, 0);
    
    rightMotors.setInverted(true);
    driveTrain = new DifferentialDrive(leftMotors, rightMotors);
    // driveTrain.setDeadband(0.1);

    leftEncoder1 = leftMotor1.getEncoder();
    leftEncoder2 = leftMotor2.getEncoder();
    rightEncoder1 = rightMotor1.getEncoder();
    rightEncoder2 = rightMotor2.getEncoder();

    leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

    // rightIntake.setInverted(false);
    // leftIntake.setInverted(false);
    // rightIntake.setNeutralMode(NeutralMode.Coast);
    // leftIntake.setNeutralMode(NeutralMode.Coast);
    // intakeActuator.setIdleMode(IdleMode.kBrake); 
    


    // rightFlywheel.follow(leftFlywheel);


    rightClimber.setInverted(true);
    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.follow(leftClimber);


    Shuffleboard.selectTab("Data");


    // dataTab.add("Left Encoder 1", leftEncoder1).withWidget(BuiltInWidgets.kEncoder);
    // dataTab.add("Right Encoder 1", rightEncoder1).withWidget(BuiltInWidgets.kEncoder);
    // dataTab.add("Drivetrain", driveTrain).withWidget(BuiltInWidgets.kDifferentialDrive);

    // gyroHeading = dataTab.add("Gyro Heading", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    dataTab.add("Gyro Heading", gyro).withWidget(BuiltInWidgets.kGyro);

    // testingGraph = dataTab.add("Flywheel Velocity", 0).withWidget(BuiltInWidgets.kGraph).withSize(2, 2).getEntry();

    // NetworkTableEntry pdpview = dataTab.add("PDP", 0).withWidget(BuiltInWidgets.kPowerDistribution).getEntry();
    // dataTab.add(PDP).withWidget(BuiltInWidgets.kPowerDistribution);
    // TODO
    // dataTab.addCamera("Front View", camera.getName(), "mjpg:http://0.0.0.0:1181/?action=stream");
    // dataTab.add("Front View", camera).withWidget(BuiltInWidgets.kCameraStream).withSize(2, 2);

    flywheelSpeedSlider = dataTab.add("Flywheel Speed (RPM)", 500)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 1, "max", 20660 * 600.0 / 2048.0))
    .getEntry();

    DriverStation.getAlliance();

    lightStrip = new LedStrip(0);

    leftMotor1.burnFlash();
    leftMotor2.burnFlash();
    rightMotor1.burnFlash();
    rightMotor2.burnFlash();
    // intakeActuator.burnFlash();

    // playlist = new Playlist(musicMotors);

    encoders = List.of(leftEncoder1, leftEncoder2, rightEncoder1, rightEncoder1);


  }



  @Override
  public void robotPeriodic() {

    //gyroHeading.setDouble(gyro.getYaw());
    // intakeActuatorGraph.setDouble(intake.getActuator().getOutputCurrent());
    //testingGraph.setDouble(leftFlywheel.getSelectedSensorVelocity() * 600.0 / 2048.0);
    // intakeActuatorGraph.setDouble(leftFlywheel.getSelectedSensorVelocity());
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance()==DriverStation.Alliance.Blue) {
        defaultColor = ColorChoices.BlueSolid;
      } 
      else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        defaultColor = ColorChoices.RedSolid;
      }
      else {
        defaultColor = ColorChoices.GreenSolid;
      }
    }
    else {
      defaultColor = ColorChoices.GreenStrobe;
    }
    //lightStrip.displayColor(defaultColor);

    // playlist.update();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    brake();
    autonomous = new Auto(driveTrain, gyro, encoders);
    task = m_chooser.getSelected();
    
    autonomous.setStatus(-1);
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + task);

    lightStrip.displayColor(defaultColor);
  }




  @Override
  public void autonomousPeriodic() {
    // intake.update();
    switch (task) {
      case "task 1-A": {
        autonomous.task_1a();
        break;
      }
      case "task 1-B": {
        autonomous.Auto2Ball();
        break;
      }
      case "task 2-A": {
        autonomous.Auto1Ball();
        break;
      }
      case "task 2-B": {
        break;
      }
      case "square task": {
        autonomous.square_task();
        break;
      }
      case "test task": {
        break;
      }
      default: {} // leave empty
    }
  }


  @Override
  public void teleopInit() {
    brake();
    lightStrip.displayColor(defaultColor);
  }

  double drive_speed = 0.0;
  double prev_drive_speed = 0.0;
  double noFilterSpeed = 0.0;
  int duration = 0;
  @Override
  public void teleopPeriodic() {

    prev_drive_speed = drive_speed;
    drive_speed = xboxController.getLeftY();

    // drive_speed = Math.sqrt(Math.abs(drive_speed)) * Math.signum(drive_speed);

    // drive_speed = (drive_speed + Math.signum(drive_speed)) * drive_speed * drive_speed * 3;
    // drive_speed = Math.max(Math.min(1, drive_speed), -1);

    noFilterSpeed = drive_speed;
    drive_speed = filter.calculate(drive_speed);

    if (noFilterSpeed > prev_drive_speed || drive_speed > -.4) {
      drive_speed = noFilterSpeed;
    }

    if (drive_speed > prev_drive_speed) {
      drive_speed = Math.min(drive_speed, prev_drive_speed + 0.02);
    }


    driveTrain.arcadeDrive(drive_speed, -xboxController.getRightX() * 0.75);

    if (joystick.getRawButton(8)) {
      intake.on();
    }
    else if (joystick.getRawButton(7)) {
      intake.reverse();
    }
    else {
      intake.off();
    }

    flywheelSpeed = flywheelSpeedSlider.getDouble(0) * 2048.0 / 600.0;

    if (xboxController.getRightTriggerAxis() > .1) {
      // flywheelMotors.set(flywheelSpeed);
      //leftFlywheel.set(TalonFXControlMode.PercentOutput, .68);
			/* 2000 RPM in either direction */
			// leftFlywheel.set(TalonFXControlMode.Velocity, shooterVelocity);
			// leftFlywheel.set(TalonFXControlMode.Velocity, 14000);
      shooter.on();
    }
    else if (xboxController.getLeftTriggerAxis() > .1) {
      // leftFlywheel.set(TalonFXControlMode.PercentOutput, .60);
      shooter.on();
    }
    else {
      // leftFlywheel.set(TalonFXControlMode.PercentOutput, 0);
      shooter.off();
      // leftFlywheel.set(TalonFXControlMode.Velocity, 0);
    }

    if (joystick.getRawButton(9) || xboxController.getPOV() == 180) {//climber down
      leftClimber.set(TalonFXControlMode.PercentOutput, -.85);
    }
    else if (joystick.getRawButton(10) || xboxController.getPOV() == 0) {
      leftClimber.set(TalonFXControlMode.PercentOutput, .85);
    }
    else {
      leftClimber.set(TalonFXControlMode.PercentOutput, 0);
    };

    if (xboxController.getStartButtonPressed()) {
      leftClimber.overrideLimitSwitchesEnable(true);
      rightClimber.overrideLimitSwitchesEnable(true);
    }

    if (joystick.getRawButton(4)) { //y button
      // intake.out();     
      // intake.getActuator().set(-1);

      // intakeController.setReference(-330, CANSparkMax.ControlType.kPosition);
      intake.out();
    }
    else if (joystick.getRawButton(6)) { //right bumper
      // intake.in();
      // intake.getActuator().set(1);

      // intakeController.setReference(0, CANSparkMax.ControlType.kPosition);
      intake.in();
    }
    else {
      intake.rest();
    }
    shooter.update();
    intake.update();

  }

  @Override
  public void disabledInit() {
    coast();
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.isFMSAttached()) {
      lightStrip.displayColor(defaultColor);
    }
    else {
      lightStrip.checkShuffleboard();
    }
  }

  @Override
  public void testInit() {
    // if (leftEncoderPos == null) {
    //   leftEncoderPos = dataTab.add("Left Encoders", 0).getEntry();
    //   rightEncoderPos = dataTab.add("Right Encoders", 0).getEntry();
    // }
  }

  @Override
  public void testPeriodic() {
    // leftEncoderPos.setDouble((leftEncoder1.getPosition() + leftEncoder2.getPosition())/2);
    // rightEncoderPos.setDouble((rightEncoder1.getPosition() + rightEncoder2.getPosition())/2);
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void brake() {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }

  public void coast() {
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }
}

// trigger - shoot
// intake in - 8
// intake out - 7
// intake actuator out - 6
// intale actuator in - 4
// climber up - 10
// climber down - 9