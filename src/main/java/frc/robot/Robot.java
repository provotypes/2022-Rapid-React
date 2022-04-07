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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.LedStrip.ColorChoices;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Robot extends TimedRobot {
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private DifferentialDrive driveTrain;
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
  private double driveRampRate = 0.5;

  private final WPI_VictorSPX leftIntake = new WPI_VictorSPX(7);
  private final WPI_VictorSPX rightIntake = new WPI_VictorSPX(8);
  private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);
  private final CANSparkMax intakeActuator = new CANSparkMax(5, MotorType.kBrushed);

  private final WPI_TalonFX leftFlywheel = new WPI_TalonFX(9);
  private final WPI_TalonFX rightFlywheel = new WPI_TalonFX(10);
  private MotorControllerGroup flywheelMotors = new MotorControllerGroup(leftFlywheel, rightFlywheel);

  private final WPI_TalonFX leftClimber = new WPI_TalonFX(11);
  private final WPI_TalonFX rightClimber = new WPI_TalonFX(12);

  private final XboxController xboxController = new XboxController(0);
  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;
  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;
  private List<RelativeEncoder> encoders;
  private AHRS gyro = new AHRS();

  private ShuffleboardTab dataTab = Shuffleboard.getTab("Data");
  private ShuffleboardTab LEDTab = Shuffleboard.getTab("LEDs");
  private NetworkTableEntry leftEncoderPos;
  private NetworkTableEntry rightEncoderPos;
  private NetworkTableEntry flywheelSpeedSlider;
  private double flywheelSpeed;
  private UsbCamera camera;
  private LedStrip lightStrip;
  // private double voltage;
  private NetworkTableEntry voltage;
  private NetworkTableEntry lightMode;
  private final SendableChooser<ColorChoices> colorChoicer = new SendableChooser<>();
  private ComplexWidget colorChoosereee;
  private boolean lastChoice;
  private NetworkTableEntry intakeActuatorGraph;
  private PowerDistribution PDP = new PowerDistribution();

  private Auto autonomous;
  
  private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // inches
  // 42 counts per revolution for the encoders

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", "no task");
    m_chooser.addOption("Task 1-A", "task 1-A");
    m_chooser.addOption("Task 1-B", "task 1-B");
    m_chooser.addOption("Task 2-A", "task 2-A");
    m_chooser.addOption("Task 1-A", "task 1-A");
    m_chooser.addOption("Square Task", "square task");
    m_chooser.addOption("Test Task", "test task");
    //SmartDashboard.putData("Auto choices", m_chooser);
    dataTab.add(m_chooser).withSize(2, 1);

    //camera = CameraServer.startAutomaticCapture();
    
    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();
    leftIntake.configFactoryDefault();
    rightIntake.configFactoryDefault();
    intakeActuator.restoreFactoryDefaults();
    leftFlywheel.configFactoryDefault();
    rightFlywheel.configFactoryDefault();
    leftClimber.configFactoryDefault();
    rightClimber.configFactoryDefault();
    
    coast();

    leftMotor1.setOpenLoopRampRate(driveRampRate);
    leftMotor2.setOpenLoopRampRate(driveRampRate);
    rightMotor1.setOpenLoopRampRate(driveRampRate);
    rightMotor2.setOpenLoopRampRate(driveRampRate);
    //TODO
    leftMotor1.setSmartCurrentLimit(0, 12, 0);
    leftMotor2.setSmartCurrentLimit(0, 12, 0);
    rightMotor1.setSmartCurrentLimit(0, 12, 0);
    rightMotor2.setSmartCurrentLimit(0, 12, 0);
    
    rightMotors.setInverted(true);
    driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoder1 = leftMotor1.getEncoder();
    leftEncoder2 = leftMotor2.getEncoder();
    rightEncoder1 = rightMotor1.getEncoder();
    rightEncoder2 = rightMotor2.getEncoder();

    leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

    rightIntake.setInverted(false);
    leftIntake.setInverted(false);
    rightIntake.setNeutralMode(NeutralMode.Coast);
    leftIntake.setNeutralMode(NeutralMode.Coast);
    intakeActuator.setIdleMode(IdleMode.kBrake); 
    
    rightFlywheel.setInverted(true);
    leftFlywheel.setNeutralMode(NeutralMode.Coast);
    rightFlywheel.setNeutralMode(NeutralMode.Coast);
    //rightFlywheel.follow(leftFlywheel);

    rightClimber.setInverted(true);
    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.follow(leftClimber);

    Shuffleboard.selectTab("Data");

    leftEncoderPos = dataTab.add("Left Encoders", 0).getEntry();
    rightEncoderPos = dataTab.add("Right Encoders", 0).getEntry();
    // gyroHeading = dataTab.add("Gyro Heading", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    dataTab.add(gyro).withWidget(BuiltInWidgets.kGyro);
    lightMode = LEDTab.add("LED Control Mode", true).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 1).getEntry();
    voltage = LEDTab.add("LED Voltage", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 5)).withPosition(1, 1).getEntry();
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
    colorChoosereee = LEDTab.add(colorChoicer).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 1);

    lastChoice = lightMode.getBoolean(true);

    intakeActuatorGraph = dataTab.add("Intake Actuator Voltage", 0).withWidget(BuiltInWidgets.kGraph).withSize(2, 2).getEntry();

    //NetworkTableEntry pdpview = dataTab.add("PDP", 0).withWidget(BuiltInWidgets.kPowerDistribution).getEntry();
    //dataTab.add(PDP).withWidget(BuiltInWidgets.kPowerDistribution);
    //TODO
    //dataTab.addCamera("Front View", camera.getName(), "mjpg:http://0.0.0.0:1181/?action=stream");
    //dataTab.add(camera);

    flywheelSpeedSlider = dataTab.add("Flywheel Speed", .5)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

    DriverStation.getAlliance();

    lightStrip = new LedStrip(0);

    leftMotor1.burnFlash();
    leftMotor2.burnFlash();
    rightMotor1.burnFlash();
    rightMotor2.burnFlash();
    intakeActuator.burnFlash();

    Playlist.load();

    encoders = List.of(leftEncoder1, leftEncoder2, rightEncoder1, rightEncoder1);

  }


  @Override
  public void robotPeriodic() {
    leftEncoderPos.setDouble((leftEncoder1.getPosition() + leftEncoder2.getPosition())/2);
    rightEncoderPos.setDouble((rightEncoder1.getPosition() + rightEncoder2.getPosition())/2);
    //gyroHeading.setDouble(gyro.getYaw());
    intakeActuatorGraph.setDouble(intakeActuator.getAppliedOutput());
    flywheelSpeed = flywheelSpeedSlider.getDouble(0);
    //TODO
    /*if (lastChoice != lightMode.getBoolean(true)) {
      if (lightMode.getBoolean(true)) {
        //voltage.delete();
        //LEDTab.add(colorChoicer).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
      }
      else {
        //colorChoicer.close();
        //voltage = LEDTab.add("LED Voltage", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 4)).getEntry();
      }
    }*/
    if (lightMode.getBoolean(true)) {
      lightStrip.displayColor(colorChoicer.getSelected());
    }
    else {
      lightStrip.displayColor(voltage.getDouble(0));
      
    }
    //lightStrip.displayColor(() -> {if (lightMode.getBoolean(true)){colorChoicer.getSelected();} else {voltage.getDouble(0);}});

    lastChoice = lightMode.getBoolean(true);

    Playlist.update();
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + task);
  }


  String task = "None";
  @Override
  public void autonomousPeriodic() {
    switch (task) {
      case "task 1-A": {
        autonomous.task_1a();
        break;
      }
      case "task 1-B": {
        break;
      }
      case "task 2-A": {
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
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.arcadeDrive(xboxController.getLeftY(), -xboxController.getRightX());
    if (xboxController.getLeftBumper()) {
      intakeMotors.set(1);
    }
    else {
      intakeMotors.set(0);
    }

    // if (xboxController.getRightBumper()) {
    //   rightIntake.set(1);
    // }
    // else {
    //   rightIntake.set(0);
    // }

    if (xboxController.getXButton()) {
      flywheelMotors.set(flywheelSpeed);
    }
    else {
      flywheelMotors.set(0);
    }

    if (xboxController.getBButton()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, -.75);
    }
    else if (xboxController.getAButton()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, .75);
    }
    else {
      leftClimber.set(TalonFXControlMode.PercentOutput, 0);
    };

    if (xboxController.getStartButtonPressed()) {
      leftClimber.overrideLimitSwitchesEnable(true);
      rightClimber.overrideLimitSwitchesEnable(true);
    }

    if (xboxController.getPOV() == 0) {
      intakeActuator.set(.75);      
    }
    else if (xboxController.getPOV() == 180) {
      intakeActuator.set(-.75);
    }
    else {
      intakeActuator.set(0);
    }

  }

  @Override
  public void disabledInit() {
    coast();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

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