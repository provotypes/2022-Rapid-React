// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Robot.resetMotor;

/** Add your docs here. */
public class Climber {
    private static Climber instance;
    private final WPI_TalonFX leftClimber = new WPI_TalonFX(7);
    private final WPI_TalonFX rightClimber = new WPI_TalonFX(7);

    private Climber() {
        resetMotor(leftClimber, NeutralMode.Brake);
        resetMotor(rightClimber, NeutralMode.Brake);
        rightClimber.setInverted(true);
        rightClimber.follow(leftClimber);

    }

    //TODO try storing actions as lambdas in the enum
    enum ClimbingStates {
        climberUp ,
        climberDown,
        climberOff
    }

    private ClimbingStates state = ClimbingStates.climberOff;
    
    public static Climber getInstance() {
        if(instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void off() {
        this.state = ClimbingStates.climberOff;
    }

    public void up() {
        this.state = ClimbingStates.climberUp;
    }

    public void down() {
        this.state = ClimbingStates.climberDown;
    }

    private void executeOff() {
        leftClimber.set(ControlMode.PercentOutput, 0);
    }

    private void executeUp() {
        // leftClimber.set
    }
}