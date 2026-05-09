// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import frc.robot.Settings.States.ActuatorState;
import frc.robot.Utilities.MechanismBase;
import frc.robot.Utilities.SimClassGenerator;

public class Actuator extends MechanismBase<Angle> {
  /** Creates a new Actuator. */
  private final RobotContainer robot;
  private final static TalonFXConfiguration configs = mechanismConfigs(0, 0, 0, 0, 0, 0, 0, 0, null);
  private ActuatorState state = ActuatorState.OFF;
  private final SingleJointedArmSim sim;

  public Actuator(RobotContainer robot) {
    super("Actuator", configs, 0, 0);
    this.robot = robot;
    sim = SimClassGenerator.getSimSingleJointedArm(null, 0, 0, 0, 0, 0, false, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    determineState();
  }

  public void determineState(){
    //Whole Match
    if(atGoal(0.01)){
      state = ActuatorState.OFF;
    }

    robot.blockDuringAuto();
    //During Teleop
    controlMotor();
  }

  public void controlMotor(){

  }

  @Override
  protected void logSubsystemMetrics() {
    
  }

  @Override
  protected Angle getActualMetric() {
    return getActualPosition();
  }

  @Override
  protected Angle getTargetMetric() {
    return getTargetPosition();
  }

  @Override
  protected void updateMechanismSimulation(double appliedVolts, double dtSeconds) {
    
  }
}
