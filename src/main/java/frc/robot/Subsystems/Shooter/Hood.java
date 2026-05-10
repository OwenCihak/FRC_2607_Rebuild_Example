// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import frc.robot.Settings.Constants.HoodConstants;
import frc.robot.Settings.States.HoodState;
import frc.robot.Utilities.Encoder;
import frc.robot.Utilities.MechanismBase;
import frc.robot.Utilities.SimClassGenerator;

/** Add your docs here. */
public class Hood extends MechanismBase<Angle>{

    private final RobotContainer robot;
    private final static TalonFXConfiguration config = mechanismConfigs(
                                                    HoodConstants.P, 
                                                    HoodConstants.I, 
                                                    HoodConstants.D, 
                                                    0, 
                                                    HoodConstants.S, 
                                                    HoodConstants.STATOR_LIMIT, 
                                                    HoodConstants.SUPPLY_LIMIT, 
                                                    HoodConstants.SUPPLY_LOWER_LIMIT,
                                                    HoodConstants.NEUTRAL_MODE);
    private HoodState state = HoodState.OFF;
    private final SingleJointedArmSim sim;
    private final static Encoder encoder = new Encoder(0, 
                                                0, 
                                                0, 
                                                0);

    public Hood(RobotContainer robot){
        super("Hood", config, HoodConstants.GEAR_RATIO, encoder, HoodConstants.MOTOR_ID);
        this.robot = robot;
        sim = SimClassGenerator.getSimSingleJointedArm(
                DCMotor.getKrakenX44(1),
                HoodConstants.GEAR_RATIO, 
                HoodConstants.MOMENT_OF_INERTIA, 
                2, 
                HoodConstants.MIN_POS,
                HoodConstants.MAX_POS, 
        false, 
        0);
    }

    @Override
    public void periodic() {
        determineState();
        control();
        logCoreMotorMetrics();
        logSubsystemMetrics();
    }

    public void setState(HoodState state){
        this.state = state;
    }

    public HoodState getState(){
        return state;
    }

    public void determineState(){
        robot.blockDuringAuto();

        if(robot.driverController.getA()){
           /*
            * if(inNeutralZone){
                state = HoodState.FERRYING;
            }
            else{
                state = HoodState.SHOOTING;
            }
            */
        }
        else if(robot.driverController.getRightBumper()){
            state = HoodState.MAX;
        }
        else if(robot.driverController.getLeftBumper()){
            state = HoodState.MIN;
        }
        else{
            state = HoodState.OFF;
        }
    }

    public void control(){ 
        switch (state) {
            case SHOOTING:
                shootingControl();
                break;
        
            case FERRYING:
                ferryingControl();
                break;

            case MIN:
                minControl();
                break;

            case MAX:
                maxControl();
                break;

            case OFF:
                offControl();
                break;

            default:
                offControl();
                break;
        }
    }

    private void shootingControl(){
        runToLocation(Degree.of(15.0));
    }

    private void ferryingControl(){
        runToLocation(Degree.of(35.0));
    }

    private void minControl(){
        runToLocation(Degree.of(HoodConstants.MIN_POS));
    }

    private void maxControl(){
        runToLocation(Degree.of(HoodConstants.MAX_POS));
    }

    private void offControl(){
        stopMotor();
    }

    @Override
    protected void logSubsystemMetrics() {
        logString(getNTKey() + "State", state.name());
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
        sim.setInputVoltage(appliedVolts);
        sim.update(dtSeconds);

        double rawAngle = sim.getAngleRads();
        double rawVelo = sim.getVelocityRadPerSec();

        Angle angle = Radians.of(rawAngle);
        AngularVelocity velo = RadiansPerSecond.of(rawVelo);

        double rotations = angle.in(Rotations);
        double rps = velo.in(RotationsPerSecond);

        simState.setRawRotorPosition(rotations);
        simState.setRotorVelocity(rps);
    }
}
