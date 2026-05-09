package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotContainer;
import frc.robot.Settings.Constants.DrumConstants;
import frc.robot.Settings.States.DrumState;
import frc.robot.Utilities.MechanismBase;
import frc.robot.Utilities.SimClassGenerator;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Drum extends MechanismBase<AngularVelocity>{

    private final RobotContainer robot;
    private final static TalonFXConfiguration config = mechanismConfigs(
                                                        DrumConstants.P, 
                                                        DrumConstants.I, 
                                                        DrumConstants.D, 
                                                        DrumConstants.V, 
                                                        DrumConstants.S, 
                                                        DrumConstants.STATOR_LIMIT, 
                                                        DrumConstants.SUPPLY_LIMIT, 
                                                        DrumConstants.SUPPLY_LOWER_LIMIT,
                                                        DrumConstants.NEUTRAL_MODE);
    private DrumState state = DrumState.OFF;
    private final FlywheelSim sim;

    public Drum(RobotContainer robot){
        super("Drum", config, DrumConstants.GEAR_RATIO, DrumConstants.ALIGNMENTS, DrumConstants.MOTOR_IDS);
        this.robot = robot;
        sim = SimClassGenerator.getSimFlyWheel(
                DCMotor.getKrakenX60(4), 
                DrumConstants.MOMENT_OF_INERTIA, 
                DrumConstants.GEAR_RATIO);
    }

    @Override
    public void periodic() {
        determineState();
        control();
        logCoreMotorMetrics();
        logSubsystemMetrics();
    }

    public void setState(DrumState state){
        this.state = state;
    }

    public DrumState setState(){
        return state;
    }

    public void determineState(){
        robot.blockDuringAuto();

        if(robot.driverController.getA()){
           /*
            * if(inNeutralZone){
                state = DrumState.FERRYING;
            }
            else{
                state = DrumState.SHOOTING;
            }
            */
        }
        else{
            state = DrumState.OFF;
        }
    }

    public void control(){
        if(state == DrumState.SHOOTING){
            shootingControl();
        }
        else if(state == DrumState.FERRYING){
            ferryingControl();
        }
        else if(state == DrumState.OFF){
            offControl();
        }
    }

    private void shootingControl(){
        runAtSpeed(RotationsPerSecond.of(50.0));
    }

    private void ferryingControl(){
        runAtSpeed(RotationsPerSecond.of(50.0));
    }

    private void offControl(){
        stopMotor();
    }

    @Override
    protected void logSubsystemMetrics() {
        logString(getNTKey() + "State", state.name());
    }

    @Override
    protected AngularVelocity getActualMetric() {
        return getActualVelocity();
    }

    @Override
    protected AngularVelocity getTargetMetric() {
        return getTargetVelocity();
    }

    @Override
    protected void updateMechanismSimulation(double appliedVolts, double dtSeconds) {
        sim.setInputVoltage(appliedVolts);
        sim.update(dtSeconds);

        AngularVelocity velo = sim.getAngularVelocity();

        simState.setRotorVelocity(velo.in(RotationsPerSecond));
    }
}