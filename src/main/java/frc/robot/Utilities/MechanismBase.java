// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Utilities.SysId.ISysIdTunable;
import frc.robot.Utilities.SysId.SysIdBuilder;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;

public abstract class MechanismBase<T extends Measure<?>> extends SubsystemBase implements ISysIdTunable {
  protected final TalonFX mainMotor;
  protected final ArrayList<TalonFX> motors = new ArrayList<>();
  protected Encoder encoder;
  protected final String subsystemName;
  protected T targetMetric;
  protected final TalonFXSimState simState;
  private final SysIdRoutine sysIdRoutine; 
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Current> supplyCurrentSignal;    
  private final StatusSignal<Angle> motorPositionSignal;  
  private final StatusSignal<AngularVelocity> motorVelocitySignal;    
  private final StatusSignal<Voltage> motorVoltageSignal;      
  private final StatusSignal<Temperature> motorTemperatureSignal;     
  private final StatusSignal<Double> closedLoopErrorSignal;          
 



  protected final VoltageOut voltageRequest = new VoltageOut(0.0);
  protected final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  protected final PositionVoltage positionRequest = new PositionVoltage(0.0);  
  protected final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    int canId) {

    this(subsystemName, config, gearRatio, null, null, canId);

   } 
  
  public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    MotorAlignmentValue[] motorAlignments,
    int... canIds) {
    
    this(subsystemName, config, gearRatio, 0.0, motorAlignments, canIds);

   }  

  public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    Encoder encoder,
    int canId) {

    this(subsystemName, config, gearRatio, null, encoder, canId);

   } 
   
   public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    MotorAlignmentValue[] motorAlignments,
    Encoder encoder,
    int... canIds) {
    
    this(subsystemName, config, gearRatio, 0.0, motorAlignments, encoder, canIds);

   }

  public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    double stepVoltage,
    MotorAlignmentValue[] motorAlignments,
    Encoder encoder,
    int... canIds
    ){

      this(subsystemName, config, gearRatio, stepVoltage, motorAlignments, canIds);
      this.encoder = encoder;

    }

  public MechanismBase(
    String subsystemName,
    TalonFXConfiguration config,
    double gearRatio,
    double stepVoltage,
    MotorAlignmentValue[] motorAlignments,
    int... canIds
    ) {

    this.subsystemName = subsystemName;

    for(int canId : canIds){
      motors.add(new TalonFX(canId));
    }

    mainMotor = motors.get(0);
    for(int i = 1; i < motors.size(); i++){
      motors.get(i).setControl(new Follower(canIds[0], motorAlignments[i-1]));
    }
    
    // FORCIBLY INJECT THE RATIO INTO THE CONFIG
    // This guarantees it is set, even if the student forgot to add it 
    // to the config object in the child class.
    config.Feedback.SensorToMechanismRatio = gearRatio;    
    mainMotor.getConfigurator().apply(config);

    statorCurrentSignal = mainMotor.getStatorCurrent();
    statorCurrentSignal.setUpdateFrequency(50);

    supplyCurrentSignal = this.mainMotor.getSupplyCurrent();
    supplyCurrentSignal.setUpdateFrequency(50);

    motorPositionSignal = mainMotor.getPosition();
    motorPositionSignal.setUpdateFrequency(50);

    motorVelocitySignal = mainMotor.getVelocity();
    motorVelocitySignal.setUpdateFrequency(50);

    motorVoltageSignal = mainMotor.getMotorVoltage();
    motorVoltageSignal.setUpdateFrequency(50);

    motorTemperatureSignal = mainMotor.getDeviceTemp();
    motorTemperatureSignal.setUpdateFrequency(50);

    closedLoopErrorSignal = this.mainMotor.getClosedLoopError();
    closedLoopErrorSignal.setUpdateFrequency(50);

    simState = mainMotor.getSimState();

    sysIdRoutine = SysIdBuilder.buildTalonFXRoutine(
        mainMotor, this, subsystemName, stepVoltage
    );        
   }

  public static TalonFXConfiguration mechanismConfigs(double P, double I, double D, double V, double S, double statorLimit, double supplyLimit, double supplyLowerLimit, NeutralModeValue neutralMode){
    TalonFXConfiguration config = new TalonFXConfiguration();

    Slot0Configs slot0 = config.Slot0;
        slot0.kP = P;
        slot0.kI = I;
        slot0.kD = D;
        slot0.kV = V;
        slot0.kS = S;

    config.withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of(statorLimit))
                            .withStatorCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(Amps.of(supplyLimit))
                            .withSupplyCurrentLowerLimit(Amps.of(supplyLimit))
                            .withSupplyCurrentLimitEnable(true));

    config.MotorOutput.NeutralMode = neutralMode;

    return config;
  }
 
  public SysIdRoutine getSysIdRoutine() {
    return sysIdRoutine;
  }    

  /*
   * Tells the motor to run with a specific voltage
   * The Advantage: If you request 6 Volts, the Talon will push exactly 6 Volts whether the 
   * battery is fresh or dying. This guarantees consistent behavior across the entire match.
   *
   * When to use it: Highly recommended over DutyCycleOut for almost everything. 
   * It is essential for SysId characterization and open-loop drivetrains.
   * 
   * Volts.of(volts) to convert double to Voltage
   *
   */
  public void runWithVoltage(Voltage volts) {
    mainMotor.setControl(voltageRequest.withOutput(volts));
  }

  /*
   * What it does: You command a target speed in Rotations Per Second (RPS). 
   * If a heavy game piece enters the mechanism and bogs the motor down, the PID loop will instantly detect the speed drop and aggressively spike the voltage to get back to the target RPS.
   *
   * When to use it: Anything that needs to maintain a constant speed under varying loads. 
   * Flywheels and Swerve Drive wheels.
   *    
   * RevolutionsPerSecond.of(velocity); to convert double to AngularVelocity
   *
   */
  public void runAtSpeed(AngularVelocity velocity) {
    mainMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  /*
   * What it does: You command a target location in Rotations. The motor will drive to that exact sensor 
   * count and physically hold itself there, fighting anyone who tries to push it off target.
   *
   * When to use it: Mechanisms that move to specific setpoints but are relatively lightweight or 
   * have short travel distances. Adjustable hoods, swerve steering (azimuth) motors, and small turrets.
   *    
   * Degrees.of(degrees) to convert double to Angle
   *
   */
  public void runToLocation(Angle angle) {
    mainMotor.setControl(positionRequest.withPosition(angle));
  }

  /*
   * What it does: This is a "Profiled" position request. If you use standard PositionVoltage to move 
   * a heavy elevator 10 rotations, the PID loop will violently slam full power instantly, which can 
   * snap chains and sheer gear teeth. MotionMagic calculates a smooth "Trapezoidal Profile"—it smoothly 
   * ramps up the acceleration, cruises at a max speed, and smoothly decelerates as it approaches the 
   * target.
   *
   * When to use it: Any heavy mechanism with physical limits. Elevators, heavy pivoting arms, 
   * and large climbing mechanisms.
   *    *    
   * Degrees.of(degrees) to convert double to Angle
   *
   */
  public void runToLocationMagicMotion(Angle angle) {
    mainMotor.setControl(motionMagicRequest.withPosition(angle));
  }

  public void runToLocationEncoder(double pos){
    mainMotor.setControl(voltageRequest.withOutput(getEncoderOutput(pos)));
  }

  public void stopMotor(){
    mainMotor.stopMotor();
  }

  protected void logDouble(String key, double value) {
    RobotLogger.logDouble(getNTKey() + key, value);
  }

  protected void logBoolean(String key, Boolean value) {
    RobotLogger.logBoolean(getNTKey() + key, value);
  }

  protected void logString(String key, String value){
    RobotLogger.logString(key, value);
  }

  public <U> void logStruct(String key, edu.wpi.first.util.struct.Struct<U> structType, U value) {
    RobotLogger.logStruct(getNTKey() + key, structType, value);
  }

  public <U> void logStructArray(String key, edu.wpi.first.util.struct.Struct<U> structType, U[] value) {
    RobotLogger.logStructArray(getNTKey() + key, structType, value);
  }

  protected void logCoreMotorMetrics() {
    T actualMetric = getActualMetric();
    T targetMetric = getTargetMetric();
    RobotLogger.logDouble(getNTKey() + "MotorVoltage", getMotorVoltage().in(Volts));   
    RobotLogger.logDouble(getNTKey() + "Motor Temp (F)", getMotorTemperature().in(Fahrenheit));
    RobotLogger.logDouble(getNTKey() + "StatorCurrent", getStatorCurrent().in(Amps));
    RobotLogger.logDouble(getNTKey() + "SupplyCurrent", getSupplyCurrent().in(Amps));
    RobotLogger.logDouble(getNTKey() + "Target" + targetMetric.unit().name(), targetMetric.magnitude());    
    RobotLogger.logDouble(getNTKey() + "Actual" + actualMetric.unit().name(), actualMetric.magnitude());        
  }

  public String getNTKey() {
    return subsystemName + "/";
  }

  /**
   * Retrieves the current actual position of the mechanism in Degrees.
   */
  public Angle getActualPosition() {
    return getPosition();
  }

  /**
   * Retrieves the current target position of the mechanism in Degrees.
   */
  public Angle getTargetPosition(){
    return positionRequest.getPositionMeasure();
  }

  /**
   * Retrieves the current actual velocity of the mechanism in RPM.
   */
  public AngularVelocity getActualVelocity() {
    return getVelocity();    
  }

  /**
   * Retrieves the current target velocity of the mechanism in RPM
   */
  public AngularVelocity getTargetVelocity(){
    return velocityRequest.getVelocityMeasure();
  }

  /**
  * Retrieves the stator current
  */
  public Current getStatorCurrent() {
    return statorCurrentSignal.refresh().getValue();    
  }

  /**
  * Retrieves the supply current
  */
  public Current getSupplyCurrent() {
      return supplyCurrentSignal.refresh().getValue();    
  }

  /**
  * Retrieves the position of the mechanism controlled by the motor
  */
  public Angle getPosition() {
    return motorPositionSignal.refresh().getValue();    
  }

  /**
  * Retrieves the velocity of the motor
  */
  public AngularVelocity getVelocity() {
    return motorVelocitySignal.refresh().getValue();    
  }

  /**
  * Retrieves the voltage being applied to the motor
  */
  public Voltage getMotorVoltage() {
    return motorVoltageSignal.refresh().getValue();    
  }

    /**
  * Retrieves the temperature being applied to the motor
  */
  public Temperature getMotorTemperature() {
    return motorTemperatureSignal.refresh().getValue();    
  }

  /**
  * Retrieves the closed loop error of the motor
  */
  public Double getClosedLoopError() {
      return closedLoopErrorSignal.refresh().getValue();    
  }

  public double getActualEncoderPos(){
    return encoder.getCanCoderPos();
  }

  public double getTargetEncoderPos(){
    return encoder.getTargetPos();
  }

  public double getEncoderOutput(double pos){
    return encoder.getOutput(pos);
  }

  public boolean atGoal(double tolerance){
    return Math.abs(getActualMetric().magnitude() - getTargetMetric().magnitude()) > tolerance;
  }

  public void logEncoderMetrics(){
    encoder.logEncoderMetrics(getNTKey());
  }
  
  protected void configureMotorForSysId() {
      // --- THE SYSID FIX: FORCE HIGH-SPEED DATA LOGGING ---
      // We tell the motor to send Voltage, Position, and Velocity at 250 Hz (every 4 milliseconds)
      mainMotor.getMotorVoltage().setUpdateFrequency(250.0);
      mainMotor.getPosition().setUpdateFrequency(250.0);
      mainMotor.getVelocity().setUpdateFrequency(250.0);
          
      // (Optional but recommended) Wait for the CAN bus to apply the changes
      try { Thread.sleep(250); } catch (InterruptedException e) {}
  }

    @Override
    public void simulationPeriodic() {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        double appliedVolts = simState.getMotorVoltage();

        // 4. Pass the voltage and the 20ms loop time to the child class!
        updateMechanismSimulation(appliedVolts, 0.020);
    }


  //Abstract methods that the inheritor must define

  /** Implement Subsystem Specific Metrics (e.g. states, booleans) */
  protected abstract void logSubsystemMetrics();

  /** Returns the current actual sensor reading as a raw double for graphing */
  protected abstract T getActualMetric();

  /** Return the current target pid reading as a raw double for graphing (only use for PID Loops)*/
  protected abstract T getTargetMetric();

 /**
  * Called automatically during simulationPeriodic.
  * The child class MUST implement this to update its specific WPILib physics model.
  * @param appliedVolts The voltage currently being applied by the TalonFX
  * @param dtSeconds The standard robot loop time (0.020 seconds)
  */
  protected abstract void updateMechanismSimulation(double appliedVolts, double dtSeconds);

}