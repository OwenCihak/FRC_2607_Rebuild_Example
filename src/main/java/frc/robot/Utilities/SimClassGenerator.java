package frc.robot.Utilities;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimClassGenerator {
    
    public static FlywheelSim getSimFlyWheel(
        DCMotor motor,
        double momentOfInertia,
        double gearRatio
    ) {
        var flywheelPlant = LinearSystemId.createFlywheelSystem(
                motor,                  // Motor
                momentOfInertia,        // J (kg * m^2)
                gearRatio               // Gearing
        );

        // 2. Initialize the simulator with the plant
       return new FlywheelSim(
            flywheelPlant,          // The physics model we just created
            motor,                  // The motor type (used by sim to calculate current draw)
            gearRatio               // The gearing
        );    
    }

    public static SingleJointedArmSim getSimSingleJointedArm(
        DCMotor motor,
        double gearRatio,
        double momentOfInertia,
        double radiusInches,
        double minAngleDegrees,
        double maxAngleDegrees,
        boolean shouldSimulateGravity,
        double startingAngleDegrees        
    ) {
        return new SingleJointedArmSim(
            motor,                                      
            gearRatio,                                  
            momentOfInertia,                            
            Units.inchesToMeters(radiusInches),
            Units.degreesToRadians(minAngleDegrees),   
            Units.degreesToRadians(maxAngleDegrees),    
            shouldSimulateGravity,          
            Units.degreesToRadians(startingAngleDegrees)
        );  
    }

}