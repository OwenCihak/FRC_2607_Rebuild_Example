// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Settings;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Constants {

    public class DrumConstants{
        public static final int[] MOTOR_IDS = {0, 1, 2, 3}; //first is main motor, rest are followers

        public static final MotorAlignmentValue[] ALIGNMENTS = {
            MotorAlignmentValue.Aligned, //applies to second motor is MOTOR_IDS
            MotorAlignmentValue.Aligned,
            MotorAlignmentValue.Aligned
        };

        public static final double GEAR_RATIO = 1.0;
        
        public static final double STATOR_LIMIT = 120.0;
        public static final double SUPPLY_LIMIT = 70.0;
        public static final double SUPPLY_LOWER_LIMIT = 40.0;

        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 0.0;
        public static final double S = 0.0;

        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        public static final double MOMENT_OF_INERTIA = 0.006;
    }    
    
    public class HoodConstants{
        public static final int MOTOR_ID = 0;

        public static final double GEAR_RATIO = 1.0;
        
        public static final double STATOR_LIMIT = 120.0;
        public static final double SUPPLY_LIMIT = 70.0;
        public static final double SUPPLY_LOWER_LIMIT = 40.0;

        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double S = 0.0;

        public static final double MIN_POS = 0.0;
        public static final double MAX_POS = 45.0;

        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final double MOMENT_OF_INERTIA = 0.006;
    }

}
