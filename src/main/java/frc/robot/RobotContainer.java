// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter.Drum;
import frc.robot.Subsystems.Shooter.Hood;
import frc.robot.Utilities.Controller;

public class RobotContainer {

  public final Controller driverController = new Controller(0);

  public final Drum drum = new Drum(this);
  public final Hood hood = new Hood(this);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void blockDuringAuto(){
    if(getAutonomousCommand() != null){
      return;
    }
  }
}
