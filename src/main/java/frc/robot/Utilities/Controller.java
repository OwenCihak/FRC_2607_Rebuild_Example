// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class Controller {

    public final XboxController controller;
    public final JoystickButton a, b, x, y, start, back, rightBumper, leftBumper, rightStick, leftStick;
    public final POVButton upDPad, downDPad, leftDPad, rightDPad;
    public final Trigger leftTrigger, rightTrigger;

    private final double triggerThreshold = 0.5;

    public Controller(int id){
        controller = new XboxController(id);
        //buttons
        a = new JoystickButton(controller, Button.kA.value);
        b = new JoystickButton(controller, Button.kB.value);
        x = new JoystickButton(controller, Button.kX.value);
        y = new JoystickButton(controller, Button.kY.value);
        start = new JoystickButton(controller, Button.kStart.value);
        back = new JoystickButton(controller, Button.kBack.value);

        //bumpers
        leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
        rightBumper = new JoystickButton(controller, Button.kRightBumper.value);

        //joysticks
        leftStick = new JoystickButton(controller, Button.kLeftStick.value);
        rightStick = new JoystickButton(controller, Button.kRightStick.value);

        //dpad
        upDPad = new POVButton(controller, 0);
        downDPad = new POVButton(controller, 180);
        leftDPad = new POVButton(controller, 270);
        rightDPad = new POVButton(controller, 90);

        //triggers
        leftTrigger = new Trigger(triggerThresholdExceeded(Axis.kLeftTrigger.value));
        rightTrigger = new Trigger(triggerThresholdExceeded(Axis.kRightTrigger.value));
    }

    public BooleanSupplier triggerThresholdExceeded(int val){
        return () -> Math.abs(controller.getRawAxis(val)) > triggerThreshold;
    }

    /**
     * @return If A was pressed
     */
    public boolean getA(){
        return a.getAsBoolean();
    }   

    /**
     * @return If B was pressed
     */
    public boolean getB(){
        return b.getAsBoolean();
    }    

    /**
     * @return If X was pressed
     */    
    public boolean getX(){
        return x.getAsBoolean();
    }    

    /**
     * @return If Y was pressed
     */    
    public boolean getY(){
        return y.getAsBoolean();
    }    

    /**
     * @return If the Start button was pressed
     */    
    public boolean getStart(){
        return start.getAsBoolean();
    }  

    /**
     * @return If the Back button was pressed
     */
    public boolean getBack(){
        return back.getAsBoolean();
    }    

    /**
     * @return If the Left Bumper was pressed
     */    
    public boolean getLeftBumper(){
        return leftBumper.getAsBoolean();
    }    

    /**
     * @return if the Right Bumper was pressed
     */    
    public boolean getRightBumper(){
        return rightBumper.getAsBoolean();
    }    
    
    /**
     * @return If the Left Stick was pressed
     */
    public boolean getLeftStick(){
        return leftStick.getAsBoolean();
    }    
    
    /**
     * @return If the Right Stick was pressed
     */
    public boolean getRightStick(){
        return rightStick.getAsBoolean();
    }    
    
    /**
     * @return If Up on the DPad was pressed
     */
    public boolean getUpDPad(){
        return upDPad.getAsBoolean();
    }

    /**
     * @return If Down on the DPad was pressed
     */
    public boolean getDownDPad(){
        return downDPad.getAsBoolean();
    }    
    
    /**
     * @return If Left on the DPad was pressed
     */
    public boolean getLeftDPad(){
        return leftDPad.getAsBoolean();
    }    
    
    /**
     * @return If Right on the DPad was pressed
     */
    public boolean getRightDPad(){
        return rightDPad.getAsBoolean();
    }    
    
    /**
     * @return If the Left Trigger was pressed
     */
    public boolean getLeftTrigger(){
        return leftTrigger.getAsBoolean();
    }

    /**
     * @return If the Right Trigger ick was pressed
     */
    public boolean getRightTrigger(){
        return rightTrigger.getAsBoolean();
    }

    /**
     * Rumbles controller
     * @param intensity How much the controller rumbles
     */
    public void rumbleOn(double intensity){
        controller.setRumble(RumbleType.kBothRumble, intensity);
    }

    /**
     * Rumbles controller 100%
     */
    public void rumbleOnFull(){
        rumbleOn(1);
    }

    /**
     * Stops rumbling controller
     */
    public void rumbleOff(){
        rumbleOn(0);
    }

}
