/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Aiming.AutoAim;
import frc.robot.Commands.Auto.AutoTest;
// import frc.Commands.Shooter.Shoot;
import frc.robot.Commands.Intake.IntakeBalls;
import frc.robot.Commands.Intake.IntakeMove;
import frc.robot.Commands.Shooter.Shoot;

public class OI {

    Constants constants;

    public Joystick driver;
    public JoystickButton intakeBalls;
    public JoystickButton intakeMove;
    public JoystickButton shootTeleop;

    public OI(){
        constants = new Constants();

        driver = new Joystick(0);


        intakeBalls = new JoystickButton(driver, 6);
        intakeBalls.whileHeld(new IntakeBalls());

        intakeMove = new JoystickButton(driver, 5);
        intakeMove.whenPressed(new IntakeMove());

        shootTeleop = new JoystickButton(driver, 3);
        shootTeleop.whenPressed(new AutoTest());

    }

    public Joystick getDriverJoystick(){
        return driver;
      }
}
