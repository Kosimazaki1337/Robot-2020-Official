/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Aiming.AimSequence;
import frc.robot.Commands.Aiming.SetAimingDown;
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.Drive.OpenLoopDrive;
import frc.robot.Commands.Intake.IntakeBalls;
import frc.robot.Commands.Intake.IntakeMove;
import frc.robot.Commands.Intake.ReverseIntake;
import frc.robot.Commands.Shooter.ButtonShoot;

public class OI {

    Constants constants;

    public Joystick operator;
    public Joystick driver;
    public JoystickButton intakeBalls;
    public JoystickButton intakeMove;
    public JoystickButton shootTeleop;
    public JoystickButton intakeReverse;
    public JoystickButton shootAutoAim;
    public JoystickButton returnToDriveTrainInDeCaseXD2;
    public JoystickButton setAimDown;
    public OI(){
        constants = new Constants();

        driver = new Joystick(0);
        operator = new Joystick(1);

        shootAutoAim =new JoystickButton(operator, 1);
        shootAutoAim.whenPressed(new AimSequence());

        setAimDown = new JoystickButton(operator, 2);
        setAimDown.whenPressed(new SetAimingDown());

        intakeBalls = new JoystickButton(driver, 6);
        intakeBalls.whileHeld(new IntakeBalls());

        intakeMove = new JoystickButton(driver, 4);
        intakeMove.whenPressed(new IntakeMove());

        shootTeleop = new JoystickButton(operator, 3);
        shootTeleop.whenHeld(new ButtonShoot());

        intakeReverse = new JoystickButton(driver, 5);
        intakeReverse.whenHeld(new ReverseIntake());

        returnToDriveTrainInDeCaseXD2 = new JoystickButton(driver, 8);
        returnToDriveTrainInDeCaseXD2.whenPressed(new OpenLoopDrive());

    }

    public Joystick getOperatorJoystick(){
        return operator;
      }

    public Joystick getDriverJoystick(){
        return driver;
      }
}
