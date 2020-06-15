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
import frc.robot.Commands.Aiming.SetAimingPosition;
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
    public JoystickButton returnToDriveTrainInDeCaseKarciaZepsula;
    public JoystickButton returnToDriveTrainInDeCaseLukaszZepsul;
    public JoystickButton setAimDown;
    public JoystickButton setFirstPosition;
    public JoystickButton setSecondPosition;

    public OI(){
        constants = new Constants();

        driver = new Joystick(0);
        operator = new Joystick(1);

        shootAutoAim =new JoystickButton(operator, 1);
        shootAutoAim.whenPressed(new AimSequence());

        setAimDown = new JoystickButton(operator, 2);
        setAimDown.whenPressed(new SetAimingPosition(Robot.limelight.getYOffset()));

        intakeMove = new JoystickButton(operator, 4);
        intakeMove.whenPressed(new IntakeMove());

        shootTeleop = new JoystickButton(operator, 3);
        shootTeleop.whenHeld(new ButtonShoot());

        returnToDriveTrainInDeCaseKarciaZepsula = new JoystickButton(driver, 3);
        returnToDriveTrainInDeCaseKarciaZepsula.whenPressed(new OpenLoopDrive());

        returnToDriveTrainInDeCaseLukaszZepsul = new JoystickButton(operator, 8);
        returnToDriveTrainInDeCaseLukaszZepsul.whenPressed(new OpenLoopDrive());

        setFirstPosition = new JoystickButton(operator, 10);
        setFirstPosition.whileHeld(new SetAimingPosition(0.34));

    }

    public Joystick getOperatorJoystick(){
        return operator;
      }

    public Joystick getDriverJoystick(){
        return driver;
      }
}
