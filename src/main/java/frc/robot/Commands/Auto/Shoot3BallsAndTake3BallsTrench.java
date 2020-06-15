/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.SetPathToDrive;
import frc.robot.Commands.Aiming.AimSequence;
import frc.robot.Commands.Aiming.SetAimingPosition;
import frc.robot.Commands.Drive.DriveTime;
import frc.robot.Commands.Drive.TurnToAngle;
import frc.robot.Commands.Intake.IntakeBalls;
import frc.robot.Commands.Intake.IntakeDown2;
import frc.robot.Commands.Intake.TurnOnIntake;

public class Shoot3BallsAndTake3BallsTrench extends SequentialCommandGroup {

  public Shoot3BallsAndTake3BallsTrench() {
  super(
    // new AimAndShoot(),
    // new ParallelCommandGroup(new IntakeBalls(), new SetPathToDrive().goFrontTrench(false)),
    // new SetPathToDrive().goFrontTrench(true),
    // new AimAndShoot());

    new ParallelCommandGroup(new IntakeDown2(), new SetAimingPosition(0.38), new DriveTime(0.7, 0.3)),
    new WaitCommand(0.3),
    new TurnToAngle(-10, false),
    new WaitCommand(0.3),
    new AimSequence(),
    new AutoShoot(6),
    new TurnToAngle(10-Robot.driveTrain.getLastAimingTurn(), false),
    new TurnOnIntake(),
    new DriveTime(5, 0.3),
    new DriveTime(0.6, -0.4),
    new TurnToAngle(-10, false),
    new AimSequence(),
    new AutoShoot(6)
  );
}
}
