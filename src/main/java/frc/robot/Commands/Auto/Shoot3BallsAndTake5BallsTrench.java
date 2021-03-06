/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SetPathToDrive;
import frc.robot.Commands.Intake.IntakeBalls;

public class Shoot3BallsAndTake5BallsTrench extends SequentialCommandGroup {
  /**
   * Creates a new Shoot3BallsAndTake5BallsTrench.
   */
  public Shoot3BallsAndTake5BallsTrench() {
    super(
      
      new AimAndShoot(),
      new ParallelCommandGroup(new IntakeBalls(), new SetPathToDrive().goBehindTrench(false)),
      new SetPathToDrive().goBehindTrench(true),
      new AimAndShoot()
    );
  }
}
