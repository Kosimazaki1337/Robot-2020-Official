/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Transporter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.Transporter.State;

public class TransporterSetIntakeMode extends InstantCommand {
  public TransporterSetIntakeMode() {
    addRequirements(Robot.transporter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.transporter.changeState(State.INTAKE);
  }
}
