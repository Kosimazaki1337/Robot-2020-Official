/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;

public class IntakeMove extends CommandBase {

  Constants constants;
  double oldT;
  double sumT;

  public IntakeMove() {
    addRequirements(Robot.intake);

    constants = new Constants().getConstants();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldT = Timer.getFPGATimestamp();
    sumT = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(constants.intakeSwitch == 0){
      Robot.intake.intakeUp();
    }else if(constants.intakeSwitch == 1){
      Robot.intake.intakeDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(constants.intakeSwitch == 0){
      constants.intakeSwitch = 1;
    }else if(constants.intakeSwitch == 1){
      constants.intakeSwitch = 0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double newT = Timer.getFPGATimestamp();
    sumT += newT - oldT;
    oldT = newT;
    return sumT >= 1.5;
  }
}
