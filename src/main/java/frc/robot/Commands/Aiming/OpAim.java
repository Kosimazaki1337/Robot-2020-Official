/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Subsystems.Aiming;

public class OpAim extends CommandBase {

  public OpAim() {
    addRequirements(Robot.aiming);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.oi.getDriverJoystick().getRawAxis(2) != 0){
      Robot.aiming.setAimSpeed(Robot.oi.getDriverJoystick().getRawAxis(2)/1.5);
    }else if(Robot.oi.getDriverJoystick().getRawAxis(3) != 0){
      Robot.aiming.setAimSpeed(-Robot.oi.getDriverJoystick().getRawAxis(3)/2);
    }else Robot.aiming.setAimSpeed(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
