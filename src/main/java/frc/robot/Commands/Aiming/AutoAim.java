/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.SpeedPID;

public class AutoAim extends CommandBase {
  double positionToHold;
  Constants constants;

  SpeedPID aimPID;

  public AutoAim() {
    addRequirements(Robot.aiming);

    constants = new Constants().getConstants();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positionToHold = Robot.aiming.getAngle();
    aimPID = new SpeedPID(0.2, 0.001, 0.0, 0.0, 0.9, -0.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualPosition = Robot.aiming.getAngle();
    SmartDashboard.putNumber("HoldPosition", positionToHold);
    SmartDashboard.putNumber("actualPotentiometer", actualPosition);
    Robot.aiming.setAimSpeed(aimPID.calculate(positionToHold, actualPosition));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("isShooting", constants.getShootingFlag());
    return !constants.getShootingFlag();
  }
}
