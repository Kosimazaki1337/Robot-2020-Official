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
    aimPID = new SpeedPID(0.183, 0.00923, 0.0038, 0.00115, 0.4, -0.4);
    aimPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double actualPosition = Robot.aiming.getAngle();
    SmartDashboard.putNumber("HoldPosition", positionToHold);
    SmartDashboard.putNumber("actualPotentiometer", actualPosition);
    Robot.aiming.setAimSpeed(aimPID.calculate(positionToHold, actualPosition));
    SmartDashboard.putNumber("Test", 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
    SmartDashboard.putNumber("Test", 3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("isShooting", Robot.aiming.getShootingState());
    return !Robot.shooter.getShootState();
  }
}
