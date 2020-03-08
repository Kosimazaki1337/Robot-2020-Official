/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTime extends CommandBase {
  /**
   * Creates a new DriveTime.
   */

  double time;
  double startTime;
  boolean isForward;
  double power;
  
  public DriveTime(double time, boolean isForward, double power) {
    this.time = time;
    this.isForward = isForward;
    this.power = power;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double a = 1.0;
    if(isForward) a = -1.0;

    double leftMotor = -a * this.power;
    double rightMotor = a * this.power;

    Robot.driveTrain.setSpeed(leftMotor, rightMotor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setSpeed(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double now = Timer.getFPGATimestamp();
    return now - startTime >= time;
  }
}
