/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTime extends CommandBase {
  /**
   * Creates a new DriveTime.
   */

  double time;
  double startTime;
  double targetAngle;
  double power;

  boolean isDriveAngle = false;
  
  public DriveTime(double time, double power) {
    this.time = time;
    this.power = power;
    targetAngle = Robot.driveTrain.getAngle();
    isDriveAngle = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  public DriveTime(double time, double power, double targetAngle){
    this.targetAngle = targetAngle;
    this.time = time;
    this.power = power;
    isDriveAngle = true;

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

    double leftMotor = -a * this.power;
    double rightMotor = a * this.power;

    
    if(isDriveAngle){
      double actualAngle = Robot.driveTrain.getAngle();
      double error = actualAngle - targetAngle;
      double turn = error * Constants.driveAnglekP;
      if(power > 0){
        Robot.driveTrain.setSpeed(leftMotor - turn, rightMotor + turn);
      }else{
        Robot.driveTrain.setSpeed(leftMotor + turn, rightMotor - turn);
      }
    }else{
      Robot.driveTrain.setSpeed(leftMotor, rightMotor);
    }
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
