/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.motion.SpeedPID;

public class OpenLoopDrive extends CommandBase {
  
  private PIDController leftController;
  private PIDController righController;

  private SpeedPID speedPIDLeft, speedPIDRight;

  private double leftSpeed, rightSpeed;

  Constants constants;

  public OpenLoopDrive() {
    addRequirements(Robot.driveTrain);
    constants = new Constants().getConstants();


    leftController = new PIDController(constants.openLoopkPLeft,
    constants.openLoopkILeft, constants.openLoopkDLeft,
    constants.kLongCANTimeoutSec);
    leftController.setTolerance(constants.openLoopErrorTolerance);

    righController = new PIDController(constants.openLoopkPRight,
    constants.openLoopkIRight, constants.openLoopkDRight,
    constants.kLongCANTimeoutSec);
    righController.setTolerance(constants.openLoopErrorTolerance);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedPIDLeft = new SpeedPID(0.1, 0.001, 0.0, 0.0);
    speedPIDRight = new SpeedPID(0.1, 0.001, 0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedV = Robot.oi.getDriverJoystick().getRawAxis(1);
    double turnV = Robot.oi.getDriverJoystick().getRawAxis(4);

    if (Math.abs(speedV) > constants.joyDeadZone || Math.abs(turnV) > constants.joyDeadZone) {
      
      //drive.setSpeed(speedV - turnV, speedV + turnV);

      if(speedV > 0.0) {
        if(turnV > 0.0) {
          leftSpeed = speedV - turnV;
          rightSpeed = Math.max(speedV, turnV);
        } else {
          leftSpeed = Math.max(speedV, -turnV);
          rightSpeed = speedV + turnV;
        }
      } else if(speedV < 0.0){
        if(turnV > 0.0) {
          leftSpeed = -Math.max(-speedV, turnV);
          rightSpeed = speedV + turnV;
        } else {
          leftSpeed = speedV - turnV;
          rightSpeed = -Math.max(-speedV, -turnV);
        }
      }
      Robot.driveTrain.setSpeed(leftController.calculate(Robot.driveTrain.getLeftSpeed(), leftSpeed*4),
      righController.calculate(-Robot.driveTrain.getRightSpeed(), -rightSpeed*4));
      SmartDashboard.putNumber("leftSpeed", leftController.calculate(Robot.driveTrain.getLeftSpeed(), leftSpeed*4));
      SmartDashboard.putNumber("rightSpeed", -rightSpeed);
      }else{
        Robot.driveTrain.setSpeed(0.0, 0.0);
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
    return false;
  }
}
