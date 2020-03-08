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
  private PIDController rightController;

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

    rightController = new PIDController(constants.openLoopkPRight,
    constants.openLoopkIRight, constants.openLoopkDRight,
    constants.kLongCANTimeoutSec);
    rightController.setTolerance(constants.openLoopErrorTolerance);

    Robot.limelight.changePipeline(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftController.reset();
    rightController.reset();
    Robot.shooter.changeShootState(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedV = Robot.oi.getDriverJoystick().getRawAxis(1);
    double turnV = Robot.oi.getDriverJoystick().getRawAxis(4);

    SmartDashboard.putBoolean("ShootingInDrive", Robot.shooter.getShootState());

    if ((Math.abs(speedV) > constants.joyDeadZone || Math.abs(turnV) > constants.joyDeadZone) && !Robot.shooter.getShootState()) {
      
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
      Robot.driveTrain.setSpeed(
          leftController.calculate(Robot.driveTrain.getLeftSpeed(), leftSpeed*4),
          rightController.calculate(-Robot.driveTrain.getRightSpeed(), -rightSpeed*4)
      );
      SmartDashboard.putNumber("leftSpeed", leftSpeed*4);
      SmartDashboard.putNumber("rightSpeed", -rightSpeed*4);
      }else{
        
        Robot.driveTrain.setSpeed(0.0, 0.0);
        leftController.reset();
        rightController.reset();
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
