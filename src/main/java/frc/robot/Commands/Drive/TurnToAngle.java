/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Controllers.TurnController;
import frc.robot.motion.TrapezoidalMotionProfile;

public class TurnToAngle extends CommandBase {
  TurnController controller;
  TrapezoidalMotionProfile profile;

  double target;
  double startAngle;
  double actualAngle;
  double error;
  boolean isOffset;

  //+ left side, - right side
  public TurnToAngle(double target, boolean isOffset) {
    this.target = target;
    this.isOffset = isOffset;
    addRequirements(Robot.driveTrain);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isOffset){
      actualAngle = Robot.driveTrain.getAngle();

      // target += actualAngle;
    }

    startAngle = Robot.driveTrain.getAngle();

    profile = new TrapezoidalMotionProfile(target, Constants.turnMaxVelocity, Constants.turnMaxAcceleration);
    

    controller = new TurnController(profile, Constants.turnAllowedError, Constants.turnkP, Constants.turnkI, Constants.turnkD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    actualAngle = Robot.driveTrain.getAngle();
    
    controller.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    error = actualAngle - startAngle - target;
    //return !controller.update();
    //SmartDashboard.putBoolean("TurnIsFinished", Math.abs(Robot.driveTrain.getAngle() - target) <= Math.abs(Constants.turnAllowedError));
    SmartDashboard.putNumber("actualAngle", Robot.driveTrain.getAngle());
    SmartDashboard.putNumber("TurnTarget", target);
    SmartDashboard.putNumber("TurnAllowedError", Constants.turnAllowedError);
    SmartDashboard.putNumber("TurnError", error);
    return Math.abs(error) <= Math.abs(Constants.turnAllowedError+1);
  }
}
