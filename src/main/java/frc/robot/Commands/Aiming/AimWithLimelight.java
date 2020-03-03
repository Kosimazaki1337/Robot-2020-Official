/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveTrain.ControlState;
import frc.robot.motion.SpeedPID;

public class AimWithLimelight extends CommandBase {
  double distance, xOffset, yOffset;

  double stage = 0;
  int flag = 0;

  private PIDController leftController;
  private PIDController rightController;

  Constants constants;

  public AimWithLimelight() {
    addRequirements(Robot.limelight);
    addRequirements(Robot.aiming);
    addRequirements(Robot.driveTrain);

    constants = new Constants().getConstants();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.driveTrain.changeControlState(ControlState.AIMING);

    leftController = new PIDController(constants.kPLimeAim,
    constants.kILimeAim, constants.kDLimeAim,
    constants.kLongCANTimeoutSec);
    leftController.setTolerance(constants.openLoopErrorTolerance);

    rightController = new PIDController(constants.kPLimeAim,
    constants.kILimeAim, constants.kDLimeAim,
    constants.kLongCANTimeoutSec);
    rightController.setTolerance(constants.openLoopErrorTolerance);

    leftController.reset();
    rightController.reset();

    Robot.limelight.changePipeline(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = Robot.limelight.getDistance();
    xOffset= Robot.limelight.getXOffset();
    yOffset = Robot.limelight.getYOffset();

    if(stage == 0){
      Robot.driveTrain.setSpeed(leftController.calculate(xOffset, 0.0),
      rightController.calculate(xOffset, 0.0));
      if(Math.abs(xOffset) < Math.abs(constants.xOffsetAllowedErrorStageOne)){
        stage = 1;
      }
    }else if(stage == 1){
      Robot.limelight.changePipeline(1);

      if(flag == 0){
        leftController = new PIDController(constants.kPLimeAimZoom2,
        constants.kILimeAimZoom2, constants.kDLimeAimZoom2,
        constants.kLongCANTimeoutSec);
        leftController.setTolerance(constants.openLoopErrorTolerance);

        rightController = new PIDController(constants.kPLimeAimZoom2,
        constants.kILimeAimZoom2, constants.kDLimeAimZoom2,
        constants.kLongCANTimeoutSec);
        rightController.setTolerance(constants.openLoopErrorTolerance);
        flag = 1;
      }
  
      Robot.driveTrain.setSpeed(leftController.calculate(xOffset, 0.0),
      rightController.calculate(xOffset, 0.0));
      if(Math.abs(xOffset) < Math.abs(constants.xOffsetAllowedErrorStageTwo)){
        stage = 2;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.changeControlState(ControlState.OPEN_LOOP);
    Robot.limelight.changePipeline(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 2 && (Math.abs(xOffset) < Math.abs(constants.xOffsetAllowedErrorStageThree));
  }
}
