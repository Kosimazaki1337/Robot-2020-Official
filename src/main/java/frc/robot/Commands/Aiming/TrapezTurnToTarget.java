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
import frc.robot.Controllers.TurnController;
import frc.robot.Subsystems.LEDState.StateLedFlag;
import frc.robot.motion.TrapezoidalMotionProfile;

public class TrapezTurnToTarget extends CommandBase {
  double xOffset;
  double yOffset;

  Constants constants;

  TrapezoidalMotionProfile profile;
  double allowedError;
  int pipeline;

  TurnController controller;
  boolean isVision = true;

  double offset;

  public TrapezTurnToTarget(int pipeline, double allowedError) {
    isVision = true;
    this.allowedError = allowedError;
    this.pipeline = pipeline;
    addRequirements(Robot.limelight);
    addRequirements(Robot.aiming);
    addRequirements(Robot.driveTrain);
    addRequirements(Robot.leds);
  }

  public TrapezTurnToTarget(int pipeline, double allowedError, double offset) {
    this.offset = offset;
    isVision = false;
    this.allowedError = allowedError;
    this.pipeline = pipeline;
    addRequirements(Robot.limelight);
    addRequirements(Robot.aiming);
    addRequirements(Robot.driveTrain);
    addRequirements(Robot.leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.limelight.changePipeline(pipeline);

    SmartDashboard.putNumber("AimAndShoot", 4);
    
    xOffset = Robot.limelight.getXOffset();
    yOffset = Robot.limelight.getYOffset();

    constants = Constants.getConstants();

    if(isVision){
      profile = new TrapezoidalMotionProfile(xOffset, 40, 30);
    } else {
      profile = new TrapezoidalMotionProfile(offset, 40, 30);
    }

    Robot.driveTrain.setLastAimingTurn(xOffset);
    
    if(pipeline == 0){
      controller = new TurnController(profile, allowedError, Constants.kPLimeAim, Constants.kILimeAim, Constants.kDLimeAim);
    } else  if(pipeline == 1){
      controller = new TurnController(profile, allowedError, Constants.kPLimeAimZoom2, Constants.kILimeAimZoom2, Constants.kDLimeAimZoom2);
    }

  }

  boolean flag = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xOffset = Robot.limelight.getXOffset();
    yOffset = Robot.limelight.getYOffset();

    Robot.leds.changeLedState(StateLedFlag.TRAPEZ_GOING);

    controller.update();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("AimAndShoot", 6);
    Robot.driveTrain.setSpeed(0.0, 0.0);
    if(pipeline == 0){
      Robot.limelight.changePipeline(1);
    } else if (pipeline == 1){
      Robot.limelight.changePipeline(0);
    }
    Robot.leds.changeLedState(StateLedFlag.TRAPEZ_FINISHED);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("AimAndShoot", 5);
    if(isVision){
      return Math.abs(xOffset) <= Math.abs(allowedError);
    } else{
      return Math.abs(Robot.driveTrain.getAngle() - offset) <= Math.abs(allowedError);
    }
  }
}
