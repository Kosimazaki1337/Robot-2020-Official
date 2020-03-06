/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class AimToTarget extends CommandBase {
  
  private PIDController controller;
  double yOffset;
  double target;
  double error;

  public AimToTarget(double target) {
    this.target = target;
    addRequirements(Robot.aiming);
    addRequirements(Robot.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.limelight.changePipeline(0);
    controller = new PIDController(0.076, 0.045, 0.0006);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yOffset = Robot.limelight.getYOffset();
    error = target - yOffset;

    Robot.aiming.setAimSpeed(controller.calculate(yOffset, target));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setAimSpeed(0.0);
    if(Math.abs(error) < Math.abs(Constants.kYAllowedError)){
        Robot.leds.setLedColorLED(Color.kBlue);
    }

    Robot.aiming.setpositionToHold2();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(error) < Math.abs(Constants.kYAllowedError);
  }
}
