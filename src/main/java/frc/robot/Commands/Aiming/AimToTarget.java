/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Aiming;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.LEDState.StateLedFlag;

public class AimToTarget extends CommandBase {
  
 // private PIDController controller;
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
    SmartDashboard.putNumber("AimAndShoot", 7);
    Robot.aiming.setMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yOffset = Robot.limelight.getYOffset();
    error = target - yOffset;
    error *= -1;

    Robot.leds.changeLedState(StateLedFlag.AUTO_AIM);

    double pos = Robot.aiming.getPotentometerPosition();

    //error <-1, 26>
    // -1 shooter jest za wysoko trzeba go opusicic  czyli -0.15
    // 26 shooter jest za nisko -> podnosimy (set 0.35)
    if(error > 0){
      Robot.aiming.setpositionToHold2(pos - 0.4);
    } else if(error < 0){
      Robot.aiming.setpositionToHold2(pos + 0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.aiming.setMode(false);
    Robot.aiming.setAimSpeed(0.0);
    Robot.leds.changeLedState(StateLedFlag.AIMED);
    if(Math.abs(error) < Math.abs(Constants.kYAllowedError)){
      Robot.leds.changeLedState(StateLedFlag.AIMED);
    }

    Robot.aiming.setpositionToHold2(Robot.aiming.getPotentometerPosition());

    SmartDashboard.putNumber("AimAndShoot", 9);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {   
    SmartDashboard.putNumber("AimAndShoot", 8);

    return Math.abs(error) < Math.abs(Constants.kYAllowedError);
  }
}
