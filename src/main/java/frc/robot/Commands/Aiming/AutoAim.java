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
import frc.robot.Subsystems.LEDState.StateLedFlag;
import frc.robot.motion.SpeedPID;

public class AutoAim extends CommandBase {
  double positionToHold;
  Constants constants;

  double yOffset;
  double target;
  double error;

  public AutoAim() {
    addRequirements(Robot.aiming);

    constants = new Constants().getConstants();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // positionToHold = Robot.aiming.getAngle();
    // aimPID = new SpeedPID(0.183, 0.00923, 0.0038, 0.00115, 0.4, -0.4);
    // aimPID.reset();
    SmartDashboard.putNumber("AimAndShoot", 11);
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
    Robot.aiming.setAimSpeed(0.0);
    Robot.aiming.setpositionToHold2(Robot.aiming.getPotentometerPosition());

    //SmartDashboard.putNumber("AimAndShoot", 12);
   // Robot.leds.changeLedState(StateLedFlag.AIMED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Robot.shooter.wasStartedShootState();
    //return ;
  }
}
