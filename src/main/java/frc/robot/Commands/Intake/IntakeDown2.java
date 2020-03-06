/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Subsystems.LEDState.StateLedFlag;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeDown2 extends InstantCommand {
  public IntakeDown2() {
    addRequirements(Robot.intake);
    addRequirements(Robot.leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.intakeDown();
    SmartDashboard.putNumber("AimAndShoot", 0);
    Robot.leds.changeLedState(StateLedFlag.INTAKE_DOWN);
  }
}
