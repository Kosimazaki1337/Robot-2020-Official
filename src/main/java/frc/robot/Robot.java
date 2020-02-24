/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Subsystems.Aiming;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDState;
import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Transporter;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static OI oi;
  public static SmartDashBoardInput sdbi;
  public static DriveTrain driveTrain;
  public static Aiming aiming;
  public static Shooter shooter;
  public static Transporter transporter;
  public static LimeLight limelight;
  public static Intake intake;
  public static LEDState leds;

  SetAutonomousCommand autonomousCommand;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    initSubsystems();
    oi = new OI();
    sdbi = new SmartDashBoardInput();
    autonomousCommand = new SetAutonomousCommand();
  }

  @Override
  public void robotPeriodic() {
    sdbi.logs();
    CommandScheduler.getInstance().run();
    leds.periodic();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    autonomousCommand.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }

  public void initSubsystems(){
    driveTrain = new DriveTrain();
    intake = new Intake();
    shooter = new Shooter();
    limelight = new LimeLight();
    transporter = new Transporter();
    aiming = new Aiming();
    leds = new LEDState();
  }
}
