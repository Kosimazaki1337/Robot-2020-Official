/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Commands.Auto.AimAndShoot;
import frc.robot.Commands.Auto.Shoot3BallsAndTake3BallsTrench;
import frc.robot.Commands.Auto.Shoot3BallsAndTake5BallsTrench;
import frc.robot.Subsystems.Aiming;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDState;
import frc.robot.Subsystems.LimeLight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Transporter;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "DriveStraight";
  private static final String k3B3BT = "ShootAndTake3BallsTrench";
  private static final String k3B5BT = "ShootAndTake5BallsTrench";
  private static final String test = "testAuto";
  private String autoSelected;

  public static OI oi;
  public static SmartDashBoardInput sdbi;
  public static DriveTrain driveTrain;
  public static Aiming aiming;
  public static Shooter shooter;
  public static Transporter transporter;
  public static LimeLight limelight;
  public static Intake intake;
  public static LEDState leds;

  public final SendableChooser<String> autoChooser = new SendableChooser<>();

  Command setAutoCommand;

  @Override
  public void robotInit() {
    autoChooser.setDefaultOption("DefaultDrive", kDefaultAuto);
    autoChooser.addOption("ShootAndTake3BallsTrench", k3B3BT);
    autoChooser.addOption("ShootAndTake5BallsTrench", k3B5BT);
    autoChooser.addOption("TestAuto", test);


    initSubsystems();
    oi = new OI();
    sdbi = new SmartDashBoardInput();
    limelight.setMode(3);
    SmartDashboard.putData(autoChooser);
  }

  @Override
  public void robotPeriodic() {
    targetVsible();
    sdbi.logs();
    CommandScheduler.getInstance().run();
    leds.periodic();

    SmartDashboard.putNumber("BatteryVoltage", RobotController.getBatteryVoltage());
    RobotController.getCANStatus();
  }

  @Override
  public void autonomousInit() {
    autoSelected = SmartDashboard.getString("AutoSelected", test);

    switch (autoSelected) {
      case k3B3BT:
        setAutoCommand = new Shoot3BallsAndTake3BallsTrench();
        setAutoCommand = new SetPathToDrive().goBehindTrench();
        setAutoCommand.schedule();
        break;
      case k3B5BT:
        setAutoCommand = new Shoot3BallsAndTake5BallsTrench();
        setAutoCommand.schedule();
        break;
      case test:
        setAutoCommand = new AimAndShoot();
        setAutoCommand.schedule();
        SmartDashboard.putNumber("AutonomousCase", 1);
      case kDefaultAuto:
      default:
        setAutoCommand = new AimAndShoot();
        setAutoCommand.schedule();
        SmartDashboard.putNumber("AutonomousCase", 1);
        break;
    }
    
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("AutoMode", autoSelected);
  }

  @Override
  public void teleopInit() {
    driveTrain.stopAndReset();
  }

  @Override
  public void teleopPeriodic() {
   //leds.blink();
   leds.teleopLeds();
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
    leds.checkSystems();
    driveTrain.setSpeed(0, 0);
    transporter.setPower(0);
    aiming.setAimSpeed(0);
    shooter.setShootSpeed(0, 0);
  }

  public void targetVsible(){
    if(Robot.limelight.isTargetVisible()){
      Robot.leds.setLedColorLED(Color.kGreen);
    }else{
      Robot.leds.setLedColorLED(Color.kBlack);
    }
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
