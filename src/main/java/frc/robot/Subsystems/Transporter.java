/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake.Flag;

public class Transporter extends SubsystemBase {
  
  public enum State{
    INTAKE, SHOOT, STOP
  }
  private I2C.Port i2cPort = I2C.Port.kOnboard;

  private WPI_VictorSPX transportMotor;

  private DigitalInput photoElectric;
  private DigitalInput limitSwitch;
  private Ultrasonic ultrasonic;

  private int countBalls = 0;
  private boolean flag = false;

  private ColorSensorV3 colorSensor;

  State mState = State.INTAKE;

  Constants constants;

  Command run;

  public Transporter(){
    transportMotor = new WPI_VictorSPX(PortMap.kTransportMotor);

    photoElectric = new DigitalInput(PortMap.kPhotoSensor);
    limitSwitch = new DigitalInput(PortMap.kLimitSwitch);
    colorSensor = new ColorSensorV3(i2cPort);
    ultrasonic = new Ultrasonic(3, 4, Unit.kMillimeters);

    transportMotor.configFactoryDefault();

    ultrasonic.setAutomaticMode(true);

    constants = new Constants().getConstants();

    countBalls = 0;
  }

  @Override
  public void periodic() { 
    double output = 0;

    // if(isTransorterFull()){
    //   Robot.limelight.setMode(3);
    // }

    if(!isTransorterFull()){
      if(ultrasonic.getRangeMM()/10 < 25.0){
          output = Constants.getConstants().twoBallsSpeedTransporter;
      }else{
          output = 0;
      }
    }else{
      output = 0;
    }

    if(Robot.shooter.getShootState()){
      output = 0.55;
    }

    if(Robot.oi.getOperatorJoystick().getRawAxis(5) > 0.2){
      output = -Robot.oi.getOperatorJoystick().getRawAxis(5)/0.6;
    }

    if(Robot.oi.getOperatorJoystick().getRawAxis(5) < -0.2){
      output = -Robot.oi.getOperatorJoystick().getRawAxis(5)/0.6;
    }

    if(Robot.intake.getIntakeFlag() == Flag.STOP){
      transportMotor.set(0.0);
    } else {
      transportMotor.set(output);
    }

  }


  public void joyControl(){
    transportMotor.set(Robot.oi.getOperatorJoystick().getRawAxis(5));
  }

  public boolean isBallMiddle(){
    return photoElectric.get();
  }

  public boolean isTransorterFull(){
    return !limitSwitch.get();
  }

  public void stopMotor(){
    transportMotor.set(0);
  }

  public double getUltrasonicDistance(){
    return ultrasonic.getRangeMM()/10;
  }

  public boolean isBallDown(){
    return ultrasonic.getRangeMM()/10 < 20.0;
  }

  public void changeState(State state){
    mState = state;
  }

  public void setPower(double speed){
    transportMotor.set(speed);;
  }

  public void resetBalls(){
    countBalls = 0;
  }

  public void logs(){
    SmartDashboard.putBoolean("transporter_LimitSwitch", isTransorterFull());
    SmartDashboard.putString("transporter_state", mState.toString());
    SmartDashboard.putNumber("transporter_Ultrasonic", ultrasonic.getRangeMM()/10);
  }
}
