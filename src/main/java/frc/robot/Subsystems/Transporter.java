/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
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
import frc.robot.Commands.Transporter.StopTransport;
import frc.robot.Commands.Transporter.TransportBallsFromIntake;
import frc.robot.Commands.Transporter.TransportBallsToShooter;
import frc.robot.Subsystems.Intake.Flag;

public class Transporter extends SubsystemBase {
  public enum State{
    INTAKE, SHOOT, STOP
  }
  private I2C.Port i2cPort = I2C.Port.kOnboard;

  private CANSparkMax transportMotor;
  private CANPIDController mController;
  private CANEncoder neoEncoder;

  private DigitalInput photoElectric;
  private DigitalInput limitSwitch;
  private Ultrasonic ultrasonic;

  private ColorSensorV3 colorSensor;
  private Color detectedColor;

  State mState = State.INTAKE;

  Constants constants;
  /*
  0 - DownStage doesn't see color
  1 - DownStage see color, MiddleStage doesn't
  2 - Wait for middleStage 
  */
  private int transporterFlag = 0;

  Command run;

  public Transporter(){
    transportMotor = new CANSparkMax(PortMap.kTransportMotor, MotorType.kBrushless);
    mController = transportMotor.getPIDController();
    neoEncoder = new CANEncoder(transportMotor);

    photoElectric = new DigitalInput(PortMap.kPhotoSensor);
    limitSwitch = new DigitalInput(PortMap.kLimitSwitch);
    colorSensor = new ColorSensorV3(i2cPort);
    ultrasonic = new Ultrasonic(3, 4, Unit.kMillimeters);

    configureSparkMax(transportMotor);
    resetHallSensor();

    ultrasonic.setAutomaticMode(true);

    constants = new Constants().getConstants();
  }

  @Override
  public void periodic() { 

    if(!isTransorterFull()){
      Robot.intake.changeIntakeFlag(Flag.START);
      SmartDashboard.putNumber("transporter_Stage", transporterFlag);
      if(transporterFlag == 0){
        if(isBallDown()){
          transporterFlag = 1; //set stage 1
          Robot.intake.changeIntakeFlag(Flag.STOP);
        } else if(isBallMiddle()){
          transportMotor.set(constants.oneBallsSpeedTransporter);
        }else stopMotor();
      }
      
      if(transporterFlag == 1){
        transportMotor.set(constants.twoBallsSpeedTransporter); //set power
        if(isBallMiddle()){
          transporterFlag = 2;//set stage 2
        }
      }

      if(transporterFlag == 2){
        if(!isBallMiddle()){
          Robot.intake.changeIntakeFlag(Flag.START);
          transportMotor.set(0.0);
          transporterFlag = 0;
        }
      }  
    } else if(ultrasonic.getRangeMM()/10 >= 18.0){
      stopMotor();
    } else if(ultrasonic.getRangeMM()/10 <= 18.0){
      stopMotor();
      Robot.intake.changeIntakeFlag(Flag.STOP);
    }
  

    //joyControl();
  }

  public void configureSparkMax(CANSparkMax spark){
    spark.setIdleMode(IdleMode.kBrake);
    spark.setClosedLoopRampRate(0.0);
    spark.setControlFramePeriodMs(constants.kLongCANTimeoutMs);
  }

  public void configurePID(CANPIDController controller){
    controller.setP(constants.kPSpark);
    controller.setI(constants.kISpark);
    controller.setD(constants.kDSpark);
    controller.setIZone(constants.kIzSpark);
    controller.setFF(constants.kFFSpark);
    controller.setOutputRange(constants.kSparkMinOutput, constants.kSparkMaxOutput);
  }

  public void joyControl(){
    transportMotor.set(Robot.oi.getDriverJoystick().getRawAxis(5)/5);
  }

  public boolean isBallMiddle(){
    return photoElectric.get();
  }

  public boolean isBallDown(){
    return colorSensor.getIR() >= constants.detectedColorMin;
  }

  public boolean isTransorterFull(){
    return !limitSwitch.get();
  }

  public void stopMotor(){
    transportMotor.set(0);
  }

  public double getHallSensor(){
    return neoEncoder.getPosition();
  }

  public double getTransporterPosition(){
    return neoEncoder.getPosition();
  }
  
  public void resetHallSensor(){
    neoEncoder.setPosition(0.0);
  }

  public void changeState(State state){
    mState = state;
  }

  public void setPower(double speed){
    transportMotor.set(speed);
  }

  public void moveBall(double ticks){
    CANError e = mController.setReference(ticks, ControlType.kPosition);
    SmartDashboard.putNumber("TRANSPORTER_CAN_ERROR", e.value);
  }

  public void logs(){
    SmartDashboard.putBoolean("transporter_PhotoElectric", isBallDown());
    SmartDashboard.putBoolean("transporter_LimitSwitch", isTransorterFull());
    SmartDashboard.putNumber("transporter_neoEncoder", getHallSensor());
    SmartDashboard.putString("transporter_state", mState.toString());
    SmartDashboard.putNumber("DetectedColor", colorSensor.getIR());
    SmartDashboard.putNumber("Ultrasonic", ultrasonic.getRangeMM()/10);
  }
}
