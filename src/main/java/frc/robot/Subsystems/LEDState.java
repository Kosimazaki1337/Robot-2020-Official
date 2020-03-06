/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import java.sql.Time;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Robot;

public class LEDState extends SubsystemBase {
  
  public enum StateLedFlag {
    TARGET_VISIBLE, 
    TARGET_NOT_VSIBLE,
    AIMED,
    POLZEI_MODE,
    TEST_MODE,
    FULL_TRANSPORTER,
    SHOOTER_STARTING,
    SHOOTING,
    SHOOTING_END,
    RAINBOW,
    TELEOP_INIT,
    AUTO_INIT,
    ENDGAME_START,
    GAME,
    TRAPEZ_GOING,
    TRAPEZ_FINISHED,
    AUTO_AIM,
    STANDARD,
    INTAKE_DOWN,
    SET_AIMING_POSITION,
    TRANSPORTER_HALF_FULL;
	} 

  private StateLedFlag LedFlag = StateLedFlag.POLZEI_MODE;

  private AddressableLED LEDS;

  private AddressableLEDBuffer LEDBuffer;

  private int m_rainbowFirstPixelHueL = 1;

  

  public LEDState() {
    LEDS = new AddressableLED(PortMap.kLedPWM);

    LEDBuffer = new AddressableLEDBuffer(58);

    LEDS.setLength(LEDBuffer.getLength());

    LEDS.setData(LEDBuffer);

    LEDS.start();
  }

  @Override
  public void periodic() {
    switch(LedFlag){
      case TARGET_VISIBLE:
        setColor(Color.kGreen);
        break;
      case SHOOTING:
        setColor(Color.kViolet);
        break;
      case SHOOTING_END:
        setColor(Color.kDarkViolet);
        break;
      case TRAPEZ_GOING:
        setColor(Color.kRosyBrown);
        break;
      case TRAPEZ_FINISHED:
        setColor(Color.kBrown);
        break;
      case AUTO_AIM:
        setColor(Color.kDarkCyan);
        break;
      case AIMED:
        setColor(Color.kCyan);
        break;
      case INTAKE_DOWN:
        setColor(Color.kOrange);
        break;
      case SET_AIMING_POSITION:
        setColor(Color.kYellow);
        break;
      case STANDARD:
      default:
        setColor(Color.kBlack);
        break;
    }
  }

  public void changeLedState(StateLedFlag stateLedFlag){
    LedFlag = stateLedFlag;
  }

  public void checkSystems(){
    checkDriveTrain();
    checkShooter();
    checkSensors();
  }

  private void rainbowL() {
    // For every pixel
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHueL + (i * 180 / LEDBuffer.getLength())) % 180;
      // Set the value
      LEDBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHueL += 2;
    // Check bounds
    m_rainbowFirstPixelHueL %= 180;
  }

  public void turnOFF(){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setLED(i, Color.kBlack);
    }
  }

  long oldTimeBlink = System.currentTimeMillis();
  Boolean flagBlink = false;

  public void blink(){
    long nowTime = System.currentTimeMillis();

    if(nowTime - oldTimeBlink > 200){
      flagBlink = !flagBlink;
      oldTimeBlink = nowTime;
    }
    if(flagBlink){
      setRGB(100, 0, 0);
      //setColor(Color.kRed);
    } else {
      setRGB(0, 0, 100);
      //turnOFF();
    }
  }


    private void checkDriveTrain(){
      if(Robot.driveTrain.getRightSpeed() < -0.1){
        for(var i = 0; i < 3; i++){
          LEDBuffer.setLED(i, Color.kRed);
        }
      }else if(Robot.driveTrain.getRightSpeed() > 0.1){
        for(var i = 0; i < 3; i++){
          LEDBuffer.setLED(i, Color.kGreen);
        }
      }else{
        for(var i = 0; i < 3; i++){
          LEDBuffer.setLED(i, Color.kBlack);
        }
      }

      if(Robot.driveTrain.getLeftSpeed() < -0.1){
        for(var i = 58; i <58; i++){
          LEDBuffer.setLED(i, Color.kRed);
        }
      }else if(Robot.driveTrain.getLeftSpeed() > 0.1){
        for(var i = 58; i < 58; i++){
          LEDBuffer.setLED(i, Color.kGreen);
        }
      }else{
        for(var i = 58; i < 58; i++){
          LEDBuffer.setLED(i, Color.kBlack);
        }
      }
  }

  public void checkShooter(){
    if(Robot.shooter.getLSpeed() < 0){
      for(var i = 26; i < 29; i++){
        LEDBuffer.setLED(i, Color.kRed);
      }
    }else if(Robot.shooter.getLSpeed() > 0){
      for(var i = 26; i < 29; i++){
        LEDBuffer.setLED(i, Color.kGreen);
      }
    }else{
      for(var i = 26; i < 29; i++){
        LEDBuffer.setLED(i, Color.kBlack);
      }
    }

    if(Robot.shooter.getRSpeed() < 0){
      for(var i = 29; i < 33; i++){
        LEDBuffer.setLED(i, Color.kRed);
      }
    }else if(Robot.shooter.getRSpeed() > 0){
      for(var i = 29; i < 33; i++){
        LEDBuffer.setLED(i, Color.kGreen);
      }
    }else{
      for(var i = 29; i < 33; i++){
        LEDBuffer.setLED(i, Color.kBlack);
      }
    }

  }

  public void ledShooterSpeedLeft(){
    
    double lftLedCount = (Robot.shooter.lMaster.getMotorOutputPercent()*100);
    double counOfOnLeds = (29*lftLedCount)/100;
    if (counOfOnLeds <= 0){
      counOfOnLeds = 1;
    }
    for(int i = 29; i >= counOfOnLeds; i--){
      LEDBuffer.setLED(i, Color.kCoral);
    }
    LEDS.setData(LEDBuffer);
    
  }

  public void ledShooterSpeedRight(){
    
    double lftLedCount = (Robot.shooter.rMaster.getMotorOutputPercent()*100);
    double counOfOnLeds = (29*lftLedCount)/100;
    if (counOfOnLeds <= 0){
      counOfOnLeds = 1;
    }
    for(int i = 56; i >= 29 + counOfOnLeds; i--){
      LEDBuffer.setLED(i, Color.kCoral);
    }
    LEDS.setData(LEDBuffer);
    
  }

  

  public void setLedColorLED( Color color){
    for(var i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setLED(i, color);
    }
    LEDS.setData(LEDBuffer);
  }

  private void checkSensors(){
    if(Robot.transporter.isBallDown()){
      LEDBuffer.setLED(7, Color.kOrange);
    }else{
      LEDBuffer.setLED(7, Color.kDarkRed);
    }

    if(Robot.transporter.isBallMiddle()){
      LEDBuffer.setLED(9, Color.kOrange);
    }else{
      LEDBuffer.setLED(9, Color.kDarkRed);
    }

    if(Robot.transporter.isTransorterFull()){
      LEDBuffer.setLED(11, Color.kOrange);
    }
    else{
      LEDBuffer.setLED(11, Color.kDarkRed);
    }

    if(Robot.transporter.getUltrasonicDistance() != 0){
      LEDBuffer.setRGB(3, 10, 10, (int)Robot.transporter.getUltrasonicDistance()*10);
    }else{
      LEDBuffer.setLED(3, Color.kBlack);
    }
  }

  void setColor(Color c){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setLED(i, c);
    }
  }
  void setRGB(int r, int g, int b){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setRGB(i, r, g, b);
    }
  }

  void setHSV(int h, int s, int v){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setHSV(i, h, s, v);
    }
  }

  int value = 255;

  public void teleopLeds(){
    long nowTime = System.currentTimeMillis();

    if(nowTime - oldTimeBlink > 0.01){
      if(!flag){
        setHSV(12, 255, value);
        value -= 10;
        if(value == 5){
          flag = !flag;
        }
      }else{
        setHSV(12, 255, value);
        value += 10;
        if(value == 255){
          flag = !flag;
        }
      }
    }
    SmartDashboard.putBoolean("Flag", flag);
    SmartDashboard.putNumber("Saturation", value);
  }

  // public void shootningLeds(){
  //   long left = Math.round(Robot.shooter.getLSpeed() / 1.413793103);
  //   for(int i = 28; i > 28-left; i--){
  //     if(i <= 0){
  //       i = 0;
  //     }
  //     LEDBuffer.setRGB(i, 240, 5, 5);
  //   }

  //   long right = Math.round(Robot.shooter.getRSpeed() / 1.413793103);
  //   for(int i = 29; i < 29+right; i++){
  //     if(i >= 57){
  //       i = 57;
  //     }
  //     LEDBuffer.setRGB(i, 240, 5, 5);
  //   }
  // }

  int a = 28;
  int b = 57;
  boolean flag = false;

  public void kaboom(){
    // long nowTime = System.currentTimeMillis();

    // if(Robot.transporter.isTransorterFull() || flag){
    //   flag = true;
    //   LEDBuffer.setLED(a, Color.kDarkRed);
    //   if(nowTime - oldTimeBlink > 0.01){
    //     a -= 1;
    //     b -= 1;
    //     oldTimeBlink = nowTime;
    //   }
    //   if(a <= 25){
    //     LEDBuffer.setLED(a+3, Color.kBlack);
    //   }
    //   if(b <= 54){
    //     LEDBuffer.setLED(b+3, Color.kBlack);
    //   }

    //   if(a == 0 || b == 29){
    //     flag = false;
    //   }
    // }else{
    //   a = 28;
    //   b = 57;;
    // }
  }

}
