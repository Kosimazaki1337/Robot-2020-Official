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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class LEDState extends SubsystemBase {
  private AddressableLED LEDS;

  private AddressableLEDBuffer LEDBuffer;

  private int m_rainbowFirstPixelHueL = 1;

  public LEDState() {
    LEDS = new AddressableLED(PortMap.kLedPWM);

    LEDBuffer = new AddressableLEDBuffer(61);

    LEDS.setLength(LEDBuffer.getLength());

    LEDS.setData(LEDBuffer);

    LEDS.start();
  }

  @Override
  public void periodic() {
    //blink();
    //rainbowL();
    
    turnOFF();
    LEDS.setData(LEDBuffer);
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
    m_rainbowFirstPixelHueL += 3;
    // Check bounds
    m_rainbowFirstPixelHueL %= 180;
  }

  private void turnOFF(){
    for (var i = 0; i < LEDBuffer.getLength(); i++) {
      LEDBuffer.setLED(i, Color.kBlack);
    }
  }

  long oldTimeBlink = System.currentTimeMillis();
  Boolean flagBlink = false;

  private void blink(){
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


    private void TeloOP(){
      long nowTime = System.currentTimeMillis();
  
      if(nowTime - oldTimeBlink > 1000){
        flagBlink = !flagBlink;
        oldTimeBlink = nowTime;
      }
      if(flagBlink){
        setRGB(255,140,0);
        //setColor(Color.kRed);
      } else {
        setRGB(0, 0, 0);
        //turnOFF();
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
}
