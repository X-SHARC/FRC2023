// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.drivers;

import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WS2812Driver extends SubsystemBase {
  /** Creates a new WS2812Driver. */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  int[] rgb = new int[15];
  boolean isRGB = false;
  int breathe = 255;
  boolean breatheReversed = false;
  int breatheH = 10;
  int blinkCount = 0;
  int ll; 

  public WS2812Driver(int dataPort, int ledLength) {
    m_led = new AddressableLED(dataPort);
    m_ledBuffer = new AddressableLEDBuffer(ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    this.ll = ledLength;

    setColor(0, 255, 0);
    //breathe();
    //showPercentage(0.5);
    //blink(0, 255, 0);
    m_led.start();
  }

  @Override
  public void periodic() {
    
  }

  public static void setColor(int r, int g, int b) {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
}


  public void turnOff() {
    setColor(0, 0, 0);
    m_led.setData(m_ledBuffer);
}

  public int[] shiftArray(int[] array){
    int last = array[array.length-1];
    for(int i = array.length-1; i > 0; i--){
      array[i] = array[i-1];
    }
    array[0] = last;
    return array;
  }

  public void toggleRGB(){
    if(!isRGB){
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        rgb[i] = i * 180/m_ledBuffer.getLength();
      }
      isRGB = true;
    }
    shiftArray(rgb);

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, rgb[i], 255, 255); 
    }

    //length -> 180 degrees
    //1 -> 180/length
    //
    m_led.setData(m_ledBuffer);
  }

  public void breathe(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, breatheH, 255, breathe); 
    }
    if(!breatheReversed){
      breathe -= 3;
      if(breathe == 0){
        breatheReversed = true;
        //if(breatheH > 255 || breatheH < 0) breatheH = 5;
        breatheH =  ThreadLocalRandom.current().nextInt(1, 256);
      }
    }
    if(breatheReversed){
      breathe += 3;
      if(breathe == 255) breatheReversed = false;
    }
    
    m_led.setData(m_ledBuffer);
  
  }

  public enum Side { LEFT, RIGHT }

  private double clamp (double n, double min, double max) {
    return n > max ? max : (n < min ? min : n);
  } 

  public void lightOneSide(Side side, int hue) {
    int ledlength = m_ledBuffer.getLength();
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      double temp;
      if (side == Side.RIGHT) {
        temp = (6*i/ledlength) - 1;
      } else {
        temp = (-6*i/ledlength) + 3;
      }
      int brightness = (int) (255 * clamp(temp, 0, 1));
      m_ledBuffer.setHSV(i, hue, brightness, brightness);
    }
    m_led.setData(m_ledBuffer);

  }

  public void setAllLeds(Side s,int r1, int g1, int b1)
  {
    if (s== Side.LEFT){
      for (int i = 0; i < m_ledBuffer.getLength()/2; i++)
      {
        m_ledBuffer.setRGB(i, r1, g1, b1);
      }}
    else {
      for (int i = m_ledBuffer.getLength()-1; i>(m_ledBuffer.getLength()/2)+1; i--)
      {
        m_ledBuffer.setRGB(i, r1, g1, b1);
      }
    }
    m_ledBuffer.setRGB((m_ledBuffer.getLength()/2)-1, 255, 255, 255);
    m_led.setData(m_ledBuffer);
  }

}
