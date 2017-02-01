import processing.serial.*;
import processing.opengl.*;

Serial port;
String arduinoData;
float quadAngle = 0.1;
float PLvl = 0;
float ILvl = 0;
float DLvl = 0;

void setup()
  {
    size(2000,1000,P3D);
    smooth();
    port = new Serial(this, "COM6", 115200);
    port.bufferUntil('\n');
  }
  
void draw() {
    background(255);
    fill(0);
    textSize(32);
    text("PID Quadcopter Demo on Arduino and Processing", 1100, 50); 
    
    drawQuad();
    drawP();
    drawI();
    drawD();
  }
  
  void serialEvent (Serial port) {
    //arduinoData = float(port.readStringUntil('\n'));
    //arduinoData = arduinoData*-1;
    //quadAngle = arduinoData * 0.015;
    
    arduinoData = port.readStringUntil('\n');
      
    int A = arduinoData.indexOf("A");
    int B = arduinoData.indexOf("B");
    int C = arduinoData.indexOf("C");
    int D = arduinoData.indexOf("D");
    quadAngle   = (float(arduinoData.substring(0, A)) * 0.015)*-1;
    PLvl        = float(arduinoData.substring(A+1, B))*6;
    ILvl        = float(arduinoData.substring(B+1, C))*3;
    DLvl        = float(arduinoData.substring(C+1, D))*5;
  }
  
  void drawQuad(){
    pushMatrix();
    translate(1300, 500);
    rotate(quadAngle);
    
    fill(250, 213, 89);
    rect(-500, -20, 1000, 40);
    
    fill(0);
    rect(-480, -120, 70, 100);
    
    fill(0);
    rect(410, -120, 70, 100);
    
    fill(100);
    ellipse(0, 0, 20, 20);
    
    fill(40, 66, 239);
    rect(-50, -30, 100, 10);
    
    popMatrix();
  }
  
  void drawP(){
    fill(0);
    textSize(32);
    text("P", 200, 150); 
    
    fill(100);
    rect(200, 200, 70, 600);
    
    fill(255, 0, 0);
    rect(200, 500, 70, PLvl);
  }
  
    void drawI(){
    fill(0);
    textSize(32);
    text("I", 350, 150); 
    
    fill(100);
    rect(350, 200, 70, 600);
    
    fill(255, 0, 0);
    rect(350, 500, 70, ILvl);
  }
  
    void drawD(){
    fill(0);
    textSize(32);
    text("D", 500, 150); 
    
    fill(100);
    rect(500, 200, 70, 600);
    
    fill(255, 0, 0);
    rect(500, 500, 70, DLvl);
  }
  