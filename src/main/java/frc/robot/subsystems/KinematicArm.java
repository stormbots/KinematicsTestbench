// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KinematicArm extends SubsystemBase {

  //encoders 
  DutyCycleEncoder enc0 = new DutyCycleEncoder(0);
  DutyCycleEncoder enc1 = new DutyCycleEncoder(1);

  Field2d field = new Field2d();

  /** Creates a new KinematicArm. */
  public KinematicArm(){

  }

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run

    //read signals
  var angle0 = -(enc0.getAbsolutePosition())*2;
  //angle0 = MathUtil.angleModulus( Math.PI*2 * Timer.getFPGATimestamp()/5 );
  var angle1 = -(enc1.getAbsolutePosition()*2);
  var angleOffset = Math.toRadians(26);
 
  SimpleMatrix sm0 = getHomogeneousMatrix(angle0-angleOffset, 0.0, 0.0);
  SimpleMatrix sm1 = sm0.mult(getHomogeneousMatrix(angle1, 1.0, 0.0));

  // SmartDashboard.putNumber("arm/degres", getAngleDegrees(sm0));
  // SmartDashboard.putNumber("arm/radians", getAngleRadians(sm0));
  // SmartDashboard.putNumber("arm/X", getX(sm0));
  // SmartDashboard.putNumber("arm/Y", getY(sm0));

  SimpleMatrix fm0 = getHomogeneousMatrix(0.0,4.0,0.0).mult(sm0); //makes it easier to see on the field
  SimpleMatrix fm1 = getHomogeneousMatrix(0.0,4.0,0.0).mult(sm1); 
  field.getObject("sm0").setPose(new Pose2d(getX(fm0), getY(fm0), new Rotation2d(getAngleRadians(fm0))));
  field.getObject("sm1").setPose(new Pose2d(getX(fm1), getY(fm1), new Rotation2d(getAngleRadians(fm1))));

  SmartDashboard.putData(field);
  }

  public SimpleMatrix getHomogeneousMatrix(Double angleRad, Double x, Double y)
  {
    double[][] m0 = {
      {Math.cos(angleRad),-Math.sin(angleRad),x},
      {Math.sin(angleRad),Math.cos(angleRad),y},
      {0,0,1}
    };
    SimpleMatrix sm0 = new SimpleMatrix(m0);
    return sm0;
  }


  public double getX(SimpleMatrix matrix) {
    double x = matrix.get(0, 2);
    return x;
  }
  public double getY(SimpleMatrix matrix) {
    double y = matrix.get(1, 2);
    return y;
  }

  public static double getAngleRadians(SimpleMatrix matrix) {
    double radians = Math.atan2(matrix.get(1,0),matrix.get(0, 0));
    return radians;
  }
  public double getAngleDegrees(SimpleMatrix matrix) {
    double degrees = Math.toDegrees(getAngleRadians(matrix));
    return degrees;
  }

  public double[] getAngle0(SimpleMatrix firstJoint, SimpleMatrix secondJoint, double lengthOne, double lengthTwo, SimpleMatrix endEffector) {
    double distance = Math.sqrt(Math.pow(endEffector.get(0, 2), 2) + Math.pow(endEffector.get(1, 2), 2));
    double q2a = Math.acos(distance*distance - (Math.pow(lengthOne, 2) + Math.pow(lengthTwo, 2))/(2*firstJoint.get(0, 2)*secondJoint.get(0, 2)));
    double q2b= Math.PI - q2a;
    var q2 = q2a; //figure out which one we want

    double q1 = Math.atan(endEffector.get(1, 2)/endEffector.get(0, 2)) - Math.atan(lengthTwo*Math.sin(q2)/(lengthOne+lengthTwo*Math.cos(q2)));
    double[] outputAngle = new double[2];
    outputAngle[0] = q1;
    outputAngle[1] = q2;
    return outputAngle;
  }
  
}
