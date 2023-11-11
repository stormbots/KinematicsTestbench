// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HardwareID;
import frc.robot.Constants.RetractConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Clamp;
import frc.robot.commands.Lerp;

public class ArmstrongArmKinematics extends SubsystemBase {
   
  public static enum RetractSolenoidPosition{
    ENGAGED,DISENGAGED
  }
  
  public static enum IntakeSolenoidPosition{
    OPEN,CLOSED
  }

  public static enum PrepareOrExecute{
    PREPARE,
    EXECUTE
  }

  public static enum PlaceOrPrepareOrExecute{
    PLACE,
    PREPARE,
    EXECUTE,
  }

  private RetractSolenoidPosition retractSolenoidPosition = RetractSolenoidPosition.DISENGAGED;
  public IntakeSolenoidPosition intakeSolenoidPosition = IntakeSolenoidPosition.CLOSED;
  public PrepareOrExecute prepareOrExecute  = PrepareOrExecute.PREPARE;
  public PlaceOrPrepareOrExecute placeOrPrepareOrExecute = PlaceOrPrepareOrExecute.PREPARE;
  
  // Angular motors
  public CANSparkMax armMotor = new CANSparkMax(Constants.HardwareID.kArmMotor, MotorType.kBrushless);
  private SparkMaxPIDController armPID = armMotor.getPIDController();
  //public Lerp armAnalogLerp = new Lerp(0, 93, -90, 90); //GET GEAR RATIO!!, this for 54:18 gear ratio
  public DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(Constants.HardwareID.kArmAnalogEncoderChannel);
  //Lerp armksFFLerp = new Lerp(RetractConstants.kMinRetractionInches, RetractConstants.kMaxRetractionInches, ArmConstants.ksFFNear, ArmConstants.ksFFFar);
  //Lerp armkCosFFLerp = new Lerp(RetractConstants.kMinRetractionInches, RetractConstants.kMaxRetractionInches, ArmConstants.kCosFFNear, ArmConstants.kCosFFFar);


  //Extension and retraction
  public CANSparkMax retractMotor = new CANSparkMax(Constants.HardwareID.kRetractMotor, MotorType.kBrushless);
  private SparkMaxPIDController retractPID = retractMotor.getPIDController();
  //Lerp retractFFLerp = new Lerp(0, 30, RetractConstants.ksFFNear, RetractConstants.ksFFFar); //inches/rotations to ff output


  // Intake Actuation
  public CANSparkMax intakeMotor = new CANSparkMax(Constants.HardwareID.kIntakeMotor, MotorType.kBrushless);
  public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, HardwareID.kIntakeForwardSolenoid, Constants.HardwareID.kIntakeReverseSolenoid); 
  public Solenoid brakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH,Constants.HardwareID.kRetractBrakeSolenoid);//temp value
  
  // Wrist Actuation
  // public Servo wristServo = new Servo(Constants.HardwareID.kWristMotorID);
  public CANSparkMax wristMotor = new CANSparkMax(HardwareID.kWristMotorID, MotorType.kBrushless);
  public DutyCycleEncoder wristAbsEncoder = new DutyCycleEncoder(HardwareID.kWristAnalogEncoderChannel);
  private double wristSetpoint =0.0;
  private double armSetpoint=0.0;
  private double retractSetpoint =0.0;
  
     
   Field2d field = new Field2d();
 
  /** Creates a new ArmstrongArmKinematics. */
  public ArmstrongArmKinematics() {
      //Prepare extension motors
      retractMotor.setIdleMode(IdleMode.kBrake);
      retractMotor.setSmartCurrentLimit(80);
      retractMotor.getEncoder().setPositionConversionFactor(1); //Set to use rotations for base unit
      retractMotor.setSoftLimit(SoftLimitDirection.kForward, RetractConstants.kRetractSoftLimitForward);
      retractMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      retractMotor.setSoftLimit(SoftLimitDirection.kReverse, RetractConstants.kRetractSoftLimitReverse);
      retractMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      retractMotor.setClosedLoopRampRate(0.05);
      retractMotor.setOpenLoopRampRate(0.05);
  
      //Prepare Arm control
      // armMotor.restoreFactoryDefaults(); //CAN/ID jank should be fixed, this can be removed.
      armMotor.setIdleMode(IdleMode.kBrake);
      armMotor.setSmartCurrentLimit(80);
      armMotor.setInverted(true);
      armMotor.getEncoder().setPositionConversionFactor(ArmConstants.kMotorEncoderConversionFactor);
      armMotor.getEncoder().setVelocityConversionFactor(ArmConstants.kMotorEncoderConversionFactor/60.0);
      armMotor.getEncoder().setPosition(getArmAngleAbsolute());
      armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kSoftLimitForwardNear);
      armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kSoftLimitReverseNear);
      armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      armMotor.setClosedLoopRampRate(0.05);
      armMotor.setOpenLoopRampRate(0.05);
      //Prepare wrist control
      wristMotor.setIdleMode(IdleMode.kCoast);
      wristMotor.setSmartCurrentLimit(12,15);
      wristMotor.setInverted(WristConstants.kReverseMotor);
      wristMotor.getEncoder().setPositionConversionFactor(WristConstants.kConversionFactor);
      if(wristAbsEncoder.isConnected()){
        wristMotor.getEncoder().setPosition(getWristAngleAbsolute());
      }
      wristMotor.setSoftLimit(SoftLimitDirection.kForward, WristConstants.kMaxAngle); 
      wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WristConstants.kMinAngle); 
      wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      wristMotor.setClosedLoopRampRate(0.05);
      wristMotor.setOpenLoopRampRate(0.05);
  
      // wristServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Keeping in case we have to hack in wrist for practice bot
  
  
      //Set all the PIDS
      armPID.setP(ArmConstants.kPNear);
      armPID.setI(ArmConstants.kINear);
      armPID.setD(ArmConstants.kDNear);
      armPID.setFF(0);
  
      retractPID.setP(RetractConstants.kPNear);
      retractPID.setI(RetractConstants.kINear);
      retractPID.setD(RetractConstants.kDNear);
      retractPID.setIZone( 0.1 / retractPID.getI()); // If we use an I term, constrain to 10% output
      retractPID.setFF(0);
  
      wristMotor.getPIDController().setP(WristConstants.kP);
      wristMotor.getPIDController().setI(WristConstants.kI);
      wristMotor.getPIDController().setD(WristConstants.kD);
      wristMotor.getPIDController().setFF(0);
  
      //Set initial positions/recalibrations
      setRetractBrake(RetractSolenoidPosition.DISENGAGED);
  
  
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    SmartDashboard.putNumber("ik/armAngle", getArmAngle());
    SmartDashboard.putNumber("ik/wristAngle", getWristAngle());
    if(getRetractRotations()<-0.1){retractMotor.getEncoder().setPosition(-0.1);}
  }
    
  public double getX(SimpleMatrix matrix) {
    double x = matrix.get(0, 2);
    return x;
  }
  public double getY(SimpleMatrix matrix) {
    double y = matrix.get(1, 2);
    return y;
  }
  public SimpleMatrix getHomogeneousMatrix(Double angleRad, Double x, Double y){
    double[][] m0 = {
    {Math.cos(angleRad),-Math.sin(angleRad),x},
    {Math.sin(angleRad),Math.cos(angleRad),y},
    {0,0,1}
    };
    SimpleMatrix sm0 = new SimpleMatrix(m0);
    return sm0;
  }
  public double getRetractRotations(){
    return retractMotor.getEncoder().getPosition()/retractMotor.getEncoder().getPositionConversionFactor();
  }
  
  public double getRetractLengthIn(){
    double rotations = getRetractRotations();
    var id = RetractConstants.kInnerDiameter;
    var strapwidth = RetractConstants.kStrapWidth;
    var od = 2*rotations*strapwidth + id ; 
    var area = Math.PI/4.0*od*od - Math.PI/4.0*id*id;
    var length = area/strapwidth;
    return length;
  }

  
  public double getRetractLengthM(){
    return Units.inchesToMeters(getRetractLengthIn());
  }

  public double getArmAngleAbsolute(){
    // var angle = armAbsEncoder.getAbsolutePosition()*360-Constants.ArmConstants.kAbsoluteAngleOffset;
    var angle = armAbsEncoder.getAbsolutePosition()*Math.PI*2;
    angle -= Math.toRadians( Constants.ArmConstants.kAbsoluteAngleOffset );
    angle = MathUtil.angleModulus(angle);
    angle = Math.toDegrees(angle);

    if( angle < -90 ){ //TODO should be hardcodedd -90, as it's a physical value not related to offset
      return angle += 360; 
    }
    return angle;
  }

  public double getArmAngle() {
    return armMotor.getEncoder().getPosition();
  }
  public double getWristAngleAbsolute(){
    var angle = wristAbsEncoder.getAbsolutePosition()*360-WristConstants.kAbsoluteAngleOffset;
    if( angle > 180 ){
      return angle - 360;
    }
    return angle;
  }
 
  /**
   * 
   * @param endEffectorX
   * @param endEffectorY
   * @param wristAngle
   * @return [ext,angle degrees]
   */
  public double[] inverseKinematics( double endEffectorX, double endEffectorY, double wristAngle) {
    double wristLength=11;
    double heightAboveFloorInches= 22; 
    wristAngle = Math.toRadians(wristAngle);
  //   double distance = Math.sqrt(Math.pow(getX(endEffector), 2) + Math.pow(getY(endEffector), 2));
  //   double q2a = Math.acos(distance*distance - (Math.pow(lengthOne, 2) + Math.pow(lengthTwo, 2))/(2*firstJoint.get(0, 2)*secondJoint.get(0, 2)));
  //   double q2b= Math.PI - q2a;
  //   var q2 = q2a; //figure out which one we want

  //   double q1 = Math.atan(endEffector.get(1, 2)/endEffector.get(0, 2)) - Math.atan(lengthTwo*Math.sin(q2)/(lengthOne+lengthTwo*Math.cos(q2)));
    double x2 = wristLength*Math.cos(wristAngle);
    double y2 = wristLength*Math.sin(wristAngle);
    double x1 = endEffectorX - x2;
    double y1 = endEffectorY - (heightAboveFloorInches + y2);
    double l1 = Math.sqrt((x1*x1)+(y1*y1));
    double armAngle = Math.atan2(y1, x1);
    armAngle = Math.toDegrees(armAngle);
    
    double returnValues[] = new double [3];
    returnValues[0] = l1-14; // distance
    returnValues[1] = armAngle; //angle degrees
    returnValues[2] = Math.toDegrees(wristAngle); //angle
  
    return returnValues;
   }

   //public void forwardKinematics(double wristAngle, double heightAboveFloorInches, double armAngle, double l1){}
   
   public boolean isWristOnTarget(double tolerance){
    if(wristSetpoint >= getWristAngle() && getWristAngleAbsolute() >= WristConstants.kMaxAngle -7 ){
      return true;
    }
    if(wristSetpoint <= getWristAngle() && getWristAngleAbsolute() <= WristConstants.kMinAngle+7){
      return true;
    }

    return Clamp.bounded(getWristAngle(),
    wristSetpoint-tolerance, 
    wristSetpoint + tolerance )
    ;
  }
  public double getWristAngle(){
    var angle = getArmAngle()+wristMotor.getEncoder().getPosition();
    return angle;
  }

  public boolean isArmOnTarget(double tolerance){
    return Clamp.bounded(getArmAngle(),
    armSetpoint-tolerance, 
    armSetpoint + tolerance )
    ;
  }

  public boolean isRetractOnTarget(double retractTolerance){
    return Clamp.bounded(getRetractRotations(),
    retractSetpoint - retractTolerance, 
    retractSetpoint + retractTolerance )
    ;
  }

  public Boolean isRobotOnTarget(double armTolerance, double retractTolerance, double wristTolerance ){
    return isArmOnTarget(armTolerance)  && isRetractOnTarget(retractTolerance) && isWristOnTarget(wristTolerance);
  }

  public void setArmPID(double setpoint){
    this.armSetpoint = setpoint;
    var armFF =Lerp.lerp(getRetractRotations(),
      RetractConstants.kRetractSoftLimitReverse, RetractConstants.kRetractSoftLimitForward, 
      ArmConstants.kCosFFNear, ArmConstants.kCosFFFar)/12.0;
    armFF*=Math.cos(Math.toRadians(getArmAngle()));
    armPID.setReference(setpoint, ControlType.kPosition, 0, armFF, ArbFFUnits.kVoltage);
  }

  public void setWristPID(double angle){
    this.wristSetpoint = angle;
    angle -= getArmAngle();
    var ff = WristConstants.kFFCos/12.0;
    ff *= Math.cos(Math.toRadians(getWristAngle()));
    wristMotor.getPIDController().setReference(angle, ControlType.kPosition, 0, ff, ArbFFUnits.kVoltage);
  }
  
  public void setRetractPID(double setpoint){
    this.retractSetpoint =setpoint;
    var retractFF = Lerp.lerp(getRetractRotations(), 
      RetractConstants.kRetractSoftLimitReverse, RetractConstants.kRetractSoftLimitForward, 
      RetractConstants.ksFFNear, RetractConstants.ksFFFar)/12.0;
    retractPID.setReference(setpoint, ControlType.kPosition, 0, retractFF, ArbFFUnits.kVoltage);
  }

  public void setRetractBrake(RetractSolenoidPosition mode){
    this.retractSolenoidPosition = mode;
    if(mode == RetractSolenoidPosition.ENGAGED){
      brakeSolenoid.set(RetractConstants.ENGAGED);
    }
    else{
      brakeSolenoid.set(RetractConstants.DISENGAGED);
    }
  }

  

}
