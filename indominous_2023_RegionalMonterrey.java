// Ultima modificacion por: Armando // Feb 14 7:36


package frc.robot;


import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

import com.fasterxml.jackson.databind.ser.impl.FailingSerializer;

//import java.security.KeyPair;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Codigo para leer el in-built encoder de los neo motors
 * Los Relative Encoder se pueden medir de 2 formas, 
 *  -La velocidad que nos proporciona las RPM (revoluciones por minuto)
 *  -La posicion que nos da el numero de vueltas totales que da el motor
 * 
 * Este codigo se recomienda probarlo solamente con un solo motor
 * 
 * 
 * Codigo obtenido desde Github de rev robotics
 * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Read%20Encoder%20Values/src/main/java/frc/robot/Robot.java
 */


public class Robot extends TimedRobot {
  AHRS ahrs;

  
  double valorEncoderIzq, distanciaRecorridaMetros;
  
  double relacionBasica = 1.09; // Motor / Llanta
  double relacionBaja = 0.47; 
  double relacionAlta = 1.36;

  double diametroLlanta = 0.1524; // Metros
  double relacionUsada;
  double distanciaObjetivo;
  double delta;
  double kOutput;
  final double kConstante = 0.5;

  

  private Joystick m_stick;
  private static final int deviceID = 1;
  
  //Motores chasis
  CANSparkMax m_derAdelante = new CANSparkMax(motorDerAdelante, MotorType.kBrushless);
  CANSparkMax m_derAtras = new CANSparkMax(motorDerAtras, MotorType.kBrushless);
  CANSparkMax m_izqAdelante = new CANSparkMax(motorIzqAdelante, MotorType.kBrushless);
  CANSparkMax m_izqAtras = new CANSparkMax(motorIzqAtras, MotorType.kBrushless);

  //motores mecanismos 
  CANSparkMax m_brazoAbajo = new CANSparkMax(brazoAbajo, MotorType.kBrushless);
  CANSparkMax m_brazoArriba = new CANSparkMax(brazoArriba, MotorType.kBrushless);
  CANSparkMax m_elevadorAbajo = new CANSparkMax(elevadorAbajo, MotorType.kBrushless);
  CANSparkMax m_elevadorArriba = new CANSparkMax(elevadorArriba, MotorType.kBrushless);


  Solenoid solenoideManitas = new Solenoid(PneumaticsModuleType.CTREPCM, kjSolenoideManitas);
  Solenoid solenoideVelocidades = new Solenoid(PneumaticsModuleType.CTREPCM, kSolenoideVelocidades);
  Solenoid solenoideCodo = new Solenoid(PneumaticsModuleType.CTREPCM, kCodo);

  MotorControllerGroup motoresBrazo = new MotorControllerGroup(m_brazoAbajo, m_brazoArriba);
  MotorControllerGroup motoresElevador = new MotorControllerGroup(m_elevadorAbajo, m_elevadorArriba);

  MotorControllerGroup chasisIzq = new MotorControllerGroup(m_izqAdelante, m_izqAtras);
  MotorControllerGroup chasisDer = new MotorControllerGroup(m_derAdelante, m_derAtras);



//variables caja
boolean velocidad;
boolean baja;
boolean alta;

//variables manitas
boolean manitasPosicion;
boolean cerrado;
boolean abierto;

//variables porcentaje velocidad
boolean porcentajeVelocidad;
boolean dosPorciento;
boolean cienPorciento;


  //encoders
  private RelativeEncoder encoderIzq;
  private RelativeEncoder encoderDer;

  private RelativeEncoder encoderBrazo;
  private RelativeEncoder encoderElevador;


 

 

//posicion encoders mecanismos
//medir valores reales*

double pBrazoNodoAbajo = 0.1;
double pBrazoNodoMedio = 0.45;
double pBrazoNodoAlto = 0.9;

  double posicionAbajoElevador = 0.3;
  double posicionMediaElevador = 0.8;
  double posicionAltaElevador = 1.48;

  // pins placer
private static int jSolenoideManitas = 1;
private static int pBrazo = 0;
private static int pElevador = 5;
private static int jNodoAbajo = 3;
private static int jNodoMedio = 4;
private static int jNodoAlto = 5;

//pins driver
private static int jTankDer = 5;
private static int jTankIzq = 0;
private static int jVelocidad2 = 1;
private static int jSolenoidBaja = 2;
private static int jSolenoideAlta = 3;
private static int jBalanceo = 4;



// Variables Motores
private static int motorDerAdelante = 1;
private static int motorDerAtras = 2;
private static int motorIzqAdelante = 3;
private static int motorIzqAtras = 4;
private static int elevadorArriba = 5;
private static int elevadorAbajo = 6;
private static int brazoArriba = 7;
private static int brazoAbajo = 8;

// camaras
private static int camaraElevador = 0; //USB
private static int camaraManitas = 1; //USB
private static int camaraChasis = 2;

//Solenoides
 private static int kjSolenoideManitas = 2;
 private static int kSolenoideVelocidades = 0;
 private static int kCodo = 3;

 //
 Joystick joyDriver = new Joystick(0);
 Joystick joyPlacer = new Joystick(1);

DifferentialDrive chasis = new DifferentialDrive(chasisIzq, chasisDer);





  
 Timer time = new Timer();
    
    double segundo = 1;

  

  @Override
  public void robotInit() {
    System.out.println("Funciona print");
   CameraServer.startAutomaticCapture(camaraChasis);
   CameraServer.startAutomaticCapture(camaraElevador);
   CameraServer.startAutomaticCapture(camaraManitas);

   
    ahrs = new AHRS(SPI.Port.kMXP);
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
  } catch (RuntimeException ex) {
      
  }
    

   
    


  }

  public void RobotPeriodic(){
  distanciaRecorridaMetros = encoderIzq.getPosition() * (Math.PI * diametroLlanta / relacionUsada)/10;
  encoderIzq = m_izqAdelante.getEncoder();
  encoderDer = m_derAdelante.getEncoder();
SmartDashboard.putBoolean("conecttado?", ahrs.isConnected());
SmartDashboard.putBoolean("Velocidad", velocidad);

    
  }


  @Override
  public void teleopInit() {

    ahrs.reset();


    encoderIzq.setPosition(0);
    // encoderIzq.setPositionConversionFactor(factor);
    //SmartDashboard.putNumber("posicion", encoderIzq.getPosition());
   
    
  }

  @Override
  public void teleopPeriodic() {
    //control driver
    chasis.tankDrive(-jTankIzq, jTankDer);

    motoresBrazo.set(0.2*pBrazo);
    motoresElevador.set(0.2*pElevador);

    if(joyDriver.getRawButton(jBalanceo)){
      if(ahrs.getRoll() >= 1){
        chasis.tankDrive(-0.2, 0.2);
      } else if(ahrs.getRoll() < 0) {
        chasis.tankDrive(0.2, 0.2);

      }

    }
    if (joyDriver.getRawButton(jVelocidad2)){
    porcentajeVelocidad = dosPorciento;
    chasis.tankDrive(-0.2*jTankIzq, 0.2*jTankDer);
    } else {
      porcentajeVelocidad = cienPorciento;
      chasis.tankDrive(-jTankIzq, jTankDer);
    }


    if(joyDriver.getRawButton(jSolenoidBaja) && velocidad == alta){
      solenoideVelocidades.set(true);
      relacionUsada = relacionBaja;
      velocidad = alta;
    }

    if (joyDriver.getRawButton(jSolenoideAlta) && velocidad == baja){
      solenoideVelocidades.set(false);
      relacionUsada = relacionBaja;
      velocidad = baja;
    }


    if (joyDriver.getRawButton(jNodoAbajo)){
      if (encoderBrazo.getPosition() >= 0.1 ){
        motoresBrazo.set(-0.2);
      } else {
        motoresBrazo.set(0.0);
      }
      if (encoderElevador.getPosition() >= 0.1){
        motoresElevador.set(-0.2);
    } else {
      motoresElevador.set(0.0);
    }

      
    

    if(joyPlacer.getRawButton(jSolenoideManitas) && solenoideManitas.get() == false){
      solenoideManitas.set(true);
      manitasPosicion = abierto;
    }
    if(joyPlacer.getRawButton(jSolenoideManitas) && solenoideManitas.get() == true){
      solenoideManitas.set(false);
      manitasPosicion = cerrado;
    }
    }
    

    

    

   
    
    //SmartDashboard.putNumber("Normal",divisor);

  
  }
  
  @Override
  public void autonomousInit() {
   
ahrs.reset();
    
   
    
    time.reset();
    time.start();

    

  }

  @Override
  public void autonomousPeriodic() {
   
}




