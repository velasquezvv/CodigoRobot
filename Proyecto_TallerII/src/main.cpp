#include "pkg/motor.h"
#include "pkg/car.h"
#include "pkg/comm.h"
#include "pkg/magneto.h"
#include "pkg/encoder.h"
#include "Arduino.h"
#include "HC05.h" // Incluye la biblioteca HC05 para comunicación Bluetooth

// -- Definiciones --
// Pines digitales 0 y 1 son RX y TX, respectivamente. Se usan para el módulo Bluetooth

#define MOTOR_A_PIN1 3   // Pin 1 para control del motor A
#define MOTOR_A_PIN2 5   // Pin 2 para control del motor A
#define MOTOR_B_PIN1 6   // Pin 1 para control del motor B
#define MOTOR_B_PIN2 9   // Pin 2 para control del motor B
#define PLANT_PIN 7      // Pin para detectar la presencia de la planta
#define PIN_ENCODER 2    // Pin para el encoder de distancia
#define BT_STATE_PIN 8   // Pin de estado del módulo Bluetooth
#define BT_CMD_PIN 4     // Pin de comando del módulo Bluetooth

// El magnetómetro usa los pines analógicos A5 y A4 (I2C)

// -- Variables Globales -----------------------------------
float posX = 0;          // Posición X inicial
float posY = 0;          // Posición Y inicial
float theta = 0;         // Rotación del carro en radianes

// Inicialización del módulo Bluetooth HC-05
HC05 bt(BT_CMD_PIN, BT_STATE_PIN);

// Buffer para recibir datos por Bluetooth
String btBuffer = "";
boolean commandComplete = false;

// Variables que entran por el bluetooth
float Xcultivo, Ycultivo, RotacionCultivo, RotacionInicial;
float PuntoInicial[2] = {0, 0}, PuntoA[2], PuntoB[2], PuntoC[2]; // Índice 0 es X, índice 1 es Y

// Variables que son calculadas dentro del código
float DistanciaObjetivo, RotacionObjetivo, PosicionActual[2];

// Inicialización del encoder y el magnetómetro
encoder regla;
magneto brujula; // Inicializa el Serial a 9600 por su cuenta

// ------------ Constructor de los motores -------------
DRV8833 driver;
Motor motorA = {MOTOR_A_PIN1, MOTOR_A_PIN2, A};
Motor motorB = {MOTOR_B_PIN1, MOTOR_B_PIN2, B};
// -------------------------------------------------------


// -- Funciones --------------------------------------------

/**
 * Procesa el comando recibido por Bluetooth
 * Formato esperado: $X1,Y1,AnguloInicial;X2,Y2;X3,Y3;XCultivo,YCultivo,AnguloCultivo@
 * También acepta el comando "S" para detener los motores
 */
void procesarComando(String cmd) {
  // Verificar si es un comando válido (debe empezar con $ y terminar con @)
  if (cmd.startsWith("$") && cmd.endsWith("@")) {
    // Eliminar los caracteres de inicio y fin
    cmd = cmd.substring(1, cmd.length() - 1);
    
    // Dividir la cadena por punto y coma para obtener cada segmento
    int separador1 = cmd.indexOf(';');
    int separador2 = cmd.indexOf(';', separador1 + 1);
    int separador3 = cmd.indexOf(';', separador2 + 1);
    
    if (separador1 != -1) {
      // Procesar segmento 1: CoordXA, CoordYA, AnguloRobot
      String segmento1 = cmd.substring(0, separador1);
      int coma1 = segmento1.indexOf(',');
      int coma2 = segmento1.indexOf(',', coma1 + 1);
      
      if (coma1 != -1 && coma2 != -1) {
        PuntoA[0] = segmento1.substring(0, coma1).toFloat();
        PuntoA[1] = segmento1.substring(coma1 + 1, coma2).toFloat();
        RotacionInicial = segmento1.substring(coma2 + 1).toFloat();
      }
      
      // Procesar segmento 2: CoordXB, CoordYB
      if (separador2 != -1) {
        String segmento2 = cmd.substring(separador1 + 1, separador2);
        int coma3 = segmento2.indexOf(',');
        
        if (coma3 != -1) {
          PuntoB[0] = segmento2.substring(0, coma3).toFloat();
          PuntoB[1] = segmento2.substring(coma3 + 1).toFloat();
        }
        
        // Procesar segmento 3: CoordXC, CoordYC
        if (separador3 != -1) {
          String segmento3 = cmd.substring(separador2 + 1, separador3);
          int coma4 = segmento3.indexOf(',');
          
          if (coma4 != -1) {
            PuntoC[0] = segmento3.substring(0, coma4).toFloat();
            PuntoC[1] = segmento3.substring(coma4 + 1).toFloat();
          }
          
          // Procesar segmento 4: CoordXCultivo, CoordYCultivo, AnguloCultivo
          String segmento4 = cmd.substring(separador3 + 1);
          int coma5 = segmento4.indexOf(',');
          int coma6 = segmento4.indexOf(',', coma5 + 1);
          
          if (coma5 != -1 && coma6 != -1) {
            Xcultivo = segmento4.substring(0, coma5).toFloat();
            Ycultivo = segmento4.substring(coma5 + 1, coma6).toFloat();
            RotacionCultivo = segmento4.substring(coma6 + 1).toFloat();
          }
        }
      }
    }
  } else if (cmd == "S") {
    // Comando de parada
    driver.motorAStop();
    driver.motorBStop();
  }
}

/**
 * Envía los datos de posición actual al dispositivo Bluetooth
 * Formato: Velocidad,Angulo,PosX,PosY
 */
void enviarPosicionActual() {
  String mensaje = "";
  mensaje += "0,"; // Velocidad actual (podríamos calcularla si es necesario)
  mensaje += String(RadToGrados(brujula.DireccionActual())) + ","; // Ángulo actual en grados
  mensaje += String(PosicionActual[0]) + ","; // Posición X actual
  mensaje += String(PosicionActual[1]); // Posición Y actual
  
  // Enviar mensaje por Bluetooth
  bt.write(mensaje.c_str());
}

/**
 * Convierte grados a radianes
 */
float GradosToRad(float grados){
   return grados * 0.017453;
}

/**
 * Convierte radianes a grados
 */
float RadToGrados(float rad){
   return rad * 57.29578;
}

/**
 * Calcula la distancia y rotación necesarias para llegar al punto objetivo
 * @param Vector Coordenadas del punto objetivo [X,Y]
 */
void DefinirObjetivos(float Vector[2]){
   DistanciaObjetivo = sqrt(pow((Vector[1] - PosicionActual[1]) ,2) + pow((Vector[0] - PosicionActual[0]) ,2));
   RotacionObjetivo = atan2(Vector[1] - PosicionActual[1] , Vector[0] - PosicionActual[0]);
   if (RotacionObjetivo < 0) {RotacionObjetivo = RotacionObjetivo + 6.2832;} // Si es negativo, suma 2π
}

/**
 * Dirige el robot hacia el objetivo actual
 * Primero rota hasta alinearse con el objetivo, luego avanza
 * Motor A = Izquierdo, Motor B = Derecho
 */
void IrHaciaObjetivos(){
   delay(200);
   // Comenzar rotación, girando los motores en direcciones opuestas
   if((brujula.DireccionActual() - RotacionObjetivo) > 0.034907){ // Rotar a la derecha (>2°)
      driver.motorAForward();
      driver.motorBReverse();
   }
   else if((brujula.DireccionActual() - RotacionObjetivo) < -0.034907){ // Rotar a la izquierda (<-2°)
      driver.motorBForward();
      driver.motorAReverse();
   }

   // Continuar rotación hasta que el robot esté a menos de 2° (0.034907 Radianes) del ángulo objetivo
   while(abs(brujula.DireccionActual() - RotacionObjetivo) > 0.034907){
      delay(10);
      // Verificar si hay comandos de parada mientras giramos
      if (bt.available()) {
        char c = bt.read();
        btBuffer += c;
        
        if (c == '@' || c == 'S') {
          commandComplete = true;
          if (btBuffer == "S") {
            driver.motorAStop();
            driver.motorBStop();
            return;
          }
          btBuffer = "";
        }
      }
   }

   // Detener el robot y esperar un momento
   driver.motorAStop();
   driver.motorBStop();
   delay(200);

   // Moverse hacia adelante hasta estar a menos de 1cm de la distancia objetivo
   driver.motorAForward();
   driver.motorBForward();
   while((DistanciaObjetivo - regla.GetDistancia()) > 0.01){
      delay(10);
      // Enviar actualización de posición cada 500ms
      unsigned long tiempoActual = millis();
      static unsigned long ultimaActualizacion = 0;
      if (tiempoActual - ultimaActualizacion > 500) {
        enviarPosicionActual();
        ultimaActualizacion = tiempoActual;
      }
      
      // Verificar si hay comandos de parada mientras avanzamos
      if (bt.available()) {
        char c = bt.read();
        btBuffer += c;
        
        if (c == '@' || c == 'S') {
          commandComplete = true;
          if (btBuffer == "S") {
            driver.motorAStop();
            driver.motorBStop();
            return;
          }
          btBuffer = "";
        }
      }
   }

   // Detener los motores, y reiniciar el contador de distancia
   driver.motorAStop();
   driver.motorBStop();
   regla.ResetDistancia();
   
   // Actualizar posición actual
   PosicionActual[0] = Vector[0];
   PosicionActual[1] = Vector[1];
   
   // Enviar posición actualizada
   enviarPosicionActual();
}

/**
 * Transforma un vector de coordenadas del sistema local al sistema global
 * Aplica rotación y traslación según la posición y orientación del cultivo
 */
void Local_a_Global(float (*Vector)[2]){
   float SavedVector[2] = {(*Vector)[0], (*Vector)[1]};
   
   (*Vector)[0] = (SavedVector[0] * cos(RotacionCultivo)) - (SavedVector[1] * sin(RotacionCultivo)) + Xcultivo;
   (*Vector)[1] = (SavedVector[0] * sin(RotacionCultivo)) + (SavedVector[1] * cos(RotacionCultivo)) + Ycultivo;
}

/**
 * Función de interrupción para el encoder
 * Se llama cada vez que se detecta un pulso en el pin del encoder
 */
void IncrementarDistEncoder(){
   regla.IncrementarDistancia();
}

/**
 * Lee y procesa los datos recibidos por Bluetooth
 */
void leerDatosBluetooth() {
  while (bt.available()) {
    char c = bt.read();
    btBuffer += c;
    
    // Si recibimos marcador de fin o comando especial
    if (c == '@' || c == 'S') {
      commandComplete = true;
      break;
    }
  }
  
  // Procesar comando completo
  if (commandComplete) {
    procesarComando(btBuffer);
    btBuffer = "";
    commandComplete = false;
  }
}

//---------------------------------------------------------

/**
 * Configuración inicial del sistema
 */
void setup() {
   // Inicializar comunicación serial para depuración
   Serial.begin(9600);
   
   // Inicializar comunicación Bluetooth
   bt.begin(9600);
   
   // Inicializar el pin del encoder, y detectar cada vez que este se activa
   pinMode(PIN_ENCODER, INPUT);
   attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), IncrementarDistEncoder, FALLING);

   pinMode(PLANT_PIN, INPUT);
   
   // Inicio de los motores
   initMotor(driver, &motorA); 
   initMotor(driver, &motorB);
   
   // Posición inicial
   PosicionActual[0] = 0;
   PosicionActual[1] = 0;
}  

/**
 * Bucle principal del programa
 */
void loop() {
   // Verificar si hay datos disponibles por Bluetooth
   leerDatosBluetooth();
   
   // Control de estado de navegación
   static boolean estadoNavegacion = false;
   
   // Comprobar si tenemos coordenadas válidas para iniciar la navegación
   if (!estadoNavegacion && PuntoA[0] != 0 && PuntoA[1] != 0 && PuntoB[0] != 0 && PuntoB[1] != 0) {
     // Convertir todas las coordenadas del sistema local al global
     Local_a_Global(&PuntoA);
     Local_a_Global(&PuntoB);
     if (PuntoC[0] != 0 || PuntoC[1] != 0) {
       Local_a_Global(&PuntoC);
     }
     
     RotacionCultivo = GradosToRad(RotacionCultivo); // Convertir a radianes
     PosicionActual[0] = PuntoInicial[0];
     PosicionActual[1] = PuntoInicial[1];
     
     // Determina el valor del offset para el magnetómetro
     brujula.SetOffsetMagnetometro(GradosToRad(RotacionInicial)); 
     
     estadoNavegacion = true;
   }
   
   // Solo ejecutar la navegación si tenemos coordenadas válidas
   if (estadoNavegacion) {
     // Espera a que la planta se coloque en su lugar
     if (digitalRead(PLANT_PIN) == HIGH) {
       // Navegar al punto A
       DefinirObjetivos(PuntoA);
       IrHaciaObjetivos();
       
       // Navegar al punto B
       DefinirObjetivos(PuntoB);
       IrHaciaObjetivos();
       
       // Si existe, navegar al punto C
       if (PuntoC[0] != 0 || PuntoC[1] != 0) {
         DefinirObjetivos(PuntoC);
         IrHaciaObjetivos();
       }
       
       estadoNavegacion = false; // Reiniciar para esperar nuevas coordenadas
     }
   }
   
   // Enviar datos de posición periódicamente
   unsigned long tiempoActual = millis();
   static unsigned long ultimoEnvio = 0;
   if (tiempoActual - ultimoEnvio > 1000) { // Cada segundo
     enviarPosicionActual();
     ultimoEnvio = tiempoActual;
   }
}
