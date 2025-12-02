/*
  SISTEMA DE ANÁLISIS DE MARCHA CON IMU 
  
  DESCRIPCIÓN GENERAL:
  Este sistema analiza patrones de marcha humana usando sensores inerciales
  (IMU) colocados en el gemelo. Detecta pasos, mide cadencia, identifica 
  cojera y monitorea la temperatura muscular.
  
  HARDWARE REQUERIDO:
    - Microcontrolador: ESP32-S3
    - IMU: ICM-20948 (acelerómetro + giroscopio + magnetómetro)
    - Sensor térmico: MLX90614 (infrarrojo sin contacto)
    - Ubicación física: Gemelo derecho de la pierna
    - Comunicación: I2C (SDA=GPIO5, SCL=GPIO4)
  
  MÉTRICAS CALCULADAS:
    ✓ Conteo automático de pasos
    ✓ Cadencia (pasos por segundo)
    ✓ Velocidad de marcha estimada (m/s)
    ✓ Energía de impacto en cada paso
    ✓ Suavidad de movimiento (jerk = cambio de aceleración)
    ✓ Fases de apoyo y vuelo del ciclo de marcha
    ✓ Detección de cojera por variabilidad temporal
    ✓ Orientación 3D (roll, pitch, yaw)
    ✓ Temperatura muscular calibrada
    ✓ Calibración automática del IMU
  
  AUTOR: Fabrizio Michele Chiaramonte
  FECHA: Noviembre 2025
*/

// LIBRERÍAS
#include "ICM_20948.h"              // Driver para el IMU ICM-20948
#include <Adafruit_MLX90614.h>       // Driver para sensor térmico infrarrojo
#include <Wire.h>                    // Comunicación I2C

// OBJETOS GLOBALES DE SENSORES
ICM_20948_I2C myICM;                 // Objeto del IMU (acelerómetro, giroscopio, magnetómetro)
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // Objeto del sensor térmico

// CONFIGURACIÓN DE TEMPORIZACIÓN
const long updateInterval = 50;               // Intervalo de actualización (50ms = 20Hz)
unsigned long lastUpdateTime = 0;             // Marca de tiempo de última actualización
unsigned long lastAngleTime = 0;              // Marca de tiempo para integración de ángulos
unsigned long lastSerialCheck = 0;            // Marca de tiempo para chequeo de salud serial
const long serialCheckInterval = 1000;        // Intervalo de chequeo serial (1 segundo)

// VARIABLES DE LECTURAS DEL IMU
// Acelerómetro (en unidades de gravedad 'g')
float accX_g, accY_g, accZ_g;

// Giroscopio (en grados por segundo)
float gyrX_dps, gyrY_dps, gyrZ_dps;

// Magnetómetro (en microteslas)
float magX, magY, magZ;

// CALIBRACIÓN DEL IMU
// Los sensores MEMS tienen pequeños errores de offset (bias) que deben
// corregirse para obtener mediciones precisas. Estos offsets se calculan
// al inicio manteniendo el sensor quieto.

float accX_offset = 0.0;  // Offset del acelerómetro en X
float accY_offset = 0.0;  // Offset del acelerómetro en Y
float accZ_offset = 0.0;  // Offset del acelerómetro en Z (debe ser 1g en reposo)
float gyrX_offset = 0.0;  // Offset del giroscopio en X (debe ser 0 en reposo)
float gyrY_offset = 0.0;  // Offset del giroscopio en Y (debe ser 0 en reposo)
float gyrZ_offset = 0.0;  // Offset del giroscopio en Z (debe ser 0 en reposo)
bool isCalibrated = false; // Bandera que indica si la calibración está completa

// ORIENTACIÓN 3D (ÁNGULOS DE EULER)
// Los ángulos se calculan integrando las velocidades angulares del giroscopio
float roll = 0.0;   // Rotación alrededor del eje X (inclinación lateral)
float pitch = 0.0;  // Rotación alrededor del eje Y (inclinación frontal/trasera)
float yaw = 0.0;    // Rotación alrededor del eje Z (giro horizontal)

// DETECCIÓN DE PASOS
int stepCount = 0;                    // Contador total de pasos detectados
bool inContact = false;               // Bandera: ¿el pie está en contacto con el suelo?
unsigned long contactStartTime = 0;   // Momento en que inició el contacto con el suelo
unsigned long lastContactDuration = 0; // Duración del último contacto (fase de apoyo)
unsigned long lastStepTime = 0;       // Momento del último paso detectado

// Variables para el algoritmo de detección
float totalAcc_prev = 0.0;            // Magnitud de aceleración previa (para detectar cruces)
bool stepDetected = false;            // Bandera: ¿se acaba de detectar un paso?

// UMBRALES DE DETECCIÓN (ajustables según la sensibilidad deseada)
const float STEP_THRESHOLD = 0.65;          // Umbral de desviación de aceleración para detectar paso
const float CONTACT_THRESHOLD = 0.25;       // Umbral para detectar contacto con suelo
const float MIN_STEP_INTERVAL = 0.3;        // Intervalo mínimo entre pasos (segundos) - evita falsos positivos
const float MAX_STEP_INTERVAL = 2.5;        // Intervalo máximo entre pasos (segundos) - descarta pasos muy lentos

// HISTORIAL DE PASOS (para análisis de variabilidad)
#define MAX_STEP_HISTORY 10           // Tamaño del buffer circular de historial

float stepTimeHistory[MAX_STEP_HISTORY];    // Tiempos entre pasos consecutivos
float stepImpactHistory[MAX_STEP_HISTORY];  // Magnitudes de impacto de cada paso
int historyIndex = 0;                       // Índice actual en el buffer circular
int historyCount = 0;                       // Cantidad de pasos registrados (máx. 10)

// Variables estadísticas calculadas del historial
float avgStepTime = 0.0;        // Tiempo promedio entre pasos
float avgStepImpact = 0.0;      // Impacto promedio de los pasos
float timeVariability = 0.0;    // Variabilidad temporal (coeficiente de variación)
float impactVariability = 0.0;  // Variabilidad de impacto (coeficiente de variación)
bool isLimping = false;         // Bandera: ¿se detecta cojera?

// UMBRALES DE DETECCIÓN DE COJERA
// La cojera se manifiesta como irregularidad en el patrón de marcha
const float TIME_VARIABILITY_THRESHOLD = 0.25;    // 25% de variabilidad temporal
const float IMPACT_VARIABILITY_THRESHOLD = 0.30;  // 30% de variabilidad de impacto

// VARIABLES DE IMPACTO
float maxImpact = 0.0;      // Máximo impacto registrado (histórico)
float currentImpact = 0.0;  // Impacto del paso actual

// TEMPERATURA MUSCULAR
float tempAmbiente = 0.0;   // Temperatura ambiente (°C)
float tempObjeto = 0.0;     // Temperatura del objeto/músculo (°C)
const float TEMP_OFFSET = 4.0; // Offset de calibración empírico (+4°C)
// Este valor se ajusta según la distancia del sensor a la piel

// MODO DE SALIDA
int outputMode = 0;  // 0=Processing (CSV), 1=Monitor (legible), 2=Plotter (gráficas)

// MÉTRICAS AVANZADAS DE MARCHA
float cadence = 0.0;          // Cadencia: pasos por segundo (Hz)
float walkingSpeed = 0.0;     // Velocidad estimada de marcha (m/s)
float impactEnergy = 0.0;     // Energía de impacto (proporcional a aceleración²)
float jerk = 0.0;             // "Jerk" = derivada de la aceleración (suavidad del movimiento)
float prevAccMagnitude = 0.0; // Magnitud de aceleración previa (para calcular jerk)
float angularVelocityTotal = 0.0; // Velocidad angular total (magnitud del vector giroscopio)

// FASES DEL CICLO DE MARCHA
// En cada ciclo de marcha hay dos fases: apoyo (pie en suelo) y vuelo (pie en aire)
float stancePhasePercent = 0.0;  // Porcentaje del tiempo en fase de apoyo
float swingPhasePercent = 0.0;   // Porcentaje del tiempo en fase de vuelo

// Contadores de tiempo para las fases
unsigned long totalStanceTime = 0;  // Tiempo acumulado en contacto con suelo
unsigned long totalSwingTime = 0;   // Tiempo acumulado en el aire
unsigned long lastPhaseTime = 0;    // Marca de tiempo del último cambio de fase

// DECLARACIÓN DE FUNCIONES
void readIMUData();                    // Lee datos del IMU y aplica calibración
void integrateAngles();                // Integra giroscopio para obtener ángulos 3D
void analyzeGait();                    // Análisis completo de marcha
void detectStep();                     // Detecta pasos usando picos de aceleración
void detectContact();                  // Detecta contacto con el suelo
void calculateGaitVariability();       // Calcula variabilidad y detecta cojera
void addStepToHistory(float stepTime, float impact); // Añade paso al historial circular
float calculateMean(float* array, int count);        // Calcula promedio de array
float calculateStdDev(float* array, int count, float mean); // Calcula desviación estándar
void sendDataToProcessing();           // Envía datos en formato CSV para Processing
void sendDataToPlotter();              // Envía datos para Serial Plotter de Arduino
void displayOnMonitor();               // Muestra datos legibles en Serial Monitor
void readTemperature();                // Lee sensor térmico infrarrojo
void checkSerialHealth();              // Verifica salud de la comunicación serial
void checkSerialCommands();            // Procesa comandos del usuario (P, M, G, C)
void calculateAdvancedMetrics();       // Calcula métricas avanzadas (cadencia, velocidad, etc.)
void calibrateIMU();                   // Calibración automática del IMU

// SETUP - CONFIGURACIÓN INICIAL
void setup() {
  // Inicializar comunicación serial a alta velocidad (115200 bps)
  Serial.begin(115200);
  while (!Serial) delay(10); // Esperar a que el puerto serial esté listo
  
  // Mostrar banner de inicio
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   ANÁLISIS DE MARCHA - SISTEMA V3.0   ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  Serial.println("🦶 Funcionalidades:");
  Serial.println("   • Conteo automático de pasos");
  Serial.println("   • Medición de impacto y energía");
  Serial.println("   • Cadencia (pasos/SEGUNDO)");
  Serial.println("   • Velocidad estimada");
  Serial.println("   • Suavidad de movimiento");
  Serial.println("   • Detección de contacto con suelo");
  Serial.println("   • Orientación 3D (Roll, Pitch, Yaw)");
  Serial.println("   • Detección de cojera por variabilidad");
  Serial.println("   • Temperatura infrarroja CALIBRADA");
  Serial.println("   • Calibración automática del IMU ✨");
  Serial.println();

  // Inicializar bus I2C
  Serial.print("⏳ Inicializando I2C... ");
  Wire.begin(5, 4); // SDA=GPIO5, SCL=GPIO4 (específico para ESP32-S3)
  Serial.println("✓ OK");

  // Inicializar IMU con reintentos automáticos
  bool icmReady = false;
  while (!icmReady) {
    myICM.begin(Wire, 0); // Iniciar IMU con dirección I2C por defecto
    Serial.print("⏳ Inicializando IMU... ");
    
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("✗ Reintentando en 2 segundos...");
      delay(2000);
    } else {
      icmReady = true;
      Serial.println("✓ OK");
    }
  }

  // Inicializar sensor térmico MLX90614
  Serial.print("⏳ Inicializando sensor térmico... ");
  if (!mlx.begin()) {
    Serial.println("✗ FALLO (continuando sin sensor térmico)");
  } else {
    Serial.println("✓ OK");
    Serial.print("   ℹ️  Offset de calibración: +");
    Serial.print(TEMP_OFFSET, 1);
    Serial.println("°C");
  }

  // Mostrar menú de modos de visualización
  Serial.println("\n═══════════════════════════════════════");
  Serial.println("✅ SISTEMA LISTO");
  Serial.println("═══════════════════════════════════════");
  Serial.println();
  
  Serial.println("\n🎛️  MODOS DE VISUALIZACIÓN:");
  Serial.println("   Envía por Serial Monitor:");
  Serial.println("   • 'P' = Modo Processing (datos CSV)");
  Serial.println("   • 'M' = Modo Monitor (legible)");
  Serial.println("   • 'G' = Modo Plotter (gráficas)");
  Serial.println("   • 'C' = Recalibrar IMU");
  Serial.println();
  Serial.println("📊 Usa Processing para visualización 3D");
  Serial.println();

  // Inicializar historial de pasos
  for (int i = 0; i < MAX_STEP_HISTORY; i++) {
    stepTimeHistory[i] = 0.0;
    stepImpactHistory[i] = 0.0;
  }

  // Inicializar marcas de tiempo
  lastAngleTime = millis();
  lastStepTime = millis();
  lastPhaseTime = millis();
  
  // CALIBRACIÓN AUTOMÁTICA AL INICIO
  // El usuario debe mantener el dispositivo quieto durante 5 segundos
  calibrateIMU();
  
  Serial.println("\n✓ Sistema calibrado y listo\n");
  Serial.println("--- INICIO DE TRANSMISIÓN ---\n");
}

// LOOP PRINCIPAL - EJECUCIÓN CONTINUA
void loop() {
  unsigned long currentTime = millis();
  
  // Procesar comandos del usuario (P, M, G, C)
  checkSerialCommands();
  
  // Verificar salud de la comunicación serial cada 1 segundo
  if (currentTime - lastSerialCheck >= serialCheckInterval) {
    lastSerialCheck = currentTime;
    checkSerialHealth();
  }

  // Leer y procesar datos del IMU si están disponibles
  if (myICM.dataReady()) {
    readIMUData();              // Leer acelerómetro, giroscopio, magnetómetro
    integrateAngles();          // Calcular orientación 3D
    analyzeGait();              // Analizar patrón de marcha
    calculateAdvancedMetrics(); // Calcular métricas avanzadas
  }

  // Leer sensor de temperatura (más lento, no necesita alta frecuencia)
  readTemperature();

  // Enviar datos según el modo seleccionado (cada 50ms = 20Hz)
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    
    switch(outputMode) {
      case 0:
        sendDataToProcessing(); // Formato CSV para Processing
        break;
      case 1:
        displayOnMonitor();     // Formato legible para humanos
        break;
      case 2:
        sendDataToPlotter();    // Formato para Arduino Serial Plotter
        break;
    }
  }
}

// CALIBRACIÓN DEL IMU
/*
  PROPÓSITO:
  Los sensores MEMS tienen pequeños errores de offset que varían con la 
  temperatura, edad del sensor, y condiciones ambientales. Esta función
  mide esos offsets durante 5 segundos con el sensor quieto.
  
  PROCESO:
  1. Recolectar 200 muestras (25ms cada una = 5 segundos total)
  2. Promediar las lecturas para obtener el offset
  3. Para el acelerómetro Z, restar 1g (gravedad en reposo)
  4. Para el giroscopio, el promedio debe ser ~0 en reposo
  
  REQUISITOS:
  - El dispositivo debe estar COMPLETAMENTE QUIETO
  - Debe estar colocado en el gemelo
  - El usuario no debe moverse durante 5 segundos
*/
void calibrateIMU() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║       CALIBRACIÓN DEL IMU              ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
  Serial.println("📍 INSTRUCCIONES:");
  Serial.println("   1. Coloca el dispositivo en el gemelo");
  Serial.println("   2. Mantén la pierna COMPLETAMENTE QUIETA");
  Serial.println("   3. La calibración tomará 5 segundos");
  Serial.println();
  Serial.println("⏳ Iniciando en 3 segundos...");
  delay(3000); // Dar tiempo al usuario para prepararse
  
  Serial.println("\n🔄 Calibrando...");
  Serial.print("   Progreso: [");
  
  const int NUM_SAMPLES = 200; // 200 muestras × 25ms = 5 segundos
  float sumAccX = 0.0, sumAccY = 0.0, sumAccZ = 0.0;
  float sumGyrX = 0.0, sumGyrY = 0.0, sumGyrZ = 0.0;
  
  int progressStep = NUM_SAMPLES / 20; // 20 bloques para barra de progreso
  
  // Recolectar muestras
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT(); // Leer acelerómetro, giroscopio, magnetómetro
      
      // Leer valores CRUDOS (sin aplicar offsets previos)
      float rawAccX = myICM.accX() / 1000.0; // Convertir de mg a g
      float rawAccY = myICM.accY() / 1000.0;
      float rawAccZ = myICM.accZ() / 1000.0;
      
      float rawGyrX = myICM.gyrX(); // Ya viene en °/s
      float rawGyrY = myICM.gyrY();
      float rawGyrZ = myICM.gyrZ();
      
      // Acumular para calcular promedio
      sumAccX += rawAccX;
      sumAccY += rawAccY;
      sumAccZ += rawAccZ;
      
      sumGyrX += rawGyrX;
      sumGyrY += rawGyrY;
      sumGyrZ += rawGyrZ;
      
      // Mostrar progreso visual
      if (i % progressStep == 0) {
        Serial.print("█");
      }
    }
    delay(25); // 200 muestras × 25ms = 5000ms = 5 segundos
  }
  
  Serial.println("] ✓");
  Serial.println();
  
  // Calcular offsets (promedios)
  accX_offset = sumAccX / NUM_SAMPLES;
  accY_offset = sumAccY / NUM_SAMPLES;
  accZ_offset = (sumAccZ / NUM_SAMPLES) - 1.0; // En reposo, Z debe medir 1g (gravedad)
  
  gyrX_offset = sumGyrX / NUM_SAMPLES; // En reposo, giroscopio debe medir 0
  gyrY_offset = sumGyrY / NUM_SAMPLES;
  gyrZ_offset = sumGyrZ / NUM_SAMPLES;
  
  isCalibrated = true; // Marcar calibración como completa
  
  // Mostrar resultados de calibración
  Serial.println("✅ CALIBRACIÓN COMPLETADA");
  Serial.println();
  Serial.println("📊 Offsets calculados:");
  Serial.print("   Acelerómetro X: ");
  Serial.print(accX_offset, 4);
  Serial.println(" g");
  Serial.print("   Acelerómetro Y: ");
  Serial.print(accY_offset, 4);
  Serial.println(" g");
  Serial.print("   Acelerómetro Z: ");
  Serial.print(accZ_offset, 4);
  Serial.println(" g");
  Serial.println();
  Serial.print("   Giroscopio X: ");
  Serial.print(gyrX_offset, 2);
  Serial.println(" °/s");
  Serial.print("   Giroscopio Y: ");
  Serial.print(gyrY_offset, 2);
  Serial.println(" °/s");
  Serial.print("   Giroscopio Z: ");
  Serial.print(gyrZ_offset, 2);
  Serial.println(" °/s");
  Serial.println();
  
  // Resetear ángulos integrados
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

// LEER DATOS DEL IMU
/*
  PROPÓSITO:
  Lee los datos crudos del IMU y aplica la calibración.
  
  PROCESO:
  1. Leer acelerómetro (mg) → convertir a g → aplicar offset
  2. Leer giroscopio (°/s) → aplicar offset
  3. Leer magnetómetro (µT) → sin calibración por ahora
*/
void readIMUData() {
  myICM.getAGMT(); // Solicitar nueva lectura del IMU
  
  // Acelerómetro: Convertir de miligramos a gravedades y aplicar offset
  accX_g = (myICM.accX() / 1000.0) - accX_offset;
  accY_g = (myICM.accY() / 1000.0) - accY_offset;
  accZ_g = (myICM.accZ() / 1000.0) - accZ_offset;
  // GIROSCOPIO: Aplicar calibración
  // Los valores ya vienen en grados/segundo (°/s), solo se resta el offset
  gyrX_dps = myICM.gyrX() - gyrX_offset;
  gyrY_dps = myICM.gyrY() - gyrY_offset;
  gyrZ_dps = myICM.gyrZ() - gyrZ_offset;
  
  // MAGNETÓMETRO: Lectura directa (sin calibración por ahora)
  // El magnetómetro no se usa actualmente pero está disponible para
  // futuras mejoras (ej: corrección de deriva del yaw)
  magX = myICM.magX();
  magY = myICM.magY();
  magZ = myICM.magZ();
}

// LEER TEMPERATURA
/*
  PROPÓSITO:
  Lee el sensor térmico infrarrojo MLX90614 para medir la temperatura
  muscular del gemelo. Útil para detectar fatiga o inflamación.
  
  FUNCIONAMIENTO:
  - Se lee solo cada 500ms (la temperatura cambia lentamente)
  - Se aplica un offset de +4°C para corregir la distancia sensor-piel
  - También se lee la temperatura ambiente como referencia
*/
void readTemperature() {
  static unsigned long lastTempRead = 0; // Variable estática que persiste entre llamadas
  
  // Solo leer cada 500ms para no saturar el sensor térmico
  if (millis() - lastTempRead >= 500) {
    // Leer temperatura cruda del objeto (músculo/piel)
    float tempRaw = mlx.readObjectTempC();
    
    // Aplicar offset de calibración empírico
    // Este valor compensa la distancia entre el sensor y la piel
    tempObjeto = tempRaw + TEMP_OFFSET;
    
    // Leer temperatura ambiente como referencia
    tempAmbiente = mlx.readAmbientTempC();
    
    // Actualizar marca de tiempo
    lastTempRead = millis();
  }
}

// INTEGRAR ÁNGULOS (ORIENTACIÓN 3D)
/*
  PROPÓSITO:
  Calcula la orientación 3D del sensor integrando las velocidades angulares
  del giroscopio a lo largo del tiempo.
  
  MÉTODO:
  - Roll, Pitch, Yaw se calculan como: ángulo_nuevo = ángulo_anterior + velocidad_angular × tiempo
  - Esto se llama "integración numérica" o "dead reckoning"
  
  LIMITACIÓN:
  - El giroscopio tiende a "derivar" con el tiempo (error acumulativo)
  - En aplicaciones que requieren precisión a largo plazo se usa un filtro
    complementario o de Kalman combinando acelerómetro y magnetómetro
*/
void integrateAngles() {
  unsigned long currentTime = millis();
  
  // Calcular tiempo transcurrido (dt) en segundos
  float dt = (currentTime - lastAngleTime) / 1000.0;
  lastAngleTime = currentTime;
  
  // INTEGRACIÓN: ángulo += velocidad_angular × tiempo
  // Roll: rotación alrededor del eje X (inclinación lateral)
  roll += gyrX_dps * dt;
  
  // Pitch: rotación alrededor del eje Y (inclinación frontal/trasera)
  pitch += gyrY_dps * dt;
  
  // Yaw: rotación alrededor del eje Z (giro horizontal, como brújula)
  yaw += gyrZ_dps * dt;
  
  // NORMALIZACIÓN: Mantener ángulos en el rango -180° a +180°
  // Esto evita que los ángulos crezcan indefinidamente y facilita su visualización
  if (roll > 180.0) roll -= 360.0;
  if (roll < -180.0) roll += 360.0;
  if (pitch > 180.0) pitch -= 360.0;
  if (pitch < -180.0) pitch += 360.0;
  if (yaw > 180.0) yaw -= 360.0;
  if (yaw < -180.0) yaw += 360.0;
}

// ANALIZAR MARCHA - FUNCIÓN PRINCIPAL
/*
  PROPÓSITO:
  Coordina todas las funciones de análisis de marcha en el orden correcto.
*/
void analyzeGait() {
  detectStep();              // 1. Detectar pasos
  detectContact();           // 2. Detectar contacto con el suelo
  calculateGaitVariability(); // 3. Calcular variabilidad y detectar cojera
}

// DETECTAR PASOS
/*
  ALGORITMO DE DETECCIÓN DE PASOS:
  
  1. Calcular la magnitud total de aceleración: √(x² + y² + z²)
  2. En reposo, esta magnitud es 1g (gravedad)
  3. Al caminar, la aceleración varía creando picos
  4. Detectar cuando la desviación cruza el umbral (STEP_THRESHOLD)
  
  CONDICIONES PARA DETECTAR UN PASO:
  - La desviación debe superar el umbral (0.65g)
  - Debe ser un cruce ascendente (paso anterior < umbral)
  - No debe haberse detectado un paso recientemente (bandera stepDetected)
  - Debe haber pasado tiempo suficiente desde el último paso (MIN_STEP_INTERVAL)
  
  ANTI-REBOTE:
  - Se usa stepDetected para evitar múltiples detecciones en un mismo pico
  - Solo se resetea cuando la aceleración baja significativamente
*/
void detectStep() {
  // Calcular magnitud total de aceleración
  float totalAcc = sqrt(accX_g*accX_g + accY_g*accY_g + accZ_g*accZ_g);
  
  // Calcular desviación de 1g (en reposo debería ser 1g)
  float accDeviation = abs(totalAcc - 1.0);
  
  // Guardar como impacto actual (se usa en visualización)
  currentImpact = accDeviation;
  
  // Obtener tiempo actual
  unsigned long currentTime = millis();
  
  // Calcular tiempo desde el último paso (en segundos)
  float timeSinceLastStep = (currentTime - lastStepTime) / 1000.0;
  
  // DETECCIÓN DE PASO: Verificar todas las condiciones
  if (accDeviation > STEP_THRESHOLD &&           // Supera el umbral
      totalAcc_prev < STEP_THRESHOLD &&          // Cruce ascendente
      !stepDetected &&                           // No detectado recientemente
      timeSinceLastStep > MIN_STEP_INTERVAL) {   // Suficiente tiempo desde último paso
    
    // Incrementar contador de pasos
    stepCount++;
    
    // Activar bandera para evitar detecciones múltiples
    stepDetected = true;
    
    // Guardar el impacto máximo de este paso
    maxImpact = accDeviation;
    
    // Añadir al historial solo si el intervalo es razonable
    // (descarta pasos muy lentos que pueden ser falsos positivos)
    if (timeSinceLastStep < MAX_STEP_INTERVAL) {
      addStepToHistory(timeSinceLastStep, accDeviation);
    }
    
    // Actualizar marca de tiempo del último paso
    lastStepTime = currentTime;
  }
  
  // RESETEAR BANDERA: Cuando la aceleración baja significativamente
  // Se usa 0.6 del umbral como histéresis para evitar oscilaciones
  if (accDeviation < STEP_THRESHOLD * 0.6) {
    stepDetected = false;
  }
  
  // Guardar valor para la próxima iteración (detectar cruces)
  totalAcc_prev = accDeviation;
}

// DETECTAR CONTACTO CON EL SUELO
/*
  PROPÓSITO:
  Determinar cuándo el pie está en contacto con el suelo vs. en el aire.
  Esto permite calcular las fases de apoyo (stance) y vuelo (swing).
  
  MÉTODO:
  - Cuando el pie está en el suelo, la aceleración es cercana a 1g (estable)
  - Cuando el pie está en el aire, la aceleración varía más
  - Se usa un umbral bajo (CONTACT_THRESHOLD = 0.25g) para detectar estabilidad
  
  FASES DE LA MARCHA:
  - STANCE (Apoyo): Pie en contacto con el suelo (~60% del ciclo)
  - SWING (Vuelo): Pie en el aire (~40% del ciclo)
*/
void detectContact() {
  // Calcular magnitud total de aceleración
  float totalAcc = sqrt(accX_g*accX_g + accY_g*accY_g + accZ_g*accZ_g);
  unsigned long currentTime = millis();
  
  // INICIO DE CONTACTO: Aceleración cercana a 1g (estable)
  if (abs(totalAcc - 1.0) < CONTACT_THRESHOLD && !inContact) {
    inContact = true;                    // Activar bandera de contacto
    contactStartTime = currentTime;      // Marcar inicio del contacto
    
    // Calcular duración de la fase de vuelo anterior
    if (lastPhaseTime > 0) {
      totalSwingTime += (currentTime - lastPhaseTime);
    }
    lastPhaseTime = currentTime;
  } 
  // FIN DE CONTACTO: Aceleración se aleja de 1g (inestable)
  else if (abs(totalAcc - 1.0) >= CONTACT_THRESHOLD && inContact) {
    inContact = false;  // Desactivar bandera de contacto
    
    // Calcular duración del contacto que acaba de terminar
    lastContactDuration = currentTime - contactStartTime;
    
    // Acumular tiempo total de apoyo
    totalStanceTime += lastContactDuration;
    
    // Marcar inicio de la fase de vuelo
    lastPhaseTime = currentTime;
  }
  
  // CALCULAR PORCENTAJES: % de tiempo en cada fase
  if (stepCount > 0) {
    unsigned long totalTime = totalStanceTime + totalSwingTime;
    if (totalTime > 0) {
      // En marcha normal: ~60% apoyo, ~40% vuelo
      stancePhasePercent = (float)totalStanceTime / totalTime * 100.0;
      swingPhasePercent = (float)totalSwingTime / totalTime * 100.0;
    }
  }
}

// AÑADIR PASO AL HISTORIAL
/*
  PROPÓSITO:
  Mantener un buffer circular con los últimos 10 pasos para análisis estadístico.
  
  ESTRUCTURA DE DATOS:
  - Buffer circular: cuando se llena, empieza a sobrescribir los datos más antiguos
  - historyIndex: apunta a la siguiente posición a escribir
  - historyCount: cantidad de datos válidos (máximo 10)
*/
void addStepToHistory(float stepTime, float impact) {
  // Guardar tiempo e impacto del paso en la posición actual
  stepTimeHistory[historyIndex] = stepTime;
  stepImpactHistory[historyIndex] = impact;
  
  // Avanzar índice de forma circular (vuelve a 0 después de 9)
  historyIndex = (historyIndex + 1) % MAX_STEP_HISTORY;
  
  // Incrementar contador hasta llenar el buffer (máximo 10)
  if (historyCount < MAX_STEP_HISTORY) {
    historyCount++;
  }
}

// CALCULAR VARIABILIDAD DE LA MARCHA
/*
  PROPÓSITO:
  Detectar patrones irregulares en la marcha que pueden indicar cojera.
  
  INDICADORES DE COJERA:
  1. Variabilidad temporal: pasos con tiempos muy diferentes
  2. Variabilidad de impacto: pasos con fuerzas muy diferentes
  
  MÉTODO ESTADÍSTICO:
  - Coeficiente de variación = desviación estándar / promedio
  - Un valor alto indica irregularidad
  - Umbrales: 25% para tiempo, 30% para impacto
*/
void calculateGaitVariability() {
  // Necesitamos al menos 3 pasos para calcular estadísticas confiables
  if (historyCount < 3) {
    isLimping = false;
    timeVariability = 0.0;
    return;
  }
  
  // Calcular promedios
  avgStepTime = calculateMean(stepTimeHistory, historyCount);
  avgStepImpact = calculateMean(stepImpactHistory, historyCount);
  
  // Calcular desviaciones estándar
  float timeStdDev = calculateStdDev(stepTimeHistory, historyCount, avgStepTime);
  float impactStdDev = calculateStdDev(stepImpactHistory, historyCount, avgStepImpact);
  
  // Calcular coeficientes de variación (CV = σ/μ)
  // CV indica la variabilidad relativa: 0 = perfectamente regular, >0.3 = muy irregular
  timeVariability = (avgStepTime > 0) ? (timeStdDev / avgStepTime) : 0.0;
  impactVariability = (avgStepImpact > 0) ? (impactStdDev / avgStepImpact) : 0.0;
  
  // DETECCIÓN DE COJERA: Si alguna variabilidad supera el umbral
  isLimping = (timeVariability > TIME_VARIABILITY_THRESHOLD) || 
              (impactVariability > IMPACT_VARIABILITY_THRESHOLD);
}

// CALCULAR MÉTRICAS AVANZADAS
/*
  PROPÓSITO:
  Calcular métricas clínicas y biomecánicas adicionales.
*/
void calculateAdvancedMetrics() {
  // CADENCIA: Pasos por segundo (Hz)
  // Fórmula: cadencia = 1 / tiempo_promedio_entre_pasos
  // Valores normales: 1.5-2.0 pasos/segundo (90-120 pasos/minuto)
  if (avgStepTime > 0 && stepCount > 0) {
    cadence = 1.0 / avgStepTime;
  } else {
    cadence = 0.0;
  }
  
  // VELOCIDAD ESTIMADA: metros por segundo
  // Fórmula empírica: velocidad ≈ cadencia × longitud_de_paso
  // Se asume una longitud de paso promedio de ~0.7 metros
  // Valores normales: 1.0-1.4 m/s (marcha normal)
  if (cadence > 0) {
    walkingSpeed = cadence * 0.7;
  } else {
    walkingSpeed = 0.0;
  }
  
  // ENERGÍA DE IMPACTO: Proporcional a aceleración²
  // Fórmula simplificada: E = 0.5 × m × v²  (donde m·v ∝ aceleración)
  float accMagnitude = sqrt(accX_g*accX_g + accY_g*accY_g + accZ_g*accZ_g);
  impactEnergy = 0.5 * accMagnitude * accMagnitude;
  
  // JERK: Derivada de la aceleración (suavidad del movimiento)
  // Jerk = Δaceleración / Δtiempo
  // Valores bajos = movimiento suave
  // Valores altos = movimiento brusco/irregular
  unsigned long currentTime = millis();
  float dt = (currentTime - lastAngleTime) / 1000.0;
  if (dt > 0 && prevAccMagnitude > 0) {
    jerk = abs(accMagnitude - prevAccMagnitude) / dt;
  }
  prevAccMagnitude = accMagnitude;
  
  // VELOCIDAD ANGULAR TOTAL: Magnitud del vector de rotación
  // Indica qué tan rápido está rotando la pierna en el espacio 3D
  angularVelocityTotal = sqrt(gyrX_dps*gyrX_dps + gyrY_dps*gyrY_dps + gyrZ_dps*gyrZ_dps);
}

// CALCULAR PROMEDIO (MEDIA ARITMÉTICA)
/*
  PROPÓSITO:
  Calcular el valor promedio de un array de datos.
  
  FÓRMULA: μ = (Σx) / n
*/
float calculateMean(float* array, int count) {
  float sum = 0.0;
  
  // Sumar todos los valores
  for (int i = 0; i < count; i++) {
    sum += array[i];
  }
  
  // Dividir entre la cantidad de valores
  return sum / count;
}

// CALCULAR DESVIACIÓN ESTÁNDAR
/*
  PROPÓSITO:
  Medir la dispersión de los datos respecto al promedio.
  
  FÓRMULA: σ = √(Σ(x - μ)² / n)
  
  INTERPRETACIÓN:
  - σ pequeña: datos concentrados cerca del promedio (marcha regular)
  - σ grande: datos dispersos (marcha irregular)
*/
float calculateStdDev(float* array, int count, float mean) {
  float sumSquares = 0.0;
  
  // Sumar los cuadrados de las diferencias
  for (int i = 0; i < count; i++) {
    float diff = array[i] - mean;        // Diferencia respecto al promedio
    sumSquares += diff * diff;           // Elevar al cuadrado y acumular
  }
  
  // Calcular la raíz cuadrada del promedio de los cuadrados
  return sqrt(sumSquares / count);
}

// ENVIAR DATOS A PROCESSING (FORMATO CSV)
/*
  PROPÓSITO:
  Enviar todos los datos en formato CSV para visualización en Processing.
  Processing es un entorno de programación visual que puede leer estos datos
  y crear gráficas 3D, animaciones, etc.
  
  FORMATO:
  roll,pitch,yaw,accZ,steps,impact,contact,limping,variability,...
  
  SINCRONIZACIÓN:
  - Espera a que haya espacio en el buffer serial antes de enviar
  - Usa flush() para asegurar que los datos se envíen inmediatamente
*/
void sendDataToProcessing() {
  // Esperar a que haya suficiente espacio en el buffer de salida
  while (Serial.availableForWrite() < 128) {
    delay(1);  // Esperar 1ms si el buffer está lleno
  }
  
  // Enviar todos los datos separados por comas (CSV)
  // El segundo parámetro de print() es la precisión decimal
  
  Serial.print(roll, 2);                          // Ángulo roll (2 decimales)
  Serial.print(",");
  Serial.print(pitch, 2);                         // Ángulo pitch
  Serial.print(",");
  Serial.print(yaw, 2);                           // Ángulo yaw
  Serial.print(",");
  Serial.print(accZ_g, 3);                        // Aceleración Z (3 decimales)
  Serial.print(",");
  Serial.print(stepCount);                        // Contador de pasos (entero)
  Serial.print(",");
  Serial.print(currentImpact, 3);                 // Impacto actual
  Serial.print(",");
  Serial.print(inContact ? 1 : 0);                // Contacto (booleano como 1/0)
  Serial.print(",");
  Serial.print(isLimping ? 1 : 0);                // Cojera detectada (booleano)
  Serial.print(",");
  Serial.print(timeVariability * 100, 1);         // Variabilidad temporal (%)
  Serial.print(",");
  Serial.print(avgStepTime, 3);                   // Tiempo promedio entre pasos
  Serial.print(",");
  Serial.print(avgStepImpact, 2);                 // Impacto promedio
  Serial.print(",");
  Serial.print(cadence, 2);                       // Cadencia (pasos/segundo)
  Serial.print(",");
  Serial.print(walkingSpeed, 2);                  // Velocidad estimada (m/s)
  Serial.print(",");
  Serial.print(impactEnergy, 3);                  // Energía de impacto
  Serial.print(",");
  Serial.print(jerk, 2);                          // Jerk (suavidad)
  Serial.print(",");
  Serial.print(angularVelocityTotal, 2);          // Velocidad angular total
  Serial.print(",");
  Serial.print(stancePhasePercent, 1);            // % fase de apoyo
  Serial.print(",");
  Serial.print(swingPhasePercent, 1);             // % fase de vuelo
  Serial.print(",");
  Serial.print(tempObjeto, 1);                    // Temperatura del músculo
  Serial.println();                               // Nueva línea (fin de registro)
  
  // Forzar envío inmediato de datos
  Serial.flush();
}

// ENVIAR DATOS AL SERIAL PLOTTER
/*
  PROPÓSITO:
  Formato específico para el Serial Plotter de Arduino IDE.
  
  FORMATO:
  Etiqueta1:valor1 Etiqueta2:valor2 ...
  
  ESCALADO:
  - Algunos valores se dividen (roll, pitch) para que quepan en el gráfico
  - Otros se invierten (suavidad = 10/jerk) para mejor visualización
  - Booleanos se convierten a valores numéricos (0, 3, 4) para verlos mejor
*/
void sendDataToPlotter() {
  // Ángulos divididos por 10 para que no dominen la escala
  Serial.print("Roll:");
  Serial.print(roll / 10.0, 1);
  Serial.print(" ");
  
  Serial.print("Pitch:");
  Serial.print(pitch / 10.0, 1);
  Serial.print(" ");
  
  // Impacto actual (valor directo)
  Serial.print("Impacto:");
  Serial.print(currentImpact, 2);
  Serial.print(" ");
  
  // Cadencia en pasos/segundo
  Serial.print("Cadencia:");
  Serial.print(cadence, 2);
  Serial.print(" ");
  
  // Velocidad en m/s
  Serial.print("Velocidad:");
  Serial.print(walkingSpeed, 2);
  Serial.print(" ");
  
  // Energía de impacto
  Serial.print("Energia:");
  Serial.print(impactEnergy, 2);
  Serial.print(" ");
  
  // Suavidad: invertida para que valores altos = mejor
  // Se usa max() para evitar división por cero
  Serial.print("Suavidad:");
  Serial.print(10.0 / max(jerk, 0.1f), 2);
  Serial.print(" ");
  
  // Variabilidad temporal en porcentaje
  Serial.print("Variabilidad:");
  Serial.print(timeVariability * 100, 1);
  Serial.print(" ");
  
  // Contacto como valor numérico (3.0 = sí, 0.0 = no)
  Serial.print("Contacto:");
  Serial.print(inContact ? 3.0 : 0.0);
  Serial.print(" ");
  
  // Cojera como valor numérico (4.0 = sí, 0.0 = no)
  Serial.print("Cojera:");
  Serial.print(isLimping ? 4.

// Cojera como valor numérico (4.0 = sí, 0.0 = no)
  Serial.print("Cojera:");
  Serial.print(isLimping ? 4.0 : 0.0);
  Serial.print(" ");
  
  // Temperatura del objeto/músculo
  Serial.print("TempObj:");
  Serial.print(tempObjeto, 1);
  
  // Nueva línea para completar el registro
  Serial.println();
  
  // Forzar envío inmediato de datos
  Serial.flush();
}

// MOSTRAR DATOS EN MONITOR (FORMATO LEGIBLE)
/*
  PROPÓSITO:
  Mostrar los datos en un formato legible para humanos en el Serial Monitor.
  Ideal para monitoreo rápido y depuración.
  
  FORMATO:
  Pasos:X | Cadencia:Y | Vel:Z | ... | CONTACTO/EN_AIRE | ⚠COJERA/✓OK
  
  CARACTERÍSTICAS:
  - Formato compacto en una sola línea
  - Etiquetas claras y abreviadas
  - Símbolos visuales (⚠ para alertas, ✓ para OK)
  - Unidades incluidas (p/s, m/s, g, °C, %)
*/
void displayOnMonitor() {
  // MÉTRICAS PRINCIPALES
  Serial.print("Pasos:");
  Serial.print(stepCount);                        // Contador total de pasos
  
  Serial.print(" | Cadencia:");
  Serial.print(cadence, 2);                       // Pasos por segundo
  Serial.print("p/s");
  
  Serial.print(" | Vel:");
  Serial.print(walkingSpeed, 2);                  // Velocidad estimada
  Serial.print("m/s");
  
  Serial.print(" | Impacto:");
  Serial.print(currentImpact, 2);                 // Impacto actual
  Serial.print("g");                              // Unidad: gravedades
  
  Serial.print(" | Energia:");
  Serial.print(impactEnergy, 2);                  // Energía de impacto
  
  // SUAVIDAD DE MOVIMIENTO (inversa del jerk)
  // Valores altos = movimiento suave
  // Valores bajos = movimiento brusco
  // Se usa max() para evitar división por cero
  Serial.print(" | Suavidad:");
  Serial.print(10.0 / max(jerk, 0.1f), 1);
  
  // FASES DEL CICLO DE MARCHA
  Serial.print(" | Apoyo:");
  Serial.print(stancePhasePercent, 0);            // Fase de contacto con suelo
  Serial.print("%");
  
  Serial.print(" | Vuelo:");
  Serial.print(swingPhasePercent, 0);             // Fase en el aire
  Serial.print("%");
  
  Serial.print(" | ");
  
  // ESTADO DE CONTACTO CON EL SUELO
  if (inContact) {
    Serial.print("CONTACTO | ");                  // Pie en el suelo
  } else {
    Serial.print("EN_AIRE | ");                   // Pie en el aire
  }
  
  // DETECCIÓN DE COJERA (con símbolos visuales)
  if (isLimping) {
    Serial.print("⚠COJERA | ");                   // ⚠ Alerta: cojera detectada
  } else {
    Serial.print("✓OK | ");                       // ✓ Marcha normal
  }
  
  // TEMPERATURA MUSCULAR
  Serial.print("Temp:");
  Serial.print(tempObjeto, 1);                    // Temperatura calibrada
  Serial.print("°C");                             // Unidad: grados Celsius
  
  // Nueva línea para completar el registro
  Serial.println();
}

// VERIFICAR COMANDOS DEL USUARIO POR SERIAL
/*
  PROPÓSITO:
  Permite al usuario cambiar el modo de visualización o recalibrar el IMU
  enviando comandos por el Serial Monitor.
  
  COMANDOS DISPONIBLES:
  - 'P' o 'p': Activar modo Processing (datos CSV)
  - 'M' o 'm': Activar modo Monitor (formato legible)
  - 'G' o 'g': Activar modo Plotter (gráficas Arduino)
  - 'C' o 'c': Recalibrar el IMU
  
  FUNCIONAMIENTO:
  - Lee un byte del puerto serial
  - Limpia el buffer (descarta caracteres extras)
  - Ejecuta la acción correspondiente según el comando
*/
void checkSerialCommands() {
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    
    // Leer el primer carácter (comando)
    char cmd = Serial.read();
    
    // Limpiar el buffer serial (descartar caracteres adicionales)
    // Esto evita que caracteres extra (como '\n' o '\r') causen problemas
    while (Serial.available() > 0) {
      Serial.read();
    }
    
    // Procesar el comando recibido
    switch(cmd) {
      // MODO PROCESSING (CSV)
      case 'P':
      case 'p':
        outputMode = 0;                           // Cambiar a modo 0
        Serial.println("\n✓ Modo PROCESSING activado (CSV)");
        break;
        
      // MODO MONITOR (LEGIBLE)
      case 'M':
      case 'm':
        outputMode = 1;                           // Cambiar a modo 1
        Serial.println("\n✓ Modo MONITOR activado (legible)");
        break;
        
      // MODO PLOTTER (GRÁFICAS)
      case 'G':
      case 'g':
        outputMode = 2;                           // Cambiar a modo 2
        Serial.println("\n✓ Modo PLOTTER activado (gráficas)");
        break;
        
      // RECALIBRAR IMU
      case 'C':
      case 'c':
        Serial.println("\n⏸️  Pausando transmisión para recalibrar...");
        delay(500);                               // Pausa breve para estabilizar
        calibrateIMU();                           // Ejecutar calibración
        Serial.println("▶️  Reanudando transmisión...\n");
        break;
    }
  }
}

// VERIFICAR SALUD DE LA COMUNICACIÓN SERIAL
/*
  PROPÓSITO:
  Mantener la comunicación serial saludable y prevenir problemas de
  saturación del buffer o pérdida de sincronización.
  
  PROBLEMAS QUE PREVIENE:
  1. Buffer de recepción lleno (datos no leídos acumulados)
  2. Buffer de transmisión saturado (no hay espacio para enviar)
  3. Pérdida de sincronización con la computadora
  
  ESTRATEGIA DE RECUPERACIÓN:
  - Si el buffer de salida está casi lleno → esperar
  - Si está crítico → reiniciar la conexión serial
  
  NOTA: Esta función se llama cada 1 segundo (ver serialCheckInterval)
*/
void checkSerialHealth() {
  // LIMPIAR BUFFER DE ENTRADA
  // Descartar cualquier dato no leído en el buffer de recepción
  // Esto previene que comandos antiguos o basura se acumulen
  while (Serial.available() > 0) {
    Serial.read();                                // Leer y descartar
  }
  
  // VERIFICAR ESPACIO EN BUFFER DE SALIDA
  // availableForWrite() devuelve cuántos bytes se pueden escribir
  // Un valor bajo indica que los datos no se están enviando lo suficientemente rápido
  
  if (Serial.availableForWrite() < 32) {          // Buffer casi lleno (< 32 bytes libres)
    delay(10);                                    // Esperar 10ms para que se vacíe
    
    // SITUACIÓN CRÍTICA: Reiniciar conexión serial
    if (Serial.availableForWrite() < 16) {        // Todavía crítico (< 16 bytes)
      Serial.end();                               // Cerrar puerto serial
      delay(100);                                 // Esperar para liberar recursos
      Serial.begin(115200);                       // Reabrir puerto serial
      
      // NOTA: Esto puede causar pérdida momentánea de datos, pero evita
      // que el sistema se bloquee completamente por saturación del buffer
    }
  }
}

/*
  RESUMEN DEL SISTEMA:
  
  FLUJO DE EJECUCIÓN:
  1. setup(): Inicializar hardware y calibrar IMU
  2. loop(): Ejecutar continuamente
     - Leer IMU (acelerómetro, giroscopio, magnetómetro)
     - Calcular orientación 3D (roll, pitch, yaw)
     - Detectar pasos y contacto con el suelo
     - Analizar variabilidad de la marcha
     - Calcular métricas avanzadas (cadencia, velocidad, etc.)
     - Enviar datos en el formato seleccionado (P/M/G)
     - Verificar comandos del usuario
     - Mantener salud de la comunicación serial
  
  CARACTERÍSTICAS PRINCIPALES:
  ✓ Calibración automática del IMU al inicio
  ✓ Detección robusta de pasos con anti-rebote
  ✓ Análisis estadístico de variabilidad (cojera)
  ✓ Múltiples formatos de salida (CSV, legible, gráfico)
  ✓ Monitoreo de temperatura muscular
  ✓ Métricas biomecánicas completas
  ✓ Sistema robusto con recuperación de errores
  
  APLICACIONES:
  - Rehabilitación física
  - Análisis deportivo
  - Detección temprana de lesiones
  - Monitoreo de pacientes con problemas de movilidad
  - Investigación biomecánica

*/