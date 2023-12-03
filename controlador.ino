#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <PID_v1.h>

#define SENSOR_PIN A1
#define REFERENCE_RESISTANCE 9777
#define NOMINAL_RESISTANCE 81777
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950

Thermistor *thermistor;

double setpoint = 235.0; // Temperatura inicial en grados Celsius
double Kp = 70;          // Ganancia proporcional
double Ki = 0.6;         // Ganancia integral
double Kd = 0.5;         // Ganancia derivativa

double output;  // Variable para almacenar la salida del control PID
double celsius; // Variable para almacenar la temperatura medida

PID myPID(&celsius, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int stepPin = 2; // Pin de paso del motor paso a paso
int dirPin = 4;  // Pin de dirección del motor paso a paso

float stepDelay = 1.0; // Tiempo entre pasos en milisegundos (valor por defecto: 1 milisegundo)
int direction = 1;     // Dirección por defecto: adelante

bool running = false; // Variable para indicar si la máquina está en marcha

void setup()
{
    pinMode(13, output);
    Serial.begin(9600);

    thermistor = new NTC_Thermistor(
        SENSOR_PIN,
        REFERENCE_RESISTANCE,
        NOMINAL_RESISTANCE,
        NOMINAL_TEMPERATURE,
        B_VALUE);

    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 255); // Límites de salida (0 a 255 para PWM)

    pinMode(stepPin, OUTPUT); // Configura el pin de paso como salida
    pinMode(dirPin, OUTPUT);  // Configura el pin de dirección como salida
}

void configureTemperature()
{
    Serial.println("Ingresa la temperatura del hotend (en grados Celsius): ");
    while (!Serial.available() != 0)
    {
        // Espera a que el usuario ingrese la temperatura
    }

    setpoint = Serial.parseFloat(); // Lee la temperatura ingresada por el usuario
    Serial.println("Temperatura aceptada.");
}

void configureStepDelay()
{
    Serial.println("Ingresa el tiempo entre pasos (en milisegundos): ");
    while (!Serial.available())
    {
        // Espera a que el usuario ingrese el tiempo entre pasos
    }

    stepDelay = Serial.parseFloat(); // Lee el tiempo entre pasos ingresado por el usuario
    Serial.println("Tiempo entre pasos configurado.");
}

void configureDirection()
{
    Serial.println("Ingresa la dirección de giro (0 para atrás, 1 para adelante): ");
    while (!Serial.available())
    {
        // Espera a que el usuario ingrese la dirección
    }

    direction = Serial.parseInt(); // Lee la dirección ingresada por el usuario
    Serial.println("Dirección configurada.");
}

void zeroSettings()
{
    setpoint = 0;
    stepDelay = 0;
    Serial.println("Temperatura y tiempo entre pasos configurados a 0.");
}

void printInfo()
{
    Serial.print("Temperatura: ");
    Serial.print(celsius);
    Serial.print(" C, Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" C, Tiempo entre pasos: ");
    Serial.print(stepDelay);
    Serial.print(" ms, Dirección: ");
    Serial.println(direction);
}

void startMachine()
{
    running = true;
    Serial.println("Maquina en marcha");
}

void stopMachine()
{
    running = false;
    Serial.println("Maquina detenida");
}

void loop()
{
    // Control PID
    celsius = thermistor->readCelsius();
    myPID.Compute();
    analogWrite(3, output); // Aplica la salida al elemento calefactor

    // Si la velocidad del motor es mayor que cero, realizar los pasos del motor
    if (stepDelay > 0 && running)
    {
        digitalWrite(dirPin, direction);
        digitalWrite(stepPin, HIGH);
        delay(stepDelay);
        digitalWrite(stepPin, LOW);
        delay(stepDelay);
    }

    // Verifica si hay comandos disponibles en el puerto serie
    if (Serial.available() > 0)
    {
        char command = Serial.read();
        switch (command)
        {
        case 'V':
            configureStepDelay();
            break;
        case 'T':
            configureTemperature();
            break;
        case 'D':
            configureDirection();
            break;
        case 'B':
            zeroSettings();
            break;
        case 'R':
            startMachine();
            break;
        case 'S':
            stopMachine();
            break;
        case 'I':
            printInfo();
            break;
        default:
            Serial.println("Comando no reconocido.");
            break;
        }
    }
}