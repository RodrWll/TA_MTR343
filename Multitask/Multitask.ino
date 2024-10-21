#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variable global para almacenar el valor del ADC
volatile int adcValue = 0;

// define four tasks for Blink, AnalogRead, PWM control, and LCD update
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
void TaskPWMControl(void *pvParameters);
void TaskLCDUpdate(void *pvParameters);

void vBlinkTask(void *pvParameters)
{
    const int ledPin = 13;
    pinMode(ledPin, OUTPUT);

    while (1)
    {
        digitalWrite(ledPin, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(ledPin, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void vReadAnalogTask(void *pvParameters)
{
    for (;;)
    {
        adcValue = analogRead(A0);
        Serial.println(adcValue);
        vTaskDelay(1); // one tick delay (15ms) in between reads for stability
    }
}

void vLCDUpdateTask(void *pvParameters)
{
    while (1)
    {
        // Actualizar el display LCD con el valor del ADC
        lcd.setCursor(0, 1); // set the cursor to column 0, line 1
        lcd.print("ADC Value: ");
        lcd.print(adcValue);
        lcd.print("    "); // Clear any leftover characters

        // Esperar 500 ms antes de la próxima actualización
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void vPWMControlTask(void *pvParameters)
{
    const int pwmPin = 9; // Pin conectado al LED
    pinMode(pwmPin, OUTPUT);
    int brightness = 0;
    int fadeAmount = 5;

    while (1)
    {
        // Ajustar el brillo del LED
        analogWrite(pwmPin, brightness);

        // Cambiar el brillo para la próxima vez
        brightness = brightness + fadeAmount;

        // Invertir la dirección del cambio de brillo en los extremos
        if (brightness <= 0 || brightness >= 255)
        {
            fadeAmount = -fadeAmount;
        }

        // Esperar 30 ms para ver el cambio de brillo
        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}

// the setup function runs once when you press reset or power the board
void setup()
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }
    lcd.init();          // initialize the lcd
    lcd.backlight();     // turn on the backlight
    lcd.setCursor(0, 0); // set the cursor to column 0, line 0
    lcd.print("MTR343");
    // Esperar 5 segundos antes de iniciar las tareas
    delay(5000);
    // Limpiar la pantalla
    lcd.clear();
    /*--------------*/
    xTaskCreate(
        vBlinkTask,  // Función de la tarea
        "BlinkTask", // Nombre de la tarea
        128,         // Tamaño de la pila en palabras
        NULL,        // Parámetros de la tarea
        2,           // Prioridad de la tarea
        NULL         // Handle de la tarea
    );
    xTaskCreate(
        vReadAnalogTask,  // Función de la tarea
        "ReadAnalogTask", // Nombre de la tarea
        128,              // Tamaño de la pila en palabras
        NULL,             // Parámetros de la tarea
        3,                // Prioridad de la tarea
        NULL              // Handle de la tarea
    );
    xTaskCreate(
        vPWMControlTask,  // Función de la tarea
        "PWMControlTask", // Nombre de la tarea
        128,              // Tamaño de la pila en palabras
        NULL,             // Parámetros de la tarea
        1,                // Prioridad de la tarea
        NULL              // Handle de la tarea
    );
    xTaskCreate(
        vLCDUpdateTask,  // Función de la tarea
        "LCDUpdateTask", // Nombre de la tarea
        128,             // Tamaño de la pila en palabras
        NULL,            // Parámetros de la tarea
        0,               // Prioridad de la tarea
        NULL             // Handle de la tarea
    );

    // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
    // Empty. Things are done in Tasks.
}