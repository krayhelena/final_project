#include <Arduino_FreeRTOS.h>
#include <semphr.h>


#define PINPWM 10
#define PINTEMP A0

SemaphoreHandle_t mutex;

volatile float setpoint = 30;   //valor desejado da temperatura (entre -40C e 125C)
volatile float Pid;        //valor final do somatório P.I.D

volatile int temperatura, anteriortemp = 0;

// Declarando as tarefas
void TaskAnalogRead( void *pvParameters );
void TaskPWM( void *pvParameters );
void TaskPID( void *pvParameters );

void setup() {

  // Inicialização da conexão serial com 9600 bits por segundo
  Serial.begin(9600);

  while (!Serial) {
    ;                 //Aguardando conexão com o serial
  }
  //criação do mutex
  mutex = xSemaphoreCreateMutex();
  if (mutex != NULL) {
    Serial.println("Mutex criado");
  }
  //Criação das tasks do sistema
  xTaskCreate(
    TaskAnalogRead
    ,  "AnalogRead"
    ,  128  // Tamanho da pilha
    ,  NULL
    ,  2  // Prioridade , 2 - MAX e 0 - MIN
    ,  NULL );

  xTaskCreate(
    TaskPID
    ,  "PID"
    ,  128  // Tamanho da pilha
    ,  NULL
    ,  1  // Prioridade
    ,  NULL );

  xTaskCreate(
    TaskPWM
    ,  "PWM"   // Nome
    ,  128  // Tamanho da pilha
    ,  NULL
    ,  0  // Prioridade
    ,  NULL );
}

void loop()
{
  // Vazio
}

//inicio das tarefas
void TaskAnalogRead(void *pvParameters)
{
  (void) pvParameters;

  //definindo o pino A0 como entrada.
  pinMode(PINTEMP, INPUT);

  volatile float Sensor;

  for (;;)  //laço infinito
  {
    //entrando no mutex
    if ( mutex != NULL ) {
      if ( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE ) {
        // ler o valor analogico do pino 0:
        Sensor = analogRead(PINTEMP);
        //fazendo a conversão do valor recebido para a faixa de temperatura do sensor tmp36
        //esses valores de 20 ate 358 foram retirados do datasheet do dispositivo.
        anteriortemp = temperatura;
        temperatura = map(Sensor, 20, 358, -40, 125);
        // print no terminal do valor da temperatura:
        Serial.print("Temperatura: ");
        Serial.println(temperatura);
        delay(10);
        //saindo do mutex
        xSemaphoreGive(mutex);
        vTaskDelay(10);  // delay para manter estavel os prints.
      }
    }
  }
}

void TaskPID(void *pvParameters)
{
  (void) pvParameters;

  volatile float k = 100;        //ganho de referencia
  volatile float h = 0.1;        //taxa de amostragem (segundos)
  volatile float ti = 10;        //tempo de integração
  volatile float td = 0.1;       //tempo de derivação

  volatile float kp = k * (1 + h / (2 * ti)); //ganho proporcional
  volatile float ki = k / ti;           //ganho integral
  volatile float kd = k * td;           //ganho derivativo

  float p = 0, i = 0, d = 0;       //sinal proporcional, integral e derivativo

  for (;;) //laço infinito
  {
    //entrando no mutex
    if ( mutex != NULL ) {
      if ( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE ) {
        //calculo do erro
        volatile float erro = temperatura - setpoint;
        Serial.print("Erro = ");
        Serial.println(erro);
        //calculo do P, I e D separados
        // Parte proporcional
        p = kp * erro;
        //Parte integral
        i = i + (ki * h * erro);
        //Parte derivativa
        d = (anteriortemp - temperatura) * kd;
        //P.I.D (sinal de controle)
        Pid = p + i + d;
        //saindo do mutex
        xSemaphoreGive(mutex);
        vTaskDelay(10);
      }
    }
  }
}

void TaskPWM(void *pvParameters)
{
  (void) pvParameters;

  // definindo o pino 10(PWM) como saída.
  pinMode(PINPWM, OUTPUT);
  volatile float PWM;

  for (;;) // laço infinito
  {
    //entrando no mutex
    if ( mutex != NULL ) {
      if ( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE ) {
        //tornar compatível com as saídas PWM do arduino
        PWM = map(Pid, -4098, 4098, 0, 255);  //váriavel manipulada
        Serial.print("PWM = ");
        Serial.println(PWM);
        analogWrite(PINPWM, PWM);
        //saindo do mutex
        xSemaphoreGive(mutex);
        vTaskDelay(10);
      }
    }
  }
}
