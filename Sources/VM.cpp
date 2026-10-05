#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h> //Para el mod con flotantes

#include <unistd.h> // Para la función sleep()
#include "../Headers/BenjasSript.h"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/BmsState_.hpp> // De aqui leemos el porcentaje de bateria
#include <unitree/idl/go2/SportModeState_.hpp> // De aqui obtenemos las distancias alrededor del robot
#include <unitree/robot/go2/sport/sport_client.hpp> // El cliente para mover al perro

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_BMSSTATE "rt/bmsstate"

#define VELOCIDAD_SEGURA 0.5f

// Como manejamos double para los valores, usaremos un valor aceptable
#define eps 0.000001 //Lo aplicaremos en EQ y NEQ
// --- MEMORIA DE LA MÁQUINA VIRTUAL ---
#define STACK_SIZE 100
double pila[STACK_SIZE];
int sp = -1; // Stack Pointer (Apunta al tope de la pila)

double rotar;
double dist;


//Globales del robot=========================================
unitree_go::msg::dds_::SportModeState_ global_sport_state;
unitree_go::msg::dds_::BmsState_ global_bms_state;

void HighStateHandler(const void *message)
{
    global_sport_state = *(unitree_go::msg::dds_::SportModeState_ *)message;
}

void BmsStateHandler(const void *message)
{
    global_bms_state = *(unitree_go::msg::dds_::BmsState_ *)message;
}

//===============================POP y PUSH =========================
// Funciones auxiliares de la pila
void push(double valor) 
{
    if (sp < STACK_SIZE - 1) 
    {
        sp++;
        pila[sp] = valor;
    }
    else 
    {
        printf("Error: Stack Overflow in VM\n");
    }
}

double pop()
{
    if (sp >= 0)
    {
        double valor = pila[sp];
        sp--;
        return valor;
    }
    else
    {
        printf("Error: Stack Underflow in VM\n");
        return 0.0;
    }
}



//FUNCIONES
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include "../Headers/BenjasSript.h"

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode
{
  /*---Basic motion---*/
  normal_stand,
  balance_stand,
  velocity_move,
  rotate_move,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  /*---Special motion ---*/
  sit,
  rise_sit,
  stop_move = 99
};

int TEST_MODE = stand_up;

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  void RobotControl()
  {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

    switch (TEST_MODE)
    {
    case normal_stand:            // 0. idle, default stand
      // sport_client.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
      sport_client.StandUp();
      break;

    case balance_stand:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
      // sport_client.Euler(0.1, 0.2, 0.3); // roll, pitch, yaw
      // sport_client.BodyHeight(0.0);      // relative height [-0.18~0.03]
      sport_client.BalanceStand();
      break;

    case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
      sport_client.Move(0.5, 0, 0);
      break;

    case rotate_move:
        sport_client.Move(0, 0, 1);
      break;

    case stand_down: // 4. position stand down.
      sport_client.StandDown();
      break;

    case stand_up: // 5. position stand up
      sport_client.StandUp();
      break;

    case damp: // 6. damping mode
      sport_client.Damp();
      break;

    case recovery_stand: // 7. recovery stand
      sport_client.RecoveryStand();
      break;

    case sit:
      if (flag == 0)
      {
        sport_client.StandDown();
        flag = 1;
      }
      break;

    case rise_sit:
      if (flag == 0)
      {
        sport_client.RiseSit();
        flag = 1;
      }
      break;

    case stop_move: // stop move
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }
  };

  // Get initial position
  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
  };


  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    // std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
    // std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0; // 初始状态的位置和偏航
  double ct = 0;         // 运行时间
  int flag = 0;          // 特殊动作执行标志
  float dt = 0.005;      // 控制步长0.001~0.01
};





// --- EL CEREBRO EJECUTOR ---
void ejecutar_VM(const char* networkInt)
{

    unitree::robot::ChannelFactory::Instance()->Init(0, "eno1");
    Custom custom;

    sleep(1); // Wait for 1 second to obtain a stable state

    custom.GetInitState(); // Get initial position
    unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

    // Inicializa la red. Cambia "eno2" por tu interfaz real si conectas al robot
    // Para pruebas usa "lo" (loopback local)
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInt);

    //Suscriptores para leer los datos
    unitree::robot::ChannelSubscriberPtr<
    unitree_go::msg::dds_::SportModeState_> suber_sport;
    suber_sport.reset(
        new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber_sport->InitChannel(HighStateHandler, 1);


    unitree::robot::ChannelSubscriberPtr<
        unitree_go::msg::dds_::BmsState_> suber_bms;
    suber_bms.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::BmsState_>(TOPIC_BMSSTATE));
    suber_bms->InitChannel(BmsStateHandler, 1);

    //Creamos el Cliente Deportivo (Una sola vez)
    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    std::cout << "[Starting Go2, wait a sec...]" << std::endl;
    sleep(2);

    

    // Abrimos el archivo en MODO BINARIO (Ignoramos el .txt por completo)
    FILE *binFile = fopen("../../Object.bin", "rb");
    if (binFile == NULL) {
        printf("Cant open the Bytecode.\n");
        return;
    }

    int opCode;
    int ejecutando = 1;

    printf("\n=== STARTING VM BENJAS_SCRIPT ===\n");

    // fread lee el siguiente entero. Si logra leer 1, entra al ciclo.
    while (ejecutando && fread(&opCode, sizeof(int), 1, binFile))
    {
        switch (opCode)
        {
            case OP_HALT: // 0
                printf("[MV] Process end.\n");
                ejecutando = 0;
                //sport_client.StopMove();
                TEST_MODE = rise_sit;
                custom.RobotControl();
                break;

            case OP_PUSH: // OP_PUSH (Ajusta los números a tu enum)
            {
                double constante;
                // Como es PUSH, sabemos que los siguientes 8 bytes son un double
                fread(&constante, sizeof(double), 1, binFile);
                push(constante);
                break;
            }

            case OP_LOAD: // OP_LOAD
            {
                int index;
                // Leemos el indice de la variable
                fread(&index, sizeof(int), 1, binFile);
                // Sacamos su valor de la tabla de simbolos global y lo metemos a la pila
                push(symbolArray[index].doubleValue);
                break;
            }

            case OP_STORE: // OP_STORE
            {
                int index;
                fread(&index, sizeof(int), 1, binFile);
                // Sacamos el valor de la pila y lo guardamos en la memoria
                symbolArray[index].doubleValue = pop();
                break;
            }

            case OP_SUB: // OP_SUB (Resta)
            {
                // OJO AL ORDEN: El primero que sale es el lado DERECHO
                double der = pop(); 
                double izq = pop();
                push(izq - der);
                break;
            }

            case OP_ADD:
            {
                double der = pop(); 
                double izq = pop();
                push(izq + der);
                break;
            }

            case OP_MUL:
            {
                double der = pop(); 
                double izq = pop();
                push(izq * der);
                break;
            }

            case OP_DIV:
            {
                double der = pop(); 
                double izq = pop();
                push(izq / der);
                break;
            }

            case OP_MOD:
            {
                double der = pop(); 
                double izq = pop();
                
                push(fmod(izq,der));
                break;
            }

            case OP_GT: // OP_GT (Mayor que)
            {
                double der = pop();
                double izq = pop();
                if (izq > der) push(1.0); // Verdadero
                else push(0.0);           // Falso
                break;
            }

            case OP_LT: // OP_GT (Mayor que)
            {
                double der = pop();
                double izq = pop();
                if (izq < der) push(1.0); // Verdadero
                else push(0.0);           // Falso
                break;
            }

            case OP_EQ:
            {
                double der = pop();
                double izq = pop();
                if (fabs(izq - der) < eps) push(1.0); // Verdadero
                else push(0.0);           // Falso
                break;
            }

            case OP_NEQ:
            {
                double der = pop();
                double izq = pop();
                if (fabs(izq - der) > eps) push(1.0); // Verdadero
                else push(0.0);           // Falso
                break;
            }

            case OP_NOT_JUMP: // OP_JMP_FALSE (Salto Condicional)
            {
                int destino;
                fread(&destino, sizeof(int), 1, binFile); // Leemos el byte de destino
                
                double condicion = pop();
                if (condicion == 0.0) // Si la condicion fue falsa
                {
                    // ¡VIAJAMOS POR EL ARCHIVO!
                    fseek(binFile, destino, SEEK_SET); 
                }
                break;
            }

            case OP_JUMP: // OP_JMP (Salto Incondicional de regreso)
            {
                int destino;
                fread(&destino, sizeof(int), 1, binFile);
                fseek(binFile, destino, SEEK_SET); // Regresamos directo al inicio del While
                break;
            }

            case OP_WALK: // OP_WALK (Ajusta al numero de tu enum)
            {
                double distanciaX = pop();
        
                //float rotacionYawFloat = float(rotacionYaw);
                
                printf("[Robot Go2] Walking %lfm...\n", distanciaX);

                //sport_client.Move(VELOCIDAD_SEGURA, 0.0f, 0.0f);
                TEST_MODE = velocity_move;
                dist = distanciaX;
                custom.RobotControl();

                float time = fabs((float)distanciaX) / VELOCIDAD_SEGURA;

                //Microsegundos
                usleep((int)(time * 1000000.0f));
                sport_client.StopMove();
                usleep((int)(distanciaX * 1000000.0f));
                sleep(3);
                break;
                
            }

            case OP_GETDIST: // El famoso 15
            {
                // 'range_obstacle' es un arreglo de distancias alrededor del robot
                // El índice [0] corresponde al sector directamente AL FRENTE (cabeza)
                double obtenerDist = (double)global_sport_state.range_obstacle()[0];

                printf("[Robot Go2] Reading distance... Next wall in %lfm...\n", obtenerDist);
                // ¡MUY IMPORTANTE! GetDist devuelve un valor, así que debe hacer un push a la pila
                push(obtenerDist);
                break;
            }

            case OP_GETBATTERY: // 16
            {
                
                int porcentajeBateria = global_bms_state.soc();

                printf("[Robot Go2] Baterry: %d...\n", porcentajeBateria);
                push((double)porcentajeBateria);
                break;
            }

            case OP_ROTATE: // 18
            {
                double radianes = pop();
                
                printf("[Robot Go2] Rotate: %lf...\n", radianes);

                //float dirGiro = radianes > 0 ? VELOCIDAD_SEGURA : -VELOCIDAD_SEGURA;
                
                float dirGiro = radianes;
                TEST_MODE = rotate_move;
                rotar = dirGiro;
                custom.RobotControl();

                float time = fabs((float)radianes) / VELOCIDAD_SEGURA;

                //Microsegundos
                usleep((int)time * 1000000);
                sport_client.StopMove();
                usleep((int)dirGiro * 1000000);
                sleep(3);
                break;
            }

            case OP_CLIMB: // 19 //todavia esta experimental
            {
                double valor = pop(); 
                printf("[Robot Go2] Escalando con parametro %lf...\n", valor);
                break;
            }

            case OP_SIT: // 20
            {
                printf("[Robot Go2] Sit...\n");
                TEST_MODE = sit;
                custom.RobotControl();
                //s.Sit();
                sleep(4);
                break;
            }

            case OP_STAND: // 21
            {
                printf("[Robot Go2] Stand Up...\n");
                TEST_MODE = stand_up;
                custom.RobotControl();
                //sport_client.StandUp();
                sleep(2);
                break;
            }
            case OP_AND: // 24
            {
                // El primero que sale es el operando derecho
                double derecho = pop(); 
                double izquierdo = pop();
                
                // En C, si ambos son diferentes de 0.0, el AND es verdadero
                if (izquierdo != 0.0 && derecho != 0.0)
                {
                    push(1.0); // Verdadero
                }
                else
                {
                    push(0.0); // Falso
                }
                break;
            }

            case OP_OR: // 25
            {
                double derecho = pop();
                double izquierdo = pop();
                
                // Si al menos uno es diferente de 0.0, el OR es verdadero
                if (izquierdo != 0.0 || derecho != 0.0)
                {
                    push(1.0); // Verdadero
                }
                else
                {
                    push(0.0); // Falso
                }
                break;
            }

            

            default:
                printf("[MV] OpCode not recognized: %d\n", opCode);
                ejecutando = 0;
                TEST_MODE = sit;
                custom.RobotControl();
                break;
        }
    }

    fclose(binFile);
    printf("=== MAQUINA VIRTUAL END ===\n");
}