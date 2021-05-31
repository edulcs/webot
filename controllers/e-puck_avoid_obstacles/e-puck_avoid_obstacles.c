/* include headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define QUANTIDADE_DE_SENSORES_DE_PROXIMIDADE 8
#define QUANTIDADE_DE_LEDS 10
#define TIME_STEP 256

WbDeviceTag MotorEsquerdo, MotorDireito;

WbFieldRef trans_field;

WbFieldRef trans_field;

WbDeviceTag SensorProximidade[QUANTIDADE_DE_SENSORES_DE_PROXIMIDADE];

WbDeviceTag Leds[QUANTIDADE_DE_LEDS];

static void configuraMotores() {


    MotorEsquerdo = wb_robot_get_device("left wheel motor");
    MotorDireito = wb_robot_get_device("right wheel motor");

    wb_motor_set_position(MotorEsquerdo, INFINITY);
    wb_motor_set_position(MotorDireito, INFINITY);

    wb_motor_set_velocity(MotorEsquerdo, 0);
    wb_motor_set_velocity(MotorDireito, 0);

}

static void configuraSensoresProximidade() {

    SensorProximidade[0] = wb_robot_get_device("ps0");
    SensorProximidade[1] = wb_robot_get_device("ps1");
    SensorProximidade[2] = wb_robot_get_device("ps2");
    SensorProximidade[3] = wb_robot_get_device("ps3");
    SensorProximidade[4] = wb_robot_get_device("ps4");
    SensorProximidade[5] = wb_robot_get_device("ps5");
    SensorProximidade[6] = wb_robot_get_device("ps6");
    SensorProximidade[7] = wb_robot_get_device("ps7");

    wb_distance_sensor_enable(SensorProximidade[0], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[1], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[2], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[3], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[4], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[5], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[6], TIME_STEP);
    wb_distance_sensor_enable(SensorProximidade[7], TIME_STEP);

}

static void configuraLeds() {

    Leds[0] = wb_robot_get_device("led0");
    wb_led_set(Leds[0], -1);

}

int main(int argc, char ** argv) {

    int i = 0;
    double LeituraSensorProximidade[QUANTIDADE_DE_SENSORES_DE_PROXIMIDADE];
    double AceleradorDireito = 1.0, AceleradorEsquerdo = 1.0;
    
    wb_robot_init();
    configuraMotores();
    configuraSensoresProximidade();
    configuraLeds();

    //supervisor
    WbNodeRef robot_node = wb_supervisor_node_get_self();

   //campo de posição
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

    const double * posicaoDoRobo; 

    // main loop
    while (wb_robot_step(TIME_STEP) != -1) {
     
      for (i = 0; i < QUANTIDADE_DE_SENSORES_DE_PROXIMIDADE; i++) {
        LeituraSensorProximidade[i] = wb_distance_sensor_get_value(SensorProximidade[i]) - 60;
      }
      
      wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

      posicaoDoRobo = wb_supervisor_field_get_sf_vec3f(trans_field);
      printf("C3PO Position: X = %.2f | Y = %.2f | Z = %.2f |\n", posicaoDoRobo[0], posicaoDoRobo[1], posicaoDoRobo[2]);

      if (
        //Primeira Caixa
        (posicaoDoRobo[0] < 0.10 && posicaoDoRobo[0] > -0.03 && posicaoDoRobo[2] < -0.17 && posicaoDoRobo[2] > -0.33) ||
        
        //SegundaCaixa
        (posicaoDoRobo[0] < -0.28 && posicaoDoRobo[0] > -0.34 && posicaoDoRobo[2] < 0.22 && posicaoDoRobo[2] > 0.17)
        
        ) {
           AceleradorDireito = 1;
           AceleradorEsquerdo = 1;
           printf("\n COLISÃO \n");
           printf("Posição da caixa: X = %.2f | Y = %.2f | Z = %.2f |\n", posicaoDoRobo[0], posicaoDoRobo[1], posicaoDoRobo[2]);
         } 
        else if (LeituraSensorProximidade[0] > 100) {
          AceleradorDireito = 1;
          AceleradorEsquerdo = -0.62;
        } else if (LeituraSensorProximidade[1] > 100) {
          AceleradorDireito = 1;
          AceleradorEsquerdo = -0.64;
        } else if (LeituraSensorProximidade[2] > 100) {
          AceleradorDireito = 1;
          AceleradorEsquerdo = -0.69;
        } else if (LeituraSensorProximidade[5] > 100) {
          AceleradorDireito = -0.68;
          AceleradorEsquerdo = 1;
        } else if (LeituraSensorProximidade[6] > 100) {
          AceleradorDireito = -0.62;
          AceleradorEsquerdo = 1;
        } else if (LeituraSensorProximidade[7] > 100) {
          AceleradorDireito = -0.64;
          AceleradorEsquerdo = 1;
        } else {
          AceleradorDireito = 1;
          AceleradorEsquerdo = 1;
        }

        wb_motor_set_velocity(MotorEsquerdo, 6.28 * AceleradorEsquerdo);
        wb_motor_set_velocity(MotorDireito, 6.28 * AceleradorDireito);

      };

      /* This is necessary to cleanup webots resources */
      wb_robot_cleanup();

      return 0;
    }