/*******************************************************************************
Code ROS modifié pour intégrer une node ROS de commande
de vélocité pour se déplacer avec le robot holonome Mecanum
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

/******************************************************************************
 * Déclaration des bibliothèques nécessaires
 ******************************************************************************/
#include "turtlebot3_mecanum.h"

/******************************************************************************
 * Intégration des commandes ROS
 ******************************************************************************/
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

const char *node_name = "Porthos_cmd";
const char *topic_name = "Porthos/cmd_vel";

#define LED_PIN 13

/*******************************************************************************
* Déclaration pour Hardware Timer (Interrupt control)
*******************************************************************************/
HardwareTimer Timer(TIMER_CH1);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/*******************************************************************************
* Déclaration pour la télécommande RC100
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel = 0.2;

/*******************************************************************************
* Déclaration pour les moteurs
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;

double linear_x = 0.0;
double linear_y = 0.0;
double angular_z = 0.0;
double goal_linear_x_velocity = 0.0;
double goal_linear_y_velocity = 0.0;
double goal_angular_velocity = 0.0;

/*******************************************************************************
* Callback ROS2 pour cmd_vel
*******************************************************************************/
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  int led_pin_user[2] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2 };
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);

  linear_x = msg->linear.x;
  linear_y = msg->linear.y;  // Correction ici
  angular_z = msg->angular.z;

  // Gestion des LEDs en fonction du mouvement
  if (linear_x == 0 && linear_y == 0) {
    digitalWrite(led_pin_user[0], HIGH);
    digitalWrite(led_pin_user[1], HIGH);
  } 
  else if (linear_x < 0) {
    digitalWrite(led_pin_user[0], HIGH);
    digitalWrite(led_pin_user[1], LOW);
  } 
  else {
    digitalWrite(led_pin_user[1], HIGH);
    digitalWrite(led_pin_user[0], LOW);
  }

  // Contrainte des vitesses maximales
  linear_x = constrain(linear_x, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  linear_y = constrain(linear_y, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  angular_z = constrain(angular_z, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  goal_linear_x_velocity = linear_x;
  goal_linear_y_velocity = linear_y;
  goal_angular_velocity  = angular_z;
}

/*******************************************************************************
* Contrôle des moteurs Mecanum
*******************************************************************************/
void controlMecanum(){
  bool dxl_comm_result = false;

  int64_t wheel_value[MECANUMWHEEL_NUM] = {0, 0, 0, 0};
  double wheel_angular_velocity[MECANUMWHEEL_NUM] = {0.0, 0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[1] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[2] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  wheel_angular_velocity[3] = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);

  for (int id = 0; id < MECANUMWHEEL_NUM; id++)
  {
    wheel_value[id] = wheel_angular_velocity[id] * 9.54 / RPM_CONSTANT_VALUE;
    wheel_value[id] = constrain(wheel_value[id], -LIMIT_X_MAX_VALUE, LIMIT_X_MAX_VALUE);
  }

#ifdef DEBUG
  Serial.print("Vx : "); Serial.print(goal_linear_x_velocity);
  Serial.print(" Vy : "); Serial.print(goal_linear_y_velocity);
  Serial.print(" W : "); Serial.println(goal_angular_velocity);
#endif

  dxl_comm_result = motor_driver.controlMotor((int64_t)wheel_value[0], (int64_t)wheel_value[1], (int64_t)wheel_value[2], (int64_t)wheel_value[3]);
  if (!dxl_comm_result) return;
}

/*******************************************************************************
* Configuration des temps de contrôle des moteurs Dynamixel
*******************************************************************************/
void startDynamixelControlInterrupt(){
  Timer.pause();
  Timer.setPeriod(CONTROL_PERIOD);
  Timer.attachInterrupt(controlMecanum);
  Timer.refresh();
  Timer.resume();
}

/*******************************************************************************
* Setup du microcontrôleur et de ROS2
*******************************************************************************/
void setup(){
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialisation ROS2
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Attendre la connexion avec l'agent micro-ROS
  bool connected = false;
  while (!connected) {
    rcl_ret_t ret = rmw_uros_sync_session(100);  // 100 ms
    if (ret == RCL_RET_OK) {
      connected = true;
    } else {
      delay(100);  // Attendre avant de réessayer
    }
  }
  
  // Création du node
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  // Création du subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    topic_name));

  // Création de l'executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Initialisation des moteurs Dynamixel
  motor_driver.init();
  pinMode(13, OUTPUT);

  // Démarrer le contrôle des moteurs
  startDynamixelControlInterrupt();
}

/*******************************************************************************
* Boucle principale
*******************************************************************************/
void loop(){
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
