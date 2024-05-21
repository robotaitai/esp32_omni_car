#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Encoder.h>
#include <esp_task_wdt.h>
#include "hal/encoder.h"
#include "vehicle/motor.h"
#include "vehicle/vehicle.h"
#include "configuration.h"
#include "pid/pos_pid.h"
#include "pid/velocity_pid.h"
#include "hal/l298n.h"
#include "comm/comm_controller.h"

#define DEBUG 0
// PID objects
POS_PID pos_pid_left_front_motor;
POS_PID pos_pid_right_front_motor;
POS_PID pos_pid_left_rear_motor;
POS_PID pos_pid_right_rear_motor;

VEL_PID vel_pid_left_front_motor;
VEL_PID vel_pid_right_front_motor;
VEL_PID vel_pid_left_rear_motor;
VEL_PID vel_pid_right_rear_motor;

POS_PID pos_pid_x;
POS_PID pos_pid_y;
POS_PID pos_pid_angular;

VEL_PID vel_pid_x;
VEL_PID vel_pid_y;
VEL_PID vel_pid_angular;

Vehicle_PIDs vehicle_pids;
/////////////////////////

// Encoder objects
Encoder encoderLF;
Encoder encoderRR;
Encoder encoderLR;
Encoder encoderRF;
///////////////////

// Motor driver objects
L298N driverLF;
L298N driverRR;
L298N driverLR;
L298N driverRF;
////////////////////////

// Motor objects
Motor left_front_motor;
Motor right_front_motor;
Motor left_rear_motor;
Motor right_rear_motor;
///////////////////////

// Vehicle object
Vehicle vehicle;
/////////////////

// Communication object
CommController comm;
//////////////////////

// FreeRtos objects
TaskHandle_t MotorControlTaskHandle;
TaskHandle_t vehicleControlTaskHandle;
TaskHandle_t CommunicationTaskHandle;
TaskHandle_t SerialCommandTaskHandle;

SemaphoreHandle_t vehicleDesiredStateMutex;
SemaphoreHandle_t MotorVelocityMutex;
SemaphoreHandle_t vehicleCurrentStateMutex;
SemaphoreHandle_t MotorUpdateMutex;

/////////////////////////// MOTOR CONTROL TASK ////////////////////////////////
void motorControlTask(void *parameter)
{
  Serial.println("function_1");
  Vehicle *vehicle = (Vehicle *)parameter;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_DT * 1000);
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    vehicle->left_front_motor.current_position = readEncoder(&vehicle->left_front_motor.encoder) * vehicle->left_front_motor.distancePerTick;

    if (xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY))
    {
      computeVelocity(&vehicle->left_front_motor);
      xSemaphoreGive(MotorVelocityMutex);
    }
    if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
    {
      motor_step(&vehicle->left_front_motor);
      xSemaphoreGive(MotorUpdateMutex);
    }

    vehicle->right_front_motor.current_position = readEncoder(&vehicle->right_front_motor.encoder) * vehicle->right_front_motor.distancePerTick;

    if (xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY))
    {
      computeVelocity(&vehicle->right_front_motor);
      xSemaphoreGive(MotorVelocityMutex);
    }
    if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
    {
      motor_step(&vehicle->right_front_motor);
      xSemaphoreGive(MotorUpdateMutex);
    }

    vehicle->left_rear_motor.current_position = readEncoder(&vehicle->left_rear_motor.encoder) * vehicle->left_rear_motor.distancePerTick;

    if (xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY))
    {
      computeVelocity(&vehicle->left_rear_motor);
      xSemaphoreGive(MotorVelocityMutex);
    }
    if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
    {
      motor_step(&vehicle->left_rear_motor);
      xSemaphoreGive(MotorUpdateMutex);
    }

    vehicle->right_rear_motor.current_position = readEncoder(&vehicle->right_rear_motor.encoder) * vehicle->right_rear_motor.distancePerTick;

    if (xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY))
    {
      computeVelocity(&vehicle->right_rear_motor);
      xSemaphoreGive(MotorVelocityMutex);
    }

    if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
    {
      motor_step(&vehicle->right_rear_motor);
      xSemaphoreGive(MotorUpdateMutex);
    }
  }
}
////////////////////////// END OF MOTOR CONTROL TASK ////////////////////////////////////////

//////////////////////// VEHICLE CONTROL TASK ///////////////////////////////////////////////
void vehicleControlTask(void *parameter)
{
  Vehicle *vehicle = (Vehicle *)parameter;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(ODOMETRY_DT * 1000);
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY))
    {
      compute_odometry_from_encoders(vehicle);
      xSemaphoreGive(MotorVelocityMutex);
    }

    if (xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY))
    {
      ProcessDataToSend(&comm, vehicle);
      xSemaphoreGive(vehicleCurrentStateMutex);
    }
  }
}
////////////////////////END OF  VEHICLE CONTROL TASK ///////////////////////////////////////////////

//////////////////////// COMMUNICATION TASK ///////////////////////////////////////////////
// void communicationTask(void *parameter)
// {
//   Vehicle *vehicle = (Vehicle *)parameter;
//   TickType_t xLastWakeTime;
//   const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);
//   xLastWakeTime = xTaskGetTickCount();
  
//   for (;;)
//   {

//     vTaskDelayUntil(&xLastWakeTime, xFrequency);

//     if (xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY))
//     {
//       Serial2.write(comm.TxData, SIZE_OF_TX_DATA); // Transmit data
//       xSemaphoreGive(vehicleCurrentStateMutex);
//     }

//     if (Serial2.available() >= 0)
//     {
//       Serial2.readBytes(comm.RxData, SIZE_OF_RX_DATA); // Receive data
//       int check = receiveData(&comm, vehicle);
//       if (check == 2)
//       {
//         Serial2.flush();
//         if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
//         {
//           translate_twist_to_motor_commands(vehicle);
//           xSemaphoreGive(MotorUpdateMutex);
//         }
//       }
//     }
//   }
// }
void communicationTask(void *parameter)
{
  Vehicle *vehicle_ = (Vehicle *)parameter;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (Serial2.available() > 0)
    {
      Serial.println("Data available on Serial2");

      int len = Serial2.readBytes(comm.RxData, SIZE_OF_RX_DATA);
      Serial.print("Read bytes: ");
      Serial.println(len);
      if (len == SIZE_OF_RX_DATA)
      {
        Serial.println("Received data from Serial2");

        int check = receiveData(&comm, vehicle_);

        if (check == 2)
        {
          Serial.println("Valid data received");
          if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY))
          {
            translate_twist_to_motor_commands(vehicle_);
            xSemaphoreGive(MotorUpdateMutex);
          }
        }
        else
        {
          Serial.println("Checksum failed or data invalid");
        }
      }
      else
      {
        Serial.println("Incomplete data received");
      }

      // Clearing buffer if needed
      while (Serial2.available() > 0)
      {
        Serial2.read();
      }
    }

    if (xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY))
    {
      if (DEBUG)Serial.println("Preparing to transmit data");
      ProcessDataToSend(&comm, vehicle_);
      Serial2.write(comm.TxData, SIZE_OF_TX_DATA); // Transmit data
      if (DEBUG)Serial.println("Data transmitted");
      xSemaphoreGive(vehicleCurrentStateMutex);
    }
  }
}

/////////////////////////?TAIO COMM TASK ///////////////////////////////////////////////

void printEncoderPositions()
{
  // Use your existing mechanism to get and print encoder positions
  int encoderLF_pos = readEncoder(&encoderLF);
  int encoderRR_pos = readEncoder(&encoderRR);
  int encoderLR_pos = readEncoder(&encoderLR);
  int encoderRF_pos = readEncoder(&encoderRF);

  Serial.print("Encoder LF: ");
  Serial.print(encoderLF_pos);
  Serial.print(", RR: ");
  Serial.print(encoderRR_pos);
  Serial.print(", LR: ");
  Serial.print(encoderLR_pos);
  Serial.print(", RF: ");
  Serial.println(encoderRF_pos);
}

void printVehicleStatus(Vehicle *vehicle)
{
  Serial.println("Vehicle Status:");
  Serial.print("Position X: ");
  Serial.println(vehicle->current_state.position.x);
  Serial.print("Position Y: ");
  Serial.println(vehicle->current_state.position.y);
  Serial.print("Angular Position: ");
  Serial.println(vehicle->current_state.position.angular);
  Serial.print("Velocity X: ");
  Serial.println(vehicle->current_state.velocity.x);
  Serial.print("Velocity Y: ");
  Serial.println(vehicle->current_state.velocity.y);
  Serial.print("Angular Velocity: ");
  Serial.println(vehicle->current_state.velocity.angular);
}

void printVehicleDesiredCommand(Vehicle *vehicle)
{
  Serial.println("Vehicle Status:");
  Serial.print("Position X: ");
  Serial.println(vehicle->desired_state.position.x);
  Serial.print("Position Y: ");
  Serial.println(vehicle->desired_state.position.y);
  Serial.print("Angular Position: ");
  Serial.println(vehicle->desired_state.position.angular);
  Serial.print("Velocity X: ");
  Serial.println(vehicle->desired_state.velocity.x);
  Serial.print("Velocity Y: ");
  Serial.println(vehicle->desired_state.velocity.y);
  Serial.print("Angular Velocity: ");
  Serial.println(vehicle->desired_state.velocity.angular);
}
void processCommand(char command, Vehicle *vehicle)
{
  const float velocity_increment = 0.1;         // Adjust velocity by 0.1 m/s with each command
  const float angular_velocity_increment = 0.1; // Adjust angular velocity by 0.1 rad/s
  // Serial.print("Processing command: ");
  // Serial.println(command);
  switch (command)
  {
  case 'w': // Increase forward velocity
    vehicle->desired_state.velocity.x += velocity_increment;
    break;
  case 's': // Decrease forward velocity (or reverse)
    vehicle->desired_state.velocity.x -= velocity_increment;
    break;
  case 'a': // Move left (decrease y velocity)
    vehicle->desired_state.velocity.y -= velocity_increment;
    break;
  case 'd': // Move right (increase y velocity)
    vehicle->desired_state.velocity.y += velocity_increment;
    break;
  case 'e': // Rotate clockwise (increase angular velocity)
    vehicle->desired_state.velocity.angular += angular_velocity_increment;
    break;
  case 'q': // Rotate counter-clockwise (decrease angular velocity)
    vehicle->desired_state.velocity.angular -= angular_velocity_increment;
    break;
  case 't': // Print current status
    printVehicleStatus(vehicle);
    break;
  case 'g': // Print current status
    printVehicleDesiredCommand(vehicle);
    break;
  default:
    Serial.println("Unknown command");
    break;
  }

  // Ensure the vehicle steps to process the new desired state
  if (xSemaphoreTake(vehicleDesiredStateMutex, portMAX_DELAY))
  {
    // ProcessDataToSend(&comm, vehicle);
    vehicle_step(vehicle);
    xSemaphoreGive(vehicleDesiredStateMutex);
  }
  else
  {
    Serial.println("Failed to take vehicle state mutex");
  }
}


void SerialCommandTask(void *pvParameters)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);
  xLastWakeTime = xTaskGetTickCount();
  Vehicle *vehicle = (Vehicle *)pvParameters; // Assuming you pass the vehicle object as a parameter when creating the task
  char incomingChar;

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (Serial.available() )
    {
      incomingChar = Serial.read();
      if (incomingChar != '\n' && incomingChar != '\r' && incomingChar != ' ')
      {
        processCommand(incomingChar, vehicle);
        Serial.print("Received command: ");
        Serial.println(incomingChar);
      }
    }
    // vTaskDelay(pdMS_TO_TICKS(10)); // Delay to prevent the task from hogging the CPU
  }
}
////////////////////////END OF COMMUNICATION TASK ///////////////////////////////////////////////
/////////////////////////////
void setup()
{
  BaseType_t taskCreated;

  // Initialize Serial for debugging
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
  {
    // Wait for Serial to initialize
  }
  Serial.println("Serial initialized");

  // Initialize Serial2 for communication
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, 16, 17);
  while (!Serial2)
  {
    // Wait for Serial2 to initialize
  }
  Serial.println("Serial2 initialized");

  // Initialize vehicle PIDs
  initPosPID(&pos_pid_x, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initPosPID(&pos_pid_y, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initPosPID(&pos_pid_angular, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);

  initVelPID(&vel_pid_x, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
  initVelPID(&vel_pid_y, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
  initVelPID(&vel_pid_angular, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  init_vehicle_pids(&vehicle_pids, vel_pid_x, vel_pid_y, vel_pid_angular, pos_pid_x, pos_pid_y, pos_pid_angular);

  // Initialize PIDs for each motor
  initPosPID(&pos_pid_left_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_left_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  initPosPID(&pos_pid_right_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_right_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  initPosPID(&pos_pid_left_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_left_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  initPosPID(&pos_pid_right_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_right_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  // Initialize encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  initEncoder(&encoderLF, LF_ENCODER_PIN_A, LF_ENCODER_PIN_B);
  initEncoder(&encoderRR, RR_ENCODER_PIN_A, RR_ENCODER_PIN_B);
  initEncoder(&encoderLR, LR_ENCODER_PIN_A, LR_ENCODER_PIN_B);
  initEncoder(&encoderRF, RF_ENCODER_PIN_A, RF_ENCODER_PIN_B);

  // Initialize motor drivers
  initL298N(&driverLF, LF_L298N_ENA, LF_L298N_IN1, LF_L298N_IN2);
  initL298N(&driverRR, RR_L298N_ENA, RR_L298N_IN1, RR_L298N_IN2);
  initL298N(&driverLR, LR_L298N_ENA, LR_L298N_IN1, LR_L298N_IN2);
  initL298N(&driverRF, RF_L298N_ENA, RF_L298N_IN1, RF_L298N_IN2);

  // Initialize motors
  initMotor(&left_front_motor, encoderLF, driverLF, pos_pid_left_front_motor, vel_pid_left_front_motor, LF_DIRECTION);
  initMotor(&right_front_motor, encoderRF, driverRF, pos_pid_right_front_motor, vel_pid_right_front_motor, RF_DIRECTION);
  initMotor(&left_rear_motor, encoderLR, driverLR, pos_pid_left_rear_motor, vel_pid_left_rear_motor, LR_DIRECTION);
  initMotor(&right_rear_motor, encoderRR, driverRR, pos_pid_right_rear_motor, vel_pid_right_rear_motor, RR_DIRECTION);

  // Initialize vehicle
  init_vehicle(&vehicle, left_front_motor, right_front_motor, left_rear_motor, right_rear_motor, vehicle_pids);

  // Initialize communication
  comm_controller_init(&comm);

  // Create Mutex for Vehicle Data
  vehicleDesiredStateMutex = xSemaphoreCreateMutex();
  MotorVelocityMutex = xSemaphoreCreateMutex();
  vehicleCurrentStateMutex = xSemaphoreCreateMutex();
  MotorUpdateMutex = xSemaphoreCreateMutex();

  // Create motorControlTask
  taskCreated = xTaskCreatePinnedToCore(
      motorControlTask,        // Task function
      "MotorControlTask",      // Name of task
      60000,                   // Stack size of task
      &vehicle,                // Parameter of the task
      3,                       // Priority of the task
      &MotorControlTaskHandle, // Task handle to keep track of created task
      MOTOR_CONTROL_CORE);     // Core where the task should run

  if (taskCreated != pdPASS)
  {
    Serial.println("MotorControlTask creation failed!");
  }
  else
  {
    Serial.println("MotorControlTask creation success!");
  }

  // Create vehicleControlTask
  taskCreated = xTaskCreatePinnedToCore(
      vehicleControlTask,        // Task function
      "VehicleControlTask",      // Name of task
      60000,                     // Stack size of task
      &vehicle,                  // Parameter of the task
      2,                         // Priority of the task
      &vehicleControlTaskHandle, // Task handle to keep track of created task
      ODOMETRY_CORE);            // Core where the task should run

  if (taskCreated != pdPASS)
  {
    Serial.println("VehicleControlTask creation failed!");
  }
  else
  {
    Serial.println("VehicleControlTask creation success!");
  }

  // Create communicationTask
  taskCreated = xTaskCreatePinnedToCore(
      communicationTask,        // Task function
      "CommunicationTask",      // Name of task
      30000,                    // Stack size of task
      &vehicle,                 // Parameter of the task
      1,                        // Priority of the task
      &CommunicationTaskHandle, // Task handle to keep track of created task
      COMMUNICATION_CORE);      // Core where the task should run

  if (taskCreated != pdPASS)
  {
    Serial.println("CommunicationTask creation failed!");
  }
  else
  {
    Serial.println("CommunicationTask creation success!");
  }

  // Create SerialCommandTask
  taskCreated = xTaskCreatePinnedToCore(
      SerialCommandTask,        // Task function
      "SerialCmdTask",          // Name of the task (for debugging)
      30000,                    // Stack size (bytes)
      &vehicle,                 // Task input parameter
      2,                        // Priority of the task
      &SerialCommandTaskHandle, // Task handle
      COMMUNICATION_CORE);      // Core you want to run the task on (0 or 1)

  if (taskCreated != pdPASS)
  {
    Serial.println("SerialCmdTask creation failed!");
  }
  else
  {
    Serial.println("SerialCmdTask creation success!");
  }
}

void loop()
{
  // Empty loop as tasks are managed by FreeRTOS
}
