#include "sdlgyro.h"
#include <godot_cpp/core/class_db.hpp>
#include <SDL2/SDL.h>
#include <godot_cpp/variant/utility_functions.hpp>
#include "GamepadMotion.hpp"
#include "godot_cpp/variant/array.hpp"
#include "godot_cpp/variant/vector2.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector4.hpp"
#include "godot_cpp/variant/variant.hpp"
#include <godot_cpp/godot.hpp>
#include <chrono>

using namespace godot;

SDL_Event event;  
SDL_GameController *controller =nullptr;

bool pollingEnabled=true;
bool gyroEnabled=false;
bool accelEnabled=false;
float deltaTime=0.0;

Vector3 Gyro;
Vector3 Accel;
Vector4 Orientation;


static constexpr float toDegPerSec = float(180. / M_PI);
static constexpr float toGs = 1.f / 9.8f;

std::chrono::steady_clock::time_point oldTime;
std::chrono::steady_clock::time_point newTime;

GamepadMotion gyroSensor;

void SDLGyro::_bind_methods() {
  ClassDB::bind_method(D_METHOD("sdl_init"),&SDLGyro::sdl_init);
  ClassDB::bind_method(D_METHOD("controller_init", "controller_index"), &SDLGyro::controller_init, DEFVAL(-1));
  ClassDB::bind_method(D_METHOD("gamepad_polling"),&SDLGyro::gamepadPolling);
  ClassDB::bind_method(D_METHOD("calibrate"),&SDLGyro::calibrate);
  ClassDB::bind_method(D_METHOD("stop_calibrate"),&SDLGyro::stop_calibrate);
  ClassDB::bind_method(D_METHOD("get_player_space"),&SDLGyro::getPlayer_space);
  ClassDB::bind_method(D_METHOD("get_world_space"),&SDLGyro::getWorld_space);
  ClassDB::bind_method(D_METHOD("get_gravity"),&SDLGyro::getGravity);
  ClassDB::bind_method(D_METHOD("get_calibrated_gyro"),&SDLGyro::getCalibratedGyro);
  ClassDB::bind_method(D_METHOD("get_processed_acceleration"),&SDLGyro::getProcessedAcceleration);
  ClassDB::bind_method(D_METHOD("set_auto_calibration"),&SDLGyro::setAutoCalibration);
  ClassDB::bind_method(D_METHOD("is_gyro_steady"),&SDLGyro::isCalibrationSteady);
  ClassDB::bind_method(D_METHOD("get_calibration_confidence"),&SDLGyro::getCalibrationConfidence);
}

void SDLGyro::sdl_init() {
  newTime=std::chrono::steady_clock::now();
  oldTime=newTime;
  //SDL initializATION
  if((SDL_Init(SDL_INIT_GAMECONTROLLER))<0){
    //UtilityFunctions::print("could not initialize SDL \n");
  }
  else{
    //UtilityFunctions::print("SDL initialized!!!!!!!!!!! \n");
  }

}

//CALIBRATION
void SDLGyro::calibrate(){
  gyroSensor.Reset();
  pollingEnabled=false;
  gyroSensor.StartContinuousCalibration(); 
}

void SDLGyro::stop_calibrate(){
  gyroSensor.PauseContinuousCalibration();
  pollingEnabled=true;
}

bool SDLGyro::isCalibrationSteady(){
  return gyroSensor.GetAutoCalibrationIsSteady();
}

float SDLGyro::getCalibrationConfidence(){
  return gyroSensor.GetAutoCalibrationConfidence();
}

//Convert To 2D
Variant SDLGyro::getPlayer_space(){
  Vector2 playerSpace;
  gyroSensor.GetWorldSpaceGyro(playerSpace[0],playerSpace[1]);
  return playerSpace;
}
Variant SDLGyro::getWorld_space(){
  Vector2 worldSpace;
  gyroSensor.GetPlayerSpaceGyro(worldSpace[0],worldSpace[1]);
  return worldSpace;
}
Variant SDLGyro::getGravity(){
  Vector3 gravity;
  gyroSensor.GetGravity(gravity[0],gravity[1], gravity[2]);
  return gravity;
}

Variant SDLGyro::getCalibratedGyro(){
  Vector3 calibratedgyro;
  gyroSensor.GetCalibratedGyro(calibratedgyro[0],calibratedgyro[1], calibratedgyro[2]);
  return calibratedgyro;
}
Variant SDLGyro::getProcessedAcceleration(){
  Vector3 processedAcc;
  gyroSensor.GetCalibratedGyro(processedAcc[0],processedAcc[1], processedAcc[2]);
  return processedAcc;
}


void SDLGyro::controller_init(int controller_index) {
  SDL_GameController *test_controller = nullptr;
  bool test_gyroEnabled = false;
  bool test_accelEnabled = false;

  if (controller_index != -1) {
    // Check if the provided index is valid
    if (controller_index < 0 || controller_index >= SDL_NumJoysticks()) {
      UtilityFunctions::print("Invalid controller index\n");
      return;
    }

    // Attempt to open the specified controller by index
    test_controller = SDL_GameControllerOpen(controller_index);
    if (test_controller && SDL_IsGameController(controller_index)) {
      // Check for gyro and accelerometer sensors
      if (SDL_GameControllerHasSensor(test_controller, SDL_SENSOR_GYRO)) {
        SDL_GameControllerSetSensorEnabled(test_controller, SDL_SENSOR_GYRO, SDL_TRUE);
        test_gyroEnabled = true;
      }
      if (SDL_GameControllerHasSensor(test_controller, SDL_SENSOR_ACCEL)) {
        SDL_GameControllerSetSensorEnabled(test_controller, SDL_SENSOR_ACCEL, SDL_TRUE);
        test_accelEnabled = true;
      }

      // If both sensors are enabled, set the controller
      if (test_accelEnabled && test_gyroEnabled) {
        controller = test_controller;
        gyroEnabled = true;
        accelEnabled = true;
      } else {
        UtilityFunctions::print("Selected controller does not have both gyro and accelerometer\n");
        SDL_GameControllerClose(test_controller);
      }
    } else {
      UtilityFunctions::print("Selected index is not a valid game controller\n");
    }

  } else {
    // Default behavior: loop through all connected controllers
    for (int i = 0; i < SDL_NumJoysticks(); i++) {
      test_controller = SDL_GameControllerOpen(i);
      if (SDL_IsGameController(i)) {
        if (SDL_GameControllerHasSensor(test_controller, SDL_SENSOR_GYRO)) {
          SDL_GameControllerSetSensorEnabled(test_controller, SDL_SENSOR_GYRO, SDL_TRUE);
          test_gyroEnabled = true;
        }
        if (SDL_GameControllerHasSensor(test_controller, SDL_SENSOR_ACCEL)) {
          SDL_GameControllerSetSensorEnabled(test_controller, SDL_SENSOR_ACCEL, SDL_TRUE);
          test_accelEnabled = true;
        }

        if (test_accelEnabled && test_gyroEnabled) {
          controller = test_controller;
          gyroEnabled = true;
          accelEnabled = true;
          break; // Use the first controller with both sensors
        } else {
          SDL_GameControllerClose(test_controller);
        }
      }
    }
  }

  if (controller) {
    UtilityFunctions::print("Controller initialized with gyro and accelerometer\n");
  } else {
    UtilityFunctions::print("No suitable controller found\n");
  }
}

void SDLGyro::setAutoCalibration(){
  gyroSensor.SetCalibrationMode(GamepadMotionHelpers::Stillness | GamepadMotionHelpers::SensorFusion);
}

Variant SDLGyro::gamepadPolling(){
  Vector4 orientation;
  //TypedArray<float> orientation;
  //IMU gyro
  if (gyroEnabled && accelEnabled){
    SDL_GameControllerGetSensorData(controller,SDL_SENSOR_GYRO, &Gyro[0], 3);
  //IMU accelerometer//
    SDL_GameControllerGetSensorData(controller,SDL_SENSOR_ACCEL, &Accel[0], 3);
  //Sensor Fussion//
    if (oldTime!=newTime)
      newTime=std::chrono::steady_clock::now();
    deltaTime=((float)std::chrono::duration_cast<std::chrono::microseconds>(newTime-oldTime).count()) / 1000000.0f;

    gyroSensor.ProcessMotion(Gyro[0]*toDegPerSec, Gyro[1]*toDegPerSec, Gyro[2]*toDegPerSec, Accel[0]*toGs, Accel[1]*toGs, Accel[2]*toGs,deltaTime);
    oldTime=std::chrono::steady_clock::now();

    gyroSensor.GetOrientation(orientation[0], orientation[1], orientation[2], orientation[3]);
  }

  //event loop//
  while(SDL_PollEvent(&event)){
    switch (event.type) {
      //hot pluging//
      case SDL_CONTROLLERDEVICEADDED:
        if (!controller){
          //UtilityFunctions::print("controller conected\n");
          //SDLGyro::controller_init();//
        }
        break;
      case SDL_CONTROLLERDEVICEREMOVED:
        SDL_GameControllerClose(controller);
        controller=nullptr;
        //UtilityFunctions::print("controller removed\n");
        gyroEnabled=false;
        accelEnabled=false;
        break;
    //-------------------//
      case SDL_QUIT:
        //UtilityFunctions::print("Quiting SDL.\n");
      default:
        break;
    }
  }
  if (pollingEnabled){
    return orientation;
    }
  else{
    orientation = Vector4(1.0,0.0,0.0,0.0);
    return orientation;
  }
}
