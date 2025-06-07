#include "MotorController.h"
static void pidCalculationTask(void *pvParameters);

MotorController::MotorController()
    : _velPID(
          0, 0, 0, [this]() { return this->_motor->getVelocity(); },
          [this](double output) { this->_motor->run(output); }),
      _posPID(
          0, 0, 0, [this]() { return this->_motor->getPosition(); },
          [this](double output) { this->_velPID.setTarget(output); }) {}

MotorController::MotorController(MotorDriver *motor)
    : _velPID(
          0, 0, 0, [this]() { return this->_motor->getVelocity(); },
          [this](double output) { this->_motor->run(output); }),
      _posPID(
          0, 0, 0, [this]() { return this->_motor->getPosition(); },
          [this](double output) { this->_velPID.setTarget(output); }) {
  _motor = motor;
}

void MotorController::attachMotorDriver(MotorDriver *motor) { _motor = motor; }

void MotorController::init(void) {
  if (_motor) {
    _motor->init();
  }

  // if (_pidCalculationTaskHandle != NULL) {
  //
  //   vTaskDelete(_pidCalculationTaskHandle);
  // }
  // xTaskCreate(pidCalculationTask, "PID_Calculation", 4096, this, 1,
  //             &_pidCalculationTaskHandle);
}

// ����PID�������ڣ��Ժ���Ϊ��λ��
void MotorController::setPIDCalculationPeriod(uint16_t periodMs) {

  _pidCalculationPeriodMs = periodMs;
}

uint16_t MotorController::getPIDCalculationPeriod() {

  return _pidCalculationPeriodMs;
}

void MotorController::setVelocity(int velocity) {
  _targetVelocity = velocity;
  _velPID.setTarget(_targetVelocity);
  _mode = VELOCITY;
}

void MotorController::setPosition(int64_t position) {
  _targetPosition = position;
  _posPID.setTarget(_targetPosition);
  _posPID.setOutputBounded(false);
  _mode = POSITION;
}

void MotorController::setPosition(int64_t position, int16_t velocity) {
  _targetPosition = position;
  _targetVelocity = velocity;
  _posPID.setTarget(_targetPosition);
  _posPID.setOutputBounds(-_targetVelocity, _targetVelocity);
  _mode = VEL_POSITION;
}

int MotorController::getVelocity(void) { return _motor->getVelocity(); }

int MotorController::getPosition(void) { return _motor->getPosition(); }

void MotorController::run(void) {
  switch (_mode) {
  case VELOCITY:
    _velPID.tick();
    break;

  case POSITION:
    _posPID.tick();
    break;

  case VEL_POSITION:
    _posPID.tick();
    _velPID.tick();
    break;

  default:
    break;
  }
}

// PID����������
static void pidCalculationTask(void *pvParameters) {
  MotorController *controller = static_cast<MotorController *>(pvParameters);
  BaseType_t xWasDelayed;
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    xWasDelayed = xTaskDelayUntil(
        &lastWakeTime, pdMS_TO_TICKS(controller->getPIDCalculationPeriod()));

    if (xWasDelayed == pdPASS) {
      controller->run();
    }
  }
}