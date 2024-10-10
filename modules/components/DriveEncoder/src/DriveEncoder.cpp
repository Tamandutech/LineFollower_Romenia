#include "DriveEncoder.h"

void DriveEncoder::ConfigEncoders() {
  // Atalhos:
  robot = Robot::getInstance();
  get_Vel = robot->getMotorData();
  get_Spec = robot->getSpecification();

  // GPIOs dos encoders dos encoders dos motores
  enc_motEsq.attachFullQuad(enc_eq_A, enc_eq_B);
  enc_motDir.attachFullQuad(enc_dir_A, enc_dir_B);

  ResetCount();
}

void DriveEncoder::ReadBoth() {
  lastPulseRight = enc_motDir.getCount();
  lastPulseLeft = enc_motEsq.getCount();

  SaveEncData(lastPulseRight, lastPulseLeft);
}

void DriveEncoder::ResetCount() {
  enc_motEsq.clearCount();
  enc_motDir.clearCount();

  SaveEncData(0, 0);
}

void DriveEncoder::SaveEncData(int32_t pulseRight, int32_t pulseLeft) {
  get_Vel->EncRight->setData(pulseRight);
  get_Vel->EncLeft->setData(pulseLeft);
  get_Vel->EncMedia->setData(abs((pulseRight + pulseLeft) / 2));
}