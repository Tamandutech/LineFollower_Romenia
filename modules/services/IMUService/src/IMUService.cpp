#include "IMUService.hpp"

IMUService::IMUService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    this->get_Vel = robot->getMotorData();
    this->get_Spec = robot->getSpecification();
    this->get_Status = robot->getStatus();
    this->get_Marks = robot->getSLatMarks();
    this->get_latArray = robot->getLatSensors();
    this->get_centerArray = robot->getCenterSensors();
    
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    start_i2c();

}

void IMUService::Run()
{
    ESP_LOGE("Sensor", "Inicio.");
    // Loop do servico
    
}

void IMUService::updateLSM6DS3() {
    float ax=0.0, ay=0.0, az=0.0;
	if (imu.accelerationAvailable()) {
		imu.readAcceleration(ax, ay, az);
	}
	float gx=0.0, gy=0.0, gz=0.0;
	if (imu.gyroscopeAvailable()) {
		imu.readGyroscope(gx, gy, gz);
	}
	ESP_LOGD(GetName().c_str(), "acel=%f %f %f gyro=%f %f %f", ax, ay, az, gx, gy, gz);

	accX = ax;
	accY = ay;
	accZ = az;
	gyroX = gx;
	gyroY = gy;
	gyroZ = gz;
}

void IMUService::start_i2c(void) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)IMU_SDA;
	conf.scl_io_num = (gpio_num_t)IMU_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}