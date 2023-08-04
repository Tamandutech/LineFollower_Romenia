#include "IMUService.hpp"

IMUService::IMUService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    // Atalhos de dados:
    this->robot = Robot::getInstance();
    
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)IMU_SDA;
	conf.scl_io_num = (gpio_num_t)IMU_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	imu.begin();
	imu.Enable_X();
	imu.Enable_G();
}

void IMUService::Run()
{
    ESP_LOGE("Sensor", "Inicio.");
    // Loop do servico
    
}

void IMUService::updateLSM6DS3() 
{// Le os valores atuais da IMU
	imu.Get_X_Axes(acc);  
	imu.Get_G_Axes(gyr);

	ESP_LOGD(GetName().c_str(), "acel=%f %f %f gyro=%f %f %f", acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
}