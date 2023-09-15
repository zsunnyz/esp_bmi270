#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include "esp_bmi270.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"

/// @brief Default constructor
BMI270::BMI270() {
    // Nothing to do
}

BMI270::~BMI270() {
    spi_bus_remove_device(this->interfaceData.spi_device);
}

/** 
 * @brief Checks whether sensor is connected, initializes sensor, then sets
 * default config parameters
 * @return Error code (0 is success, negative is failure, positive is warning)
 */ 
int8_t BMI270::begin() {
    // Variable to track errors returned by API calls
    int8_t err = BMI2_OK;

    // Set helper function pointers
    this->sensor.read = this->readRegisters;
    this->sensor.write = this->writeRegisters;
    this->sensor.delay_us = this->usDelay;
    this->sensor.intf_ptr = &(this->interfaceData);
    this->sensor.read_write_len = 32;

    // Initialize the sensor
    ESP_LOGI(BMI_TAG, "Initialising the sensor");
    // Need this line to ensure that the config file gets set properly
    this->sensor.config_file_ptr = NULL;
    err = bmi270_init(&this->sensor);
    if(err != BMI2_OK) return err;

    ESP_LOGI(BMI_TAG, "IMU SPI features");
    // Enable the accelerometer and gyroscope
    uint8_t features[] = {BMI2_ACCEL, BMI2_GYRO};
    err = enableFeatures(features, 2);
    if(err != BMI2_OK) return err;

    // Get the accelerometer and gyroscope configs
    ESP_LOGI(BMI_TAG, "IMU SPI device retrieving configs");
    bmi2_sens_config configs[2];
    configs[0].type = BMI2_ACCEL;
    configs[1].type = BMI2_GYRO;
    err = getConfigs(configs, 2);
    if(err != BMI2_OK) return err;
    
    ESP_LOGI(BMI_TAG, "IMU SPI device retreived configs");
    // Store the accelerometer and gyroscope ranges, these are needed elsewhere
    rawToGs = convertRawToGsScalar(configs[0].cfg.acc.range);
    rawToDegSec = convertRawToDegSecScalar(configs[1].cfg.gyr.range);

    // Done!
    return BMI2_OK;
}

/**
 * @brief Simple way to initialise the IMU as a device on the SPI bus, start communications
 *          and set default configurations
 * 
 * @param spi_host: The SPI peripheral the imu is connected to, can be:
 *                      - SPI1_HOST
 *                      - SPI2_HOST
 *                      - SPI3_HOST
 * @param imu_CS_num: The chip select GPIO pin number for the BMI270
 * @param ret_imu_spi_dev: The function sets the return IMU SPI device struct
 * @param clock_speed: Clock speed of the IMU in hz, default is 100kHz
 * @param queue_size: The queue size for SPI transactions, default 3
 * @return (int8_t): Error code (0 is success, negative is failure, positive is warning)
 */
int8_t BMI270::beginSPI(spi_host_device_t spi_host, gpio_num_t imu_CS_num, spi_device_interface_config_t* ret_imu_spi_dev, int clock_speed, int queue_size) {
    *ret_imu_spi_dev = {.command_bits=0, 
                        .address_bits=8, 
                        .dummy_bits=0, 
                        .mode=0, 
                        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
                        .clock_speed_hz=clock_speed, 
                        .spics_io_num=imu_CS_num,
                        .flags = SPI_DEVICE_HALFDUPLEX, 
                        .queue_size=queue_size};
    
    this->interfaceData.spi_host = spi_host;
    this->interfaceData.spi_intf_conf = ret_imu_spi_dev;

    BMI2_INTF_RETURN_TYPE err = this->SPI_dev_init();
    if (err != ESP_OK) {
        return err; 
    }

    // Initialise sensor
    return this->begin(); 
}

/**
 * @brief Initialize the IMU on the SPI bus and begin communication with the sensor over SPI, 
 * intiialize it, and set default config parameters
 * 
 * @param spi_host: The SPI peripheral the imu is connected to, can be:
 *                      - SPI1_HOST
 *                      - SPI2_HOST
 *                      - SPI3_HOST
 * @param device_conf: A pointer to the configuration for a SPI slave device 
 * @return (int8_t): Error code (0 is success, negative is failure, positive is warning)
 * 
 * @note the SPI bus should already be initialised by calling spi_bus_initialize before calling this function
 */
int8_t BMI270::beginSPI(spi_host_device_t spi_host, spi_device_interface_config_t *device_conf) {
    this->interfaceData.spi_host = spi_host;
    this->interfaceData.spi_intf_conf = device_conf;

    BMI2_INTF_RETURN_TYPE err = this->SPI_dev_init();
    if (err != ESP_OK) {
        return err; 
    }

    // Initialise sensor
    return this->begin();

}

/**
 * @brief Helper function to add the IMU as an SPI bus
 * 
 * @return (BMI2_INTF_RETURN_TYPE): Error code (0 is success, negative is failure, positive is warning)
 */
BMI2_INTF_RETURN_TYPE BMI270::SPI_dev_init() {
    esp_err_t err = spi_bus_add_device(this->interfaceData.spi_host, this->interfaceData.spi_intf_conf, &(this->interfaceData.spi_device));
    if (err != ESP_OK) {
        ESP_LOGE(BMI_TAG, "Could not add device!, got error: %s", esp_err_to_name(err));
        return BMI2_E_DEV_NOT_FOUND;
    }

    // Set interface
    this->sensor.intf = BMI2_SPI_INTF;
    this->interfaceData.interface = BMI2_SPI_INTF;

    this->sensor.dummy_byte = 1;

    ESP_LOGI(BMI_TAG, "IMU SPI device successfully initialised");
    return BMI2_OK;
}

/**
 * @brief Helper function to read sensor registers
 * 
 * @param regAddress: Start address to read
 * @param dataBuffer: Buffer to store register values
 * @param numBytes: Number of bytes to read
 * @param interfacePtr: Pointer to interface data, see BMI270_InterfaceData
 * @return (BMI2_INTF_RETURN_TYPE): (0 is success, negative is failure, positivie is warning)
 */
BMI2_INTF_RETURN_TYPE BMI270::readRegisters(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr) {
    // Make sure the number of bytes is valid
    if (numBytes == 0) {
        return BMI2_E_COM_FAIL;
    }

    BMI270_InterfaceData* interfaceData = (BMI270_InterfaceData *) interfacePtr;

    switch (interfaceData -> interface) {
        case BMI2_SPI_INTF:
            return readRegistersSPI(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        // case BMI2_I2C_INTF:
        //     return readRegistersI2C(regAddress, dataBuffer, numBytes, interfaceData);
        //     break;
        default:
            return BMI2_E_COM_FAIL;
            break;
    }
}

/**
 * @brief Helper function to read sensor registers over SPI
 * 
 * @param regAddress: Start address to read
 * @param dataBuffer: Buffer to store register values
 * @param numBytes: Number of bytes to read
 * @param interfaceData: Pointer to interface data
 * @return (BMI2_INTF_RETURN_TYPE): (0 is success, negative is failure, positivie is warning)
 */
BMI2_INTF_RETURN_TYPE BMI270::readRegistersSPI(uint8_t regAddress, uint8_t* dataBuffer, uint32_t numBytes, BMI270_InterfaceData* interfaceData) {
    spi_transaction_t trans = {.addr=regAddress,
                                .length = 0, 
                                .rxlength=numBytes*8, 
                                .rx_buffer=dataBuffer};

    ESP_LOGV(BMI_TAG, "Reading to register 0x%x with information at address %p", regAddress, dataBuffer);
    esp_err_t err = spi_device_polling_transmit(interfaceData->spi_device, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(BMI_TAG, "Failed to transmit to read register %x, error %s", regAddress, esp_err_to_name(err));
        return BMI2_E_COM_FAIL;
    }
    return BMI2_OK;
}

/**
 * @brief Helper function to write sensor registers over SPI
 * 
 * @param regAddress: Start address to write
 * @param dataBuffer: Buffer to store register values
 * @param numBytes: Number of bytes to write
 * @param interfacePtr: Pointer to interface data
 * @return (BMI2_INTF_RETURN_TYPE): (0 is success, negative is failure, positivie is warning)
 */
BMI2_INTF_RETURN_TYPE BMI270::writeRegisters(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, void* interfacePtr) {
    // Make sure the number of bytes is valid
    if (numBytes == 0) {
        return BMI2_E_COM_FAIL;
    }

    BMI270_InterfaceData* interfaceData = (BMI270_InterfaceData *) interfacePtr;

    switch (interfaceData->interface) {
        case BMI2_SPI_INTF:
            return writeRegistersSPI(regAddress, dataBuffer, numBytes, interfaceData);
            break;
        // case BMI2_I2C_INTF:
        //     return writeRegistersI2C(regAddress, dataBuffer, numBytes, interfaceData);
        //     break;
        default:
            return BMI2_E_COM_FAIL;
            break;
    }
}

/**
 * @brief Helper function to write sensor registers over SPI
 * 
 * @param regAddress: Start address to write
 * @param dataBuffer: Buffer to store register values
 * @param numBytes: Number of bytes to write
 * @param interfaceData: Pointer to interface data
 * @return (BMI2_INTF_RETURN_TYPE): (0 is success, negative is failure, positivie is warning)
 */
BMI2_INTF_RETURN_TYPE BMI270::writeRegistersSPI(uint8_t regAddress, const uint8_t* dataBuffer, uint32_t numBytes, BMI270_InterfaceData* interfaceData) {
    spi_transaction_t trans = {.addr=regAddress, 
                                .length = numBytes*8, 
                                .rxlength=0, 
                                .tx_buffer=dataBuffer};

    ESP_LOGV(BMI_TAG, "Writing to register 0x%x with information at address %p", regAddress, dataBuffer);
    esp_err_t err = spi_device_polling_transmit(interfaceData->spi_device, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(BMI_TAG, "Failed to write register %x, error %s", regAddress, esp_err_to_name(err));
        return BMI2_E_COM_FAIL;
    }

    return BMI2_OK;
}

/**
 * @brief Helper function to delay for some amount of time
 * 
 * @param period: Number of microseconds to delay
 * @param interfacePtr: Pointer to interface data
 */
void BMI270::usDelay(uint32_t period, void* interfacePtr)
{
    ets_delay_us(period);
}

/**
 * @brief Enables features of teh sensor
 * 
 * @param features: Array of features to be enabled, see bmi270_sensor_enable for possible values
 * @param numFeatures:Size of features array
 * @return (int8_t): (0 is success, negative is failure, positive is warning)
 */
int8_t BMI270::enableFeatures(uint8_t* features, uint8_t numFeatures)
{
    return bmi270_sensor_enable(features, numFeatures, &sensor);
}

/**
 * @brief Disables features of teh sensor
 * 
 * @param features: Array of features to be enabled, see bmi270_sensor_disable for possible values
 * @param numFeatures:Size of features array
 * @return (int8_t): (0 is success, negative is failure, positive is warning)
 */
int8_t BMI270::disableFeatures(uint8_t* features, uint8_t numFeatures)
{
    return bmi270_sensor_disable(features, numFeatures, &sensor);
}

/**
 * @brief Gets configuration parameters for sensor features, such as the
 * sensors or interrupts
 * @param configs Array of configs to be set. Possible types include:
 *     BMI2_ACCEL
 *     BMI2_GYRO
 *     BMI2_AUX
 *     BMI2_GYRO_GAIN_UPDATE
 *     BMI2_ANY_MOTION
 *     BMI2_NO_MOTION
 *     BMI2_SIG_MOTION
 *     BMI2_STEP_COUNTER_PARAMS
 *     BMI2_STEP_DETECTOR
 *     BMI2_STEP_COUNTER
 *     BMI2_STEP_ACTIVITY
 *     BMI2_WRIST_GESTURE
 *     BMI2_WRIST_WEAR_WAKE_UP
 * @param numConfigs Size of configs array
 * @return Error code (0 is success, negative is failure, positive is warning)
 */
int8_t BMI270::getConfigs(bmi2_sens_config* configs, uint8_t numConfigs)
{
    return bmi270_get_sensor_config(configs, numConfigs, &sensor);
}