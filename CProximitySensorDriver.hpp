#ifndef CDISTANCESENSORDRIVER_HPP
#define CDISTANCESENSORDRIVER_HPP

#include <driver/gpio.h>

#include "DBC.hpp"
#include "ReturnCode.hpp"
#include "IDriver.hpp"
#include "IDriverManager.hpp"
#include <unistd.h>
#include <esp_timer.h>
#include <esp32/rom/ets_sys.h>
#include "CThread.hpp"
#include <driver/gpio.h>
#include <esp_err.h>

#define ESP_ERR_ULTRASONIC_PING 0x200
#define ESP_ERR_ULTRASONIC_PING_TIMEOUT 0x201
#define ESP_ERR_ULTRASONIC_ECHO_TIMEOUT 0x202
#define TRIG_PIN GPIO_NUM_19
#define ECHO_PIN GPIO_NUM_21

typedef struct
{
    gpio_num_t trigger_pin; //!< GPIO output pin for trigger
    gpio_num_t echo_pin;    //!< GPIO input pin for echo
} ultrasonic_sensor_t;

/**
 * @namespace
 * @brief
 */
namespace Drv
{
    /**
     * @class   CProximitySensorDriver
     * @brief   Initialize, deinitialize, start and stop the sensor
     */
    class CProximitySensorDriver : public IDriver, public Utils::CThread
    {
    public:
        /**
         * @fn      CProximitySensorDriver
         * @brief   Construct a new CDistanceSensorDriver object
         * @param   a_roIDM
         */
        CProximitySensorDriver(Srv::IDriverManager &a_roIDM);

        /**
         * @fn      init
         * @brief   Initialize the sensor
         * @return  ReturnCode
         */
        ReturnCode init();

        /**
         * @fn      deinit
         * @brief   Deinitialize the sensor
         * @return  ReturnCode
         */
        ReturnCode deinit();

        /**
         * @fn      start
         * @brief   Sets the pins to high level
         * @return  ReturnCode
         */
        ReturnCode start();

        /**
         * @fn      stop
         * @brief   Sets low level to pins
         * @return  ReturnCode
         */
        ReturnCode stop();

        /**
         * @fn      sendDataToDriver
         * @brief   Sends the data to DriverManager
         * @param   a_data
         * @return  ReturnCode
         */
        ReturnCode sendDataToDriver(const MsgGeneric &a_roMsg);

        void run();
        /**
         * @fn      ~CProximitySensorDriver
         * @brief   Destroy the CDistanceSensorDriver object
         */
        ~CProximitySensorDriver();

    private:
        /**
         * @var     m_roIDM
         * @brief   reference to DriverManager object attached in constructor.
         */
        Srv::IDriverManager &m_roIDM;

    private:
        /**
         * @fn      CProximitySensorDriver
         * @brief   Construct a new CDistanceSensorDriver constructor and set as private
         */
        CProximitySensorDriver();

        /**
         * @fn      CProximitySensorDriver
         * @brief   Construct a copy constructor and set as private
         */
        CProximitySensorDriver(const CProximitySensorDriver &);

        /**
         * @var     m_u32InputData
         * @brief   variable to store the time data if it's rising edge on trigger pin, set as private to avoid misues
         */
        uint32_t m_u32InputData;

        /**
         * @var     m_u32OutputData
         * @brief   variable to store the time data if it's rising edge on echo pin, set as private to avoid misues
         */
        uint32_t m_u32OutputData;

        /**
         * @var     m_u32RawData
         * @brief   variable to store the time data if it's rising edge on echo pin, set as private to avoid misues
         */
        uint32_t m_u32RawData;
        /**
         * @var     m_u32RawData
         * @brief   variable to store the time data if it's rising edge on echo pin, set as private to avoid misues
         */

        ultrasonic_sensor_t m_sSensor;

        /**
         * Device descriptor
         */

        /**
         * @brief Init ranging module
         *
         * @param dev Pointer to the device descriptor
         * @return `ESP_OK` on success
         */
        esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);

        /**
         * @brief Measure time between ping and echo
         *
         * @param dev Pointer to the device descriptor
         * @param max_time_us Maximal time to wait for echo
         * @param[out] time_us Time, us
         * @return `ESP_OK` on success, otherwise:
         *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
         *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
         *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
         */
        esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t max_time_us, uint32_t *time_us);

        /**
         * @brief Measure distance in meters
         *
         * @param dev Pointer to the device descriptor
         * @param max_distance Maximal distance to measure, meters
         * @param[out] distance Distance in meters
         * @return `ESP_OK` on success, otherwise:
         *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
         *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
         *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
         */
        esp_err_t ultrasonic_measure(const ultrasonic_sensor_t *dev, float max_distance, float *distance);

        /**
         * @brief Measure distance in centimeters
         *
         * @param dev Pointer to the device descriptor
         * @param max_distance Maximal distance to measure, centimeters
         * @param[out] distance Distance in centimeters
         * @return `ESP_OK` on success, otherwise:
         *         - ::ESP_ERR_ULTRASONIC_PING         - Invalid state (previous ping is not ended)
         *         - ::ESP_ERR_ULTRASONIC_PING_TIMEOUT - Device is not responding
         *         - ::ESP_ERR_ULTRASONIC_ECHO_TIMEOUT - Distance is too big or wave is scattered
         */
        esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance, uint32_t *distance);
    };
}

#endif // CDISTANCESENSORDRIVER_HPP