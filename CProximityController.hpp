#ifndef CPROXIMITYCONTROLLER_HPP
#define CPROXIMITYCONTROLLER_HPP

#include "ISWC.hpp"
#include "IRTE.hpp"
#include "DBC.hpp"

namespace SwC
{
    /**
     * @class   CProximityController
     * @brief   Returns a value of distance in centimeters
     */
    class CProximityController : public ISWC
    {
    public:
        /**
         * @fn      CProximityController(Rte::IRTE& a_roIRTE)
         * @brief   Create Buzzer object with access to IRTE interface methods.
         * @param   a_roIRTE
         */
        CProximityController(Rte::IRTE &a_roIRTE);
        /**
         * @fn      ReturnCode run()
         * @brief   Function that measures the distance
         * @return  ReturnCode
         */
        ReturnCode run();
        /**
         * @fn      ReturnCode init()
         * @brief   Initialized the ProximityController
         * @return  ReturnCode
         */
        ReturnCode init();
        /**
         * @fn      ReturnCode deinit()
         * @brief   Deinitialize the ProximityController
         * @return  ReturnCode
         */
        ReturnCode deinit();

        /**
         * @fn      ~CProximityController()
         * @brief    Destroy the CProximityController object
         */
        ~CProximityController();

    private:
        /**
         * @var     m_roIRTE
         * @brief   Allow to use RTE methods
         */
        Rte::IRTE &m_roIRTE;

        /**
         * @var     m_u64DistanceCentimeters
         * @brief   store a value of distance
         */
        uint64_t m_u64DistanceCentimeters;

        /**
         * @var     m_u64RawData
         * @brief   store a raw value of distance
         */
        uint64_t m_u64RawData;

    private:
        /**
         * @fn      CProximityController()
         * @brief   Construct a new CProximityController object
         */
        CProximityController();
        /**
         * @fn      CProximityController(const CProximityController&)
         * @brief   Construct a new CProximityController object
         */
        CProximityController(const CProximityController &);
    };
}

#endif // CPROXIMITYCONTROLLER_HPP