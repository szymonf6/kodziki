#include "CProximityController.hpp"
#include <stdio.h>

namespace SwC
{
    CProximityController::CProximityController(Rte::IRTE &a_roIRTE) : m_roIRTE(a_roIRTE)
    {
        m_u64RawData = 0u;
        m_u64DistanceCentimeters = 0u;
    }

    ReturnCode CProximityController::run()
    {
        ReturnCode returnRun = RETURN_NOT_OK;
        // printf("Proxi run\n");
        eSigID_t idDriver = sigProximitySensorRawData;
        eSigID_t idSwc = sigProximitySensorDistance;

        if (RETURN_OK == m_roIRTE.readSignal(idDriver, m_u64RawData))
        {
            // printf("Proxi read signal\n");

            if (m_u64RawData > 0u)
            {
                m_u64DistanceCentimeters = m_u64RawData / 58;
                // printf("Proxi Distance: %llu\n", m_u64DistanceCentimeters);

                returnRun = RETURN_OK;
            }
        }

        m_roIRTE.writeSignal(idSwc, m_u64DistanceCentimeters);

        return returnRun;
    }

    ReturnCode CProximityController::init()
    {
        return RETURN_OK;
    }

    ReturnCode CProximityController::deinit()
    {
        return RETURN_OK;
    }

    CProximityController::~CProximityController()
    {
    }
}