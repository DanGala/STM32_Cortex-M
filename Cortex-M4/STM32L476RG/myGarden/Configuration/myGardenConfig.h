/**
 * @file myGardenConfig.h
 * @author DGM
 * \brief Configuration symbols definition for myGarden project
 */

#define ADC_CHANNEL_COUNT 3

#define ADC_CHANNELS { \
    ADCChannel("MoistureSense1", 1.0, 0.0), \
    ADCChannel("MoistureSense2", 1.0, 0.0), \
    ADCChannel("MoistureSense3", 1.0, 0.0) \
}
