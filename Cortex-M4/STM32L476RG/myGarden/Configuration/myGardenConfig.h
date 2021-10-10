/**
 * @file myGardenConfig.h
 * @author DGM
 * \brief Configuration symbols definition for myGarden project
 */

#define POT_COUNT 3

#define POTS_INIT { \
    Pot(Plant("Opuntia", 3.0f), RotatingPlate(1.27, 0.3), Pump(), MoistureSensor(0)), \
	Pot(Plant("Montsera Deliciosa", 2.0f), RotatingPlate(2.7, 1.3), Pump(), MoistureSensor(1)), \
	Pot(Plant("Helianthus annuus", 1.8f), RotatingPlate(2.0, 0.7), Pump(), MoistureSensor(2)) \
}

#define ADC_CHANNEL_COUNT 3

#define ADC_CHANNELS { \
    ADCChannel("MoistureSense1", 1.0, 0.0), \
    ADCChannel("MoistureSense2", 1.0, 0.0), \
    ADCChannel("MoistureSense3", 1.0, 0.0) \
}
