/**
 * @file myGardenConfig.h
 * @author DGM
 * \brief Configuration symbols definition for myGarden project
 */

#define POT_COUNT 3

#define POTS_INIT { \
	Pot(Plant("Opuntia", 40.0f), RotatingPlate(1.27, 0.3), Pump(), MoistureSensor(0)), \
	Pot(Plant("Montsera Deliciosa", 25.0f), RotatingPlate(2.7, 1.3), Pump(), MoistureSensor(1)), \
	Pot(Plant("Helianthus annuus", 30.0f), RotatingPlate(2.0, 0.7), Pump(), MoistureSensor(2)) \
}

#define ADC_CHANNEL_COUNT 3

#define ADC_CHANNELS { \
	ADCChannel(-61.6598, 3.21863), \
	ADCChannel(-61.6598, 3.21863), \
	ADCChannel(-61.6598, 3.21863) \
}

#define USE_INCREMENTAL_ENCODER

#define ENCODER_CPR 12

#define MOTOR_UMAX 5

#define GEAR_RATIO 298

#define USE_BDC_MOTOR
