#ifndef __PLANT_H_
#define __PLANT_H_

class Plant
{
public:
	Plant(const char * name, float minMoisture);
	Plant(const char * name, uint8_t health, float minMoisture, bool waterNeeded);
	void UpdateHealth(float moisture);
	bool NeedsWatered();
	
private:
 	/** Health penalty in % for every update where moisture is out of range */
	static constexpr uint8_t MOISTURE_BELOW_LIMIT_PENALTY = 10u;
	/** Health recovery in % for every update where moisture is within limits */
	static constexpr uint8_t MOISTURE_OK_RECOVERY = 20u;
	/** Maximum valid health in % */
	static constexpr uint8_t FULL_HEALTH = 100u;
	/** Health threshold for watering */
	static constexpr uint8_t WATERING_THRESHOLD = 0;

	const char * name; /**< String identifying the plant by name */
	uint8_t health; /**< Health bar percentage (%) */
	float minMoisture; /**< Minimum volummetric water content in % */
	bool waterNeeded; /**< Flag to indicate when the soil is too dry for this plant's continued health */
};

#endif //#ifndef __PLANT_H_