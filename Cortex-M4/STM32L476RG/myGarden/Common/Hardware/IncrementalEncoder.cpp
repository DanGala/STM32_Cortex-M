#include "SystemHeaders.h"

#ifdef USE_INCREMENTAL_ENCODER

/**
 * \brief Default c'tor
 */
IncrementalEncoder::IncrementalEncoder()
{
}

/**
 * \brief Initializer
 */
void IncrementalEncoder::Initialize()
{
	InitializeHardware();
}

#endif //#ifdef USE_INCREMENTAL_ENCODER