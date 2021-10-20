/**
 * @file IncrementalEncoder.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-20
 * @copyright Copyright (c) 2021
 */

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