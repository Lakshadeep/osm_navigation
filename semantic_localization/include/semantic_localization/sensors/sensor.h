/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "../pf/pf.h"

// Forward declarations
class SensorData;


// Base class for all sensors
class Sensor
{

public:
    Sensor();                                              // Default constructor
    virtual ~Sensor();                                     // Default destructor
    virtual bool UpdateAction(pf_t *pf, SensorData *data); // Update the filter based on the action model.
                                                           // Returns true if the filter
                                                           // has been updated.
    virtual bool InitSensor(pf_t *pf, SensorData *data);   // Initialize the filter based on the sensor model.
                                                           // Returns true if the filter has been initialized.
    virtual bool UpdateSensor(pf_t *pf, SensorData *data); // Update the filter based on the sensor model.
                                                           // Returns true if the filter has been updated.

    bool is_action;                                        // Flag is true if this is the action sensor
    pf_vector_t pose;                                      // Action pose (action sensors only)
};



// Base class for all sensor measurements
class SensorData
{
    
public:
    Sensor *sensor;              // Pointer to sensor that generated the data
    virtual ~SensorData() {}
    double time;                 // Data timestamp
};

#endif
