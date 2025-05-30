/*
Name:		RegisterHandler.h
Created:	24/Aug/2022
Author:		Daniel Young

This file is part of Alpha2MQTT (A2M) which is released under GNU GENERAL PUBLIC LICENSE.
See file LICENSE or go to https://choosealicense.com/licenses/gpl-3.0/ for full license details.

Notes

Handles AlphaESS specific functions as of v1.23 AlphaESS Modbus documentation
Namely:
	0x03 READ DATA REGISTER
	0x06 WRITE SINGLE REGISTER
	0x0A WRITE DATA REGISTER

I presumed when I started that WRITE SINGLE REGISTER was for single two byte registers, and was
put in to provide a simpler way to write the bulk of the smaller registers without needing to worry
about sending over data lengths, etc.  On my Smile B3 at least, WRITE SINGLE REGISTER does nothing,
it does not attempt any data changes and it does not yield any response (nor slave error) from the
system.  I found that WRITE DATA REGISTER will write any appropriate single (2 byte) or double (4 byte)
register, and we just leverage 'Number of bytes' to guide it accordingly.
*/
#include "RegisterHandler.h"

/*
Default Constructor

Initialisations and what not
*/
RegisterHandler::RegisterHandler()
{

}

/*
Constructor which allows setting the RS485 class level variable

Initialisations and what not
*/
RegisterHandler::RegisterHandler(RS485Handler* modBus)
{
	_modBus = modBus;
}

/*
Default Destructor

Disconnections, clean-up and what not
*/
RegisterHandler::~RegisterHandler()
{
	_modBus = NULL;
}

/*
setSerialNumberPrefix

To allow appropriate error codes to be spat out for AL or AE based serial numbers
*/
void RegisterHandler::setSerialNumberPrefix(uint8_t char1, uint8_t char2)
{
	_serialNumberPrefix[0] = char1;
	_serialNumberPrefix[1] = char2;
}

/*
setModbus

Set the class level variable for RS485
*/
void RegisterHandler::setModbus(RS485Handler* modBus)
{
	_modBus = modBus;
}



/*
readHandledRegister

This will perform validation, sense checking and cleansed / appropriately cast results for a whole
raft of registers (300+.)  If the request is for a register which isn't handled it will throw back an appropriate error
*/
modbusRequestAndResponseStatusValues RegisterHandler::readHandledRegister(uint16_t registerAddress, modbusRequestAndResponse* rs)
{
	// To account for custom registers just before sending
	uint16_t registerAddressToSend;
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;

	// Slave Address
	// Function Code
	// Starting Address High
	// Starting Address Low, 
	// Number Of Registers High Byte
	// Number Of Registers Low Byte
	// CRC Low Byte
	// CRC High Byte

	// For custom registers
	int16_t batteryPower = 0;
	int32_t pvPower = 0;
	int32_t gridPower = 0;
	uint16_t gridVoltage = 0;

	// Determine number of registers/data type/mqtt name based on register passed in
	switch (registerAddress)
	{

	case REG_GRID_METER_RW_GRID_METER_CT_ENABLE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_RW_GRID_METER_CT_RATE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}

	case REG_GRID_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}

	case REG_GRID_METER_R_VOLTAGE_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_VOLTAGE_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_VOLTAGE_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_CURRENT_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_CURRENT_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_CURRENT_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_FREQUENCY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}

	case REG_GRID_METER_R_ACTIVE_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}

	case REG_GRID_METER_R_ACTIVE_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}

	case REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_REACTIVE_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_REACTIVE_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_REACTIVE_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_TOTAL_REACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_APPARENT_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_APPARENT_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_APPARENT_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_TOTAL_APPARENT_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_GRID_METER_R_POWER_FACTOR_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_POWER_FACTOR_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_POWER_FACTOR_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_GRID_METER_R_TOTAL_POWER_FACTOR:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_RW_PV_METER_CT_ENABLE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_RW_PV_METER_CT_RATE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_VOLTAGE_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_VOLTAGE_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_VOLTAGE_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_CURRENT_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_CURRENT_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_CURRENT_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_FREQUENCY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_ACTIVE_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_ACTIVE_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_ACTIVE_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_TOTAL_ACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_REACTIVE_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_REACTIVE_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_REACTIVE_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_TOTAL_REACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_APPARENT_POWER_OF_A_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_APPARENT_POWER_OF_B_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_APPARENT_POWER_OF_C_PHASE_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_TOTAL_APPARENT_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_PV_METER_R_POWER_FACTOR_OF_A_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_POWER_FACTOR_OF_B_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_POWER_FACTOR_OF_C_PHASE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_PV_METER_R_TOTAL_POWER_FACTOR:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_STATUS:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_BATTERY_HOME_R_RELAY_STATUS:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MIN_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MAX_CELL_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MIN_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MAX_CELL_TEMPERATURE:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MAX_CHARGE_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_MAX_DISCHARGE_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_CHARGE_CUT_OFF_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_DISCHARGE_CUT_OFF_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BMU_SOFTWARE_VERSION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_LMU_SOFTWARE_VERSION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_ISO_SOFTWARE_VERSION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_NUMBER:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_CAPACITY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_TYPE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_SOH:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
//		rs->hasLookup = true;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_CHARGE_ENERGY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_DISCHARGE_ENERGY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_ENERGY_CHARGE_FROM_GRID_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_POWER:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_REMAINING_TIME:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_CHARGE_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_DISCHARGE_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_REMAINING_CHARGE_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_REMAINING_DISCHARGE_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_MAX_CHARGE_POWER:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_MAX_DISCHARGE_POWER:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_RW_BATTERY_MOS_CONTROL:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
//		rs->hasLookup = true;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_SOC_CALIBRATION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
//		rs->hasLookup = true;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_SINGLE_CUT_ERROR_CODE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_3_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_4_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_5_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_FAULT_6_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_3_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_4_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_5_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_BATTERY_HOME_R_BATTERY_WARNING_6_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_VOLTAGE_L1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_VOLTAGE_L2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_VOLTAGE_L3:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_CURRENT_L1:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_CURRENT_L2:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_CURRENT_L3:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_POWER_L1_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_POWER_L2_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_POWER_L3_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_POWER_TOTAL_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L3:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_CURRENT_L1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_CURRENT_L2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_CURRENT_L3:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_POWER_L1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_POWER_L2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_POWER_L3_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_BACKUP_POWER_TOTAL_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_FREQUENCY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV1_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV1_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV1_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_PV2_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV2_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV2_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_PV3_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV3_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV3_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_PV4_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV4_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV4_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_PV5_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV5_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV5_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_PV6_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV6_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_PV6_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_TEMP:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_WARNING_1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_WARNING_2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_TOTAL_PV_ENERGY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_WORKING_MODE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
//		rs->hasLookup = true;
		rs->registerCount = 1;
		break;
	}
#ifdef EMS_35_36
	case REG_INVERTER_HOME_R_INVERTER_BAT_VOLTAGE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_BAT_CURRENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_BAT_POWER:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_TOTAL_REACT_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_TOTAL_APPARENT_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FREQUENCY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_BACKUP_FREQUENCY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_POWER_FACTOR:
	{
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_1_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_2_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_3_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_4_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
#endif // EMS_35_36
	case REG_INVERTER_HOME_R_PV_TOTAL_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_1:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 5;
		break;
	}
	case REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_1:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 5;
		break;
	}
	case REG_INVERTER_INFO_R_SERIAL_NUMBER_1:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 10;
		break;
	}
	case REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_1:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 5;
		break;
	}
	case REG_SYSTEM_INFO_RW_FEED_INTO_GRID_PERCENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_SYSTEM_FAULT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_SYSTEM_INFO_RW_SYSTEM_TIME_YEAR_MONTH:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_RW_SYSTEM_TIME_DAY_HOUR:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_RW_SYSTEM_TIME_MINUTE_SECOND:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_EMS_SN_BYTE_1_2:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 8;
		break;
	}
	case REG_SYSTEM_INFO_R_EMS_VERSION_HIGH:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_EMS_VERSION_MIDDLE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_EMS_VERSION_LOW:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_PROTOCOL_VERSION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_1:
	{
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 4;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_MAX_FEED_INTO_GRID_PERCENT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_PV_CAPACITY_STORAGE_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_PV_CAPACITY_OF_GRID_INVERTER_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_SYSTEM_MODE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_METER_CT_SELECT:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_BATTERY_READY:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_IP_METHOD:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_LOCAL_IP_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
//		rs->hasLookup = true; // Using this to treat output as string
		break;
	}
	case REG_SYSTEM_CONFIG_RW_SUBNET_MASK_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
//		rs->hasLookup = true; // Using this to treat output as string
		break;
	}
	case REG_SYSTEM_CONFIG_RW_GATEWAY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
//		rs->hasLookup = true; // Using this to treat output as string
		break;
	}
	case REG_SYSTEM_CONFIG_RW_MODBUS_ADDRESS:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_CONFIG_RW_MODBUS_BAUD_RATE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_TIMING_RW_TIME_PERIOD_CONTROL_FLAG:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_TIMING_RW_UPS_RESERVE_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_CHARGE_CUT_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_START_TIME_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_START_TIME_2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
#ifdef EMS_35_36
	case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_START_TIME_1_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_START_TIME_2_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2_MIN:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
#endif // EMS_35_36
	case REG_DISPATCH_RW_DISPATCH_START:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_DISPATCH_RW_ACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_DISPATCH_RW_REACTIVE_POWER_1:
	{
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_DISPATCH_RW_DISPATCH_MODE:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_DISPATCH_RW_DISPATCH_SOC:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_DISPATCH_RW_DISPATCH_TIME_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_DISPATCH_RW_DISPATCH_PARA_7:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_DISPATCH_RW_DISPATCH_PARA_8:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_AUXILIARY_R_EMS_DI0:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_AUXILIARY_R_EMS_DI1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_SYSTEM_OP_R_PV_INVERTER_ENERGY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_SYSTEM_OP_R_SYSTEM_FAULT_1:
	{
		rs->returnDataType = modbusReturnDataType::unsignedInt;
		rs->registerCount = 2;
//		rs->hasLookup = true;
		break;
	}
	case REG_SAFETY_TEST_RW_GRID_REGULATION:
	{
		rs->returnDataType = modbusReturnDataType::unsignedShort;
		rs->registerCount = 1;
//		rs->hasLookup = true;
		break;
	}
	case REG_CUSTOM_SYSTEM_DATE_TIME:
	{
		// Custom, going to do the date/time formatting, so swap back to register for YEAR/MONTH and pull 3 bytes.
		rs->returnDataType = modbusReturnDataType::character;
		rs->registerCount = 3;
		break;
	}
	case REG_CUSTOM_LOAD:
	{
		// Custom, derive a load using a combination of registers to substitute lack of load from a data register
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	case REG_CUSTOM_GRID_CURRENT_A_PHASE:
	{
		// Custom, derive a grid current using grid power / grid voltage, seeing as REG_GRID_METER_R_CURRENT_OF_A_PHASE is always zero for me
		rs->returnDataType = modbusReturnDataType::signedShort;
		rs->registerCount = 1;
		break;
	}
	case REG_CUSTOM_TOTAL_SOLAR_POWER:
	{
		// Custom, derive a safe total solar power using figures from either PV CT or in the case of hybrids, from the individual PV strings too.
		rs->returnDataType = modbusReturnDataType::signedInt;
		rs->registerCount = 2;
		break;
	}
	default:
	{
		// Not a valid register we have written code to handle, do something here to prevent the send
		result = modbusRequestAndResponseStatusValues::notHandledRegister;
		strcpy(rs->statusMqttMessage, MODBUS_REQUEST_AND_RESPONSE_NOT_HANDLED_REGISTER_MQTT_DESC);
		strcpy(rs->displayMessage, MODBUS_REQUEST_AND_RESPONSE_NOT_HANDLED_REGISTER_DISPLAY_DESC);
		break;
	}
	}


	// If a custom register address we've made up to do some of our own work, swap it around here.
	if (result == modbusRequestAndResponseStatusValues::preProcessing)
	{
		if (registerAddress == REG_CUSTOM_LOAD)
		{
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_SIGNED_INT_DESC);

			/*
			Load is not exposed by the inverter, so we need a custom routine to pull the three registers relevantand do the calculations on the chip.
			OK so theory is
			Cosumption is PV generating
			Plus the grid, providing that the grid IS pulling
			Plus the battery, providing that it is discharging AND grid not pulling(i.e. not forcibly discharging to grid)
			*/


			uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_PV_METER_R_TOTAL_ACTIVE_POWER_1 >> 8, REG_PV_METER_R_TOTAL_ACTIVE_POWER_1 & 0xff, 0, 2, 0, 0 };
			result = _modBus->sendModbus(frame, sizeof(frame), rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
			{
				pvPower = (int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);
				uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV1_POWER_1 >> 8, REG_INVERTER_HOME_R_PV1_POWER_1 & 0xff, 0, 2, 0, 0 };
				result = _modBus->sendModbus(frame, sizeof(frame), rs);
				if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
				{
					pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
					uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV2_POWER_1 >> 8, REG_INVERTER_HOME_R_PV2_POWER_1 & 0xff, 0, 2, 0, 0 };
					result = _modBus->sendModbus(frame, sizeof(frame), rs);
					if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
					{
						pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
						uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV3_POWER_1 >> 8, REG_INVERTER_HOME_R_PV3_POWER_1 & 0xff, 0, 2, 0, 0 };
						result = _modBus->sendModbus(frame, sizeof(frame), rs);
						if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
						{
							pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
							uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV4_POWER_1 >> 8, REG_INVERTER_HOME_R_PV4_POWER_1 & 0xff, 0, 2, 0, 0 };
							result = _modBus->sendModbus(frame, sizeof(frame), rs);
							if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
							{
								pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
								uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV5_POWER_1 >> 8, REG_INVERTER_HOME_R_PV5_POWER_1 & 0xff, 0, 2, 0, 0 };
								result = _modBus->sendModbus(frame, sizeof(frame), rs);
								if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
								{
									pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
									uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV6_POWER_1 >> 8, REG_INVERTER_HOME_R_PV6_POWER_1 & 0xff, 0, 2, 0, 0 };
									result = _modBus->sendModbus(frame, sizeof(frame), rs);
									if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
									{
										pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));

										if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
										{
											// Generate a frame without CRC (ending 0, 0), sendModbus will do the rest
											uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1 >> 8, REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1 & 0xff, 0, 2, 0, 0 };
											// And send to the device, it's all synchronos so by the time we get a response we will know if success or failure
											result = _modBus->sendModbus(frame, sizeof(frame), rs);
											gridPower = (int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);
											if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
											{
												// Generate a frame without CRC (ending 0, 0), sendModbus will do the rest
												uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_BATTERY_HOME_R_BATTERY_POWER >> 8, REG_BATTERY_HOME_R_BATTERY_POWER & 0xff, 0, 1, 0, 0 };
												// And send to the device, it's all synchronos so by the time we get a response we will know if success or failure
												result = _modBus->sendModbus(frame, sizeof(frame), rs);
												if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
												{
													batteryPower = (int16_t)(rs->data[0] << 8 | rs->data[1]);
													rs->signedIntValue =
														pvPower
														-
														// Minus feeding, if feeding
														(gridPower < 0 ? (int32_t)abs(gridPower) : (int32_t)0)
														+
														// Plus purchase, if purchasing
														(gridPower > 0 ? gridPower : (int32_t)0)
														-
														// Minus battery, if charging
														(batteryPower < 0 ? (int32_t)abs(batteryPower) : (int32_t)0)
														+
														// Plus battery, if discharging
														((int32_t)batteryPower > 0 ? (int32_t)batteryPower : (int32_t)0);

													rs->dataSize = 4;
													rs->data[0] = rs->signedIntValue >> 24;
													rs->data[1] = rs->signedIntValue >> 16;
													rs->data[2] = rs->signedIntValue >> 8;
													rs->data[3] = rs->signedIntValue & 0xff;
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}

		}
		else if(registerAddress == REG_CUSTOM_GRID_CURRENT_A_PHASE)
		{
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_SIGNED_SHORT_DESC);

			/*
			Grid Current (Phase A) is not exposed by my inverter, so I am using a custom routine to pull the two registers relevant and do the calculations on the chip.
			OK so theory is
			I = P/V
			Ensure V > 0 to avoid division by zero
			*/

			uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_1 >> 8, REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_1 & 0xff, 0, 2, 0, 0 };
			result = _modBus->sendModbus(frame, sizeof(frame), rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
			{
				gridPower = (int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);

				uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_GRID_METER_R_VOLTAGE_OF_A_PHASE >> 8, REG_GRID_METER_R_VOLTAGE_OF_A_PHASE & 0xff, 0, 1, 0, 0 };
				result = _modBus->sendModbus(frame, sizeof(frame), rs);
				if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
				{
					gridVoltage = ((uint16_t)(rs->data[0] << 8 | rs->data[1])) * GRID_VOLTAGE_MULTIPLIER;

					rs->signedShortValue = (gridVoltage == 0 ? 0 : gridPower / gridVoltage);

					rs->dataSize = 2;
					rs->data[0] = rs->signedShortValue >> 8;
					rs->data[1] = rs->signedShortValue & 0xff;
				}
			}

		}
		else if (registerAddress == REG_CUSTOM_TOTAL_SOLAR_POWER)
		{
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_SIGNED_INT_DESC);

			/*
			PV if AC coupled is via  PV CT and results are stored in
			REG_PV_METER_R_TOTAL_ACTIVE_POWER_1

			PV if hybrid is via the individual string readings from within the Alpha, namely
			REG_INVERTER_HOME_R_PV1_POWER_1
			REG_INVERTER_HOME_R_PV2_POWER_1
			REG_INVERTER_HOME_R_PV3_POWER_1
			REG_INVERTER_HOME_R_PV4_POWER_1
			REG_INVERTER_HOME_R_PV5_POWER_1
			REG_INVERTER_HOME_R_PV6_POWER_1

			So essentially to get a solar reading which is safe across all types, we will just add all these up and present as a custom reg
			*/


			uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_PV_METER_R_TOTAL_ACTIVE_POWER_1 >> 8, REG_PV_METER_R_TOTAL_ACTIVE_POWER_1 & 0xff, 0, 2, 0, 0 };
			result = _modBus->sendModbus(frame, sizeof(frame), rs);
			if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
			{
				pvPower = (int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);
				uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV1_POWER_1 >> 8, REG_INVERTER_HOME_R_PV1_POWER_1 & 0xff, 0, 2, 0, 0 };
				result = _modBus->sendModbus(frame, sizeof(frame), rs);
				if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
				{
					pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
					uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV2_POWER_1 >> 8, REG_INVERTER_HOME_R_PV2_POWER_1 & 0xff, 0, 2, 0, 0 };
					result = _modBus->sendModbus(frame, sizeof(frame), rs);
					if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
					{
						pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
						uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV3_POWER_1 >> 8, REG_INVERTER_HOME_R_PV3_POWER_1 & 0xff, 0, 2, 0, 0 };
						result = _modBus->sendModbus(frame, sizeof(frame), rs);
						if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
						{
							pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
							uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV4_POWER_1 >> 8, REG_INVERTER_HOME_R_PV4_POWER_1 & 0xff, 0, 2, 0, 0 };
							result = _modBus->sendModbus(frame, sizeof(frame), rs);
							if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
							{
								pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
								uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV5_POWER_1 >> 8, REG_INVERTER_HOME_R_PV5_POWER_1 & 0xff, 0, 2, 0, 0 };
								result = _modBus->sendModbus(frame, sizeof(frame), rs);
								if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
								{
									pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
									uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER, REG_INVERTER_HOME_R_PV6_POWER_1 >> 8, REG_INVERTER_HOME_R_PV6_POWER_1 & 0xff, 0, 2, 0, 0 };
									result = _modBus->sendModbus(frame, sizeof(frame), rs);
									if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
									{
										pvPower = pvPower + ((int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]));
										rs->signedIntValue = pvPower;

										rs->dataSize = 4;
										rs->data[0] = rs->signedIntValue >> 24;
										rs->data[1] = rs->signedIntValue >> 16;
										rs->data[2] = rs->signedIntValue >> 8;
										rs->data[3] = rs->signedIntValue & 0xff;
									}
								}
							}
						}
					}
				}
			}

		}
		else
		{
			// Normal route, however with a quick check for registers which may need switching
			switch (registerAddress)
			{
			case REG_CUSTOM_SYSTEM_DATE_TIME:
			{
				registerAddressToSend = REG_SYSTEM_INFO_RW_SYSTEM_TIME_YEAR_MONTH;
				break;
			}
			default:
			{
				registerAddressToSend = registerAddress;
				break;
			}
			}

			if (result == modbusRequestAndResponseStatusValues::preProcessing)
			{
				// Generate a frame without CRC (ending 0, 0), sendModbus will do the rest
				uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER,
						    (uint8_t)((registerAddressToSend >> 8) & 0xff), (uint8_t)(registerAddressToSend & 0xff),
						    0, rs->registerCount,
						    0, 0 };

				// And send to the device, it's all synchronos so by the time we get a response we will know if success or failure
				result = _modBus->sendModbus(frame, sizeof(frame), rs);
			}
		}
	}

	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
	{
		// So, it's a success, so we can process according to the rules of the Modbus documentation
		// What we are aiming for as an understanding of a correct value, correctly typed, identifiable by any calling function

		switch (rs->returnDataType)
		{
		case modbusReturnDataType::character:
		{
			memcpy(rs->characterValue, rs->data, rs->dataSize);
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_CHARACTER_DESC);
			break;
		}
		case modbusReturnDataType::unsignedInt:
		{
			rs->unsignedIntValue = (uint32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_UNSIGNED_INT_DESC);
			break;
		}
		case modbusReturnDataType::unsignedShort:
		{
			rs->unsignedShortValue = (uint16_t)(rs->data[0] << 8 | rs->data[1]);
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_UNSIGNED_SHORT_DESC);
			break;
		}
		case modbusReturnDataType::signedInt:
		{
			rs->signedIntValue = (int32_t)(rs->data[0] << 24 | rs->data[1] << 16 | rs->data[2] << 8 | rs->data[3]);
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_SIGNED_INT_DESC);
			break;
		}
		case modbusReturnDataType::signedShort:
		{
			rs->signedShortValue = (int16_t)(rs->data[0] << 8 | rs->data[1]);
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_SIGNED_SHORT_DESC);
			break;
		}
		case modbusReturnDataType::notDefined:
		{
			strcpy(rs->returnDataTypeDesc, MODBUS_RETURN_DATA_TYPE_NOT_DEFINED_DESC);
			result = modbusRequestAndResponseStatusValues::notHandledRegister;
			break;
		}
		}

		switch (registerAddress)
		{
		case REG_GRID_METER_RW_GRID_METER_CT_ENABLE:
		{
			// Type: Unsigned Short
			// 1/bit
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);

			break;
		}
		case REG_GRID_METER_RW_GRID_METER_CT_RATE:
		{
			// Type: Unsigned Short
			// 1/bit
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_GRID_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1:
		{
			// Type: Unsigned Integer
			// 0.01kWh/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * 0.01);
			break;
		}

		case REG_GRID_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1:
		{
			// Type: Unsigned Integer
			// 0.01kWh/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * 0.01);
			break;
		}

		case REG_GRID_METER_R_VOLTAGE_OF_A_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_GRID_METER_R_VOLTAGE_OF_B_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_GRID_METER_R_VOLTAGE_OF_C_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_GRID_METER_R_CURRENT_OF_A_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_GRID_METER_R_CURRENT_OF_B_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_GRID_METER_R_CURRENT_OF_C_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_GRID_METER_R_FREQUENCY:
		{
			// Type: Unsigned Short
			// 0.01Hz
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * FREQUENCY_MULTIPLIER);
			break;
		}
		case REG_GRID_METER_R_ACTIVE_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Minus = feeding, Positive = drawing
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_ACTIVE_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Minus = feeding, Positive = drawing
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_ACTIVE_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Minus = feeding, Positive = drawing
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_TOTAL_ACTIVE_POWER_1:
		{
			// Type: Integer
			// 1W/bit
			// Minus = feeding, Positive = drawing
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_REACTIVE_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_REACTIVE_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_REACTIVE_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_TOTAL_REACTIVE_POWER_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_APPARENT_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_APPARENT_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_APPARENT_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_TOTAL_APPARENT_POWER_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_GRID_METER_R_POWER_FACTOR_OF_A_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_GRID_METER_R_POWER_FACTOR_OF_B_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_GRID_METER_R_POWER_FACTOR_OF_C_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_GRID_METER_R_TOTAL_POWER_FACTOR:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_PV_METER_RW_PV_METER_CT_ENABLE:
		{
			// Type: Unsigned Short
			// 1/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_PV_METER_RW_PV_METER_CT_RATE:
		{
			// Type: Unsigned Short
			// 1/bit
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_PV_METER_R_TOTAL_ENERGY_FEED_TO_GRID_1:
		{
			// Type: Unsigned Integer
			// 0.01kWh/bit
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * 0.01);
			break;
		}

		case REG_PV_METER_R_TOTAL_ENERGY_CONSUMED_FROM_GRID_1:
		{
			// Type: Unsigned Integer
			// 0.01kWh/bit
			// This is total PV generation
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * 0.01);
			break;
		}

		case REG_PV_METER_R_VOLTAGE_OF_A_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_PV_METER_R_VOLTAGE_OF_B_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_PV_METER_R_VOLTAGE_OF_C_PHASE:
		{
			// Type: Unsigned Short
			// 1V
			// Presume * 0.1 as result appears to reflect this.  I.e. my voltage 2421, or 242.1
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * GRID_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_PV_METER_R_CURRENT_OF_A_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_PV_METER_R_CURRENT_OF_B_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_PV_METER_R_CURRENT_OF_C_PHASE:
		{
			// Type: Short
			// 0.1A
			// Always returns 0 for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_PV_METER_R_FREQUENCY:
		{
			// Type: Unsigned Short
			// 0.01Hz
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * FREQUENCY_MULTIPLIER);
			break;
		}
		case REG_PV_METER_R_ACTIVE_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Current generation
			// Positive = PV generating / In theory should never be negative.
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_ACTIVE_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Current generation
			// Positive = PV generating / In theory should never be negative.
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_ACTIVE_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1W/bit
			// Current generation
			// Positive = PV generating / In theory should never be negative.
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_TOTAL_ACTIVE_POWER_1:
		{
			// Type: Integer
			// 1W/bit
			// Current generation
			// Positive = PV generating / In theory should never be negative.
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_REACTIVE_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_REACTIVE_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_REACTIVE_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_TOTAL_REACTIVE_POWER_1:
		{
			// Type: Integer
			// 1var
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_APPARENT_POWER_OF_A_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_APPARENT_POWER_OF_B_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_APPARENT_POWER_OF_C_PHASE_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_TOTAL_APPARENT_POWER_1:
		{
			// Type: Integer
			// 1VA
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_PV_METER_R_POWER_FACTOR_OF_A_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_PV_METER_R_POWER_FACTOR_OF_B_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_PV_METER_R_POWER_FACTOR_OF_C_PHASE:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_PV_METER_R_TOTAL_POWER_FACTOR:
		{
			// Type: Short
			// 0.01
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_BATTERY_HOME_R_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_CURRENT:
		{
			// Type: Short
			// 0.1A/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_STATUS:
		{
			// Type: Unsigned Short
			// <<Note1 - BATTERY STATUS LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case BATTERY_STATUS_CHARGE0_DISCHARGE0:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE0_DISCHARGE0_DESC);
				break;
			}
			case BATTERY_STATUS_CHARGE0_DISCHARGE1:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE0_DISCHARGE1_DESC);
				break;
			}
			case BATTERY_STATUS_CHARGE1_DISCHARGE0:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE1_DISCHARGE0_DESC);
				break;
			}
			case BATTERY_STATUS_CHARGE1_DISCHARGE1:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE1_DISCHARGE1_DESC);
				break;
			}
			case BATTERY_STATUS_CHARGE2_DISCHARGE0:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE2_DISCHARGE0_DESC);
				break;
			}
			case BATTERY_STATUS_CHARGE2_DISCHARGE1:
			{
				strcpy(rs->dataValueFormatted, BATTERY_STATUS_CHARGE2_DISCHARGE1_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_BATTERY_HOME_R_RELAY_STATUS:
		{
			// Type: Unsigned Short
			// <<Note2 - BATTERY RELAY STATUS LU>>
			switch (rs->unsignedShortValue)
			{
			case BATTERY_RELAY_STATUS_CHARGE_AND_DISCHARGE_RELAYS_CLOSED:
			{
				strcpy(rs->dataValueFormatted, BATTERY_RELAY_STATUS_CHARGE_AND_DISCHARGE_RELAYS_CLOSED_DESC);
				break;
			}
			case BATTERY_RELAY_STATUS_CHARGE_DISCHARGE_RELAYS_NOT_CONNECTED:
			{
				strcpy(rs->dataValueFormatted, BATTERY_RELAY_STATUS_CHARGE_DISCHARGE_RELAYS_NOT_CONNECTED_DESC);
				break;
			}
			case BATTERY_RELAY_STATUS_ONLY_CHARGE_RELAY_CLOSED:
			{
				strcpy(rs->dataValueFormatted, BATTERY_RELAY_STATUS_ONLY_CHARGE_RELAY_CLOSED_DESC);
				break;
			}
			case BATTERY_RELAY_STATUS_ONLY_DISCHARGE_RELAY_CLOSED:
			{
				strcpy(rs->dataValueFormatted, BATTERY_RELAY_STATUS_ONLY_DISCHARGE_RELAY_CLOSED_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_MIN_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// My min cell voltage is reading as 334, so * 0.001 = 0.334V.  I consider the document wrong, think it should be 0.01
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * CELL_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_MAX_CELL_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.001V/bit
			// My min cell voltage is reading as 335, so * 0.001 = 0.335V.  I consider the document wrong, think it should be 0.01
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * CELL_VOLTAGE_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_PACK_ID_OF_MIN_CELL_TEMPERATURE:
		{
			// Type: Unsigned Short
			// 0.001D/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_CELL_ID_OF_MIN_CELL_TEMPERATURE:
		{
			// Type: Unsigned Short
			// 0.001D/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_MIN_CELL_TEMPERATURE:
		{
			// Type: Short
			// 0.001D/bit
			// ###HERE###
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * BATTERY_TEMP_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_PACK_ID_OF_MAX_CELL_TEMPERATURE:
		{
			// Type: Unsigned Short
			// 0.001D/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_CELL_ID_OF_MAX_CELL_TEMPERATURE:
		{
			// Type: Unsigned Short
			// 0.001D/bit
			// I believe this is an identifier so shouldn't be subject to multiplication like in the documentation
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_MAX_CELL_TEMPERATURE:
		{
			// Type: Short
			// 0.001D/bit
			// ###HERE###
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * BATTERY_TEMP_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_MAX_CHARGE_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_MAX_DISCHARGE_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// 1000 for me, M8456-P spec is 50A so a bit curious
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_CHARGE_CUT_OFF_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// 576 for me, so 57.6V as per M8456-P spec
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_DISCHARGE_CUT_OFF_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// 480 for me, so 48.0V as per M8456-P spec
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_BMU_SOFTWARE_VERSION:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_LMU_SOFTWARE_VERSION:
		{
			// Type: Unsigned Short
			// //
			// 164 at present
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_ISO_SOFTWARE_VERSION:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_NUMBER:
		{
			// Type: Unsigned Short
			// Battery modules number
			// Yep, I get 4
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_CAPACITY:
		{
			// Type: Unsigned Short
			// 0.1kWh/bit
			// 114 so 11.4, correct for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_KWH_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_TYPE:
		{
			// Type: Unsigned Short
			// <<Note3 - BATTERY TYPE LOOKUP>>
			// 24
			switch (rs->unsignedShortValue)
			{
			case BATTERY_TYPE_M4860:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_M4860_DESC);
				break;
			}
			case BATTERY_TYPE_M48100:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_M48100_DESC);
				break;
			}
			case BATTERY_TYPE_48112_P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_48112_P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE5_BAT:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE5_BAT_DESC);
				break;
			}
			case BATTERY_TYPE_M4856_P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_M4856_P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_10_3P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_10_3P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_10_1P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_10_1P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_5_8P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_5_8P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_5_JP:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_5_JP_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_13_7P:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_13_7P_DESC);
				break;
			}
			case BATTERY_TYPE_SMILE_BAT_8_2_PHA:
			{
				strcpy(rs->dataValueFormatted, BATTERY_TYPE_SMILE_BAT_8_2_PHA_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_SOH:
		{
			// Type: Unsigned Short
			// 0.1/bit
			// Always zero for me, or SOH is actually zero, so I cannot confirm
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_WARNING_1:
		{
			// Type: Unsigned Integer
			// <<Note28 - Battery Warning>>
			if (rs->unsignedIntValue == 0) {
				strcpy(rs->dataValueFormatted, "0");
			} else {
				const char *warning;
				if (rs->unsignedIntValue & 0b00000000000000000000000000000001)
					warning = BATTERY_WARNING_BIT_0;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000000010)
					warning = BATTERY_WARNING_BIT_1;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000000100)
					warning = BATTERY_WARNING_BIT_2;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000001000)
					warning = BATTERY_WARNING_BIT_3;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000010000)
					warning = BATTERY_WARNING_BIT_4;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000100000)
					warning = BATTERY_WARNING_BIT_5;
				else if (rs->unsignedIntValue & 0b00000000000000000000000001000000)
					warning = BATTERY_WARNING_BIT_6;
				else if (rs->unsignedIntValue & 0b00000000000000000000000010000000)
					warning = BATTERY_WARNING_BIT_7;
				else if (rs->unsignedIntValue & 0b00000000000000000000000100000000)
					warning = BATTERY_WARNING_BIT_8;
				else
					warning = "Unknown";
				snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "0x%lX - %s", rs->unsignedIntValue, warning);
			}
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_1:
		{
			// Type: Unsigned Integer
			// <<Note4 - BATTERY ERROR LOOKUP>>
			if (rs->unsignedIntValue == 0) {
				strcpy(rs->dataValueFormatted, "0");
			} else {
				const char *fault;
				if (rs->unsignedIntValue & 0b00000000000000000000000000000001)
					fault = BATTERY_ERROR_BIT_0;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000000010)
					fault = BATTERY_ERROR_BIT_1;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000000100)
					fault = BATTERY_ERROR_BIT_2;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000001000)
					fault = BATTERY_ERROR_BIT_3;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000010000)
					fault = BATTERY_ERROR_BIT_4;
				else if (rs->unsignedIntValue & 0b00000000000000000000000000100000)
					fault = BATTERY_ERROR_BIT_5;
				else if (rs->unsignedIntValue & 0b00000000000000000000000001000000)
					fault = BATTERY_ERROR_BIT_6;
				else if (rs->unsignedIntValue & 0b00000000000000000000000010000000)
					fault = BATTERY_ERROR_BIT_7;
				else if (rs->unsignedIntValue & 0b00000000000000000000000100000000)
					fault = BATTERY_ERROR_BIT_8;
				else if (rs->unsignedIntValue & 0b00000000000000000000001000000000)
					fault = BATTERY_ERROR_BIT_9;
				else if (rs->unsignedIntValue & 0b00000000000000000000010000000000)
					fault = BATTERY_ERROR_BIT_10;
				else if (rs->unsignedIntValue & 0b00000000000000000000100000000000)
					fault = BATTERY_ERROR_BIT_11;
				else if (rs->unsignedIntValue & 0b00000000000000000001000000000000)
					fault = BATTERY_ERROR_BIT_12;
				else if (rs->unsignedIntValue & 0b00000000000000000010000000000000)
					fault = BATTERY_ERROR_BIT_13;
				else if (rs->unsignedIntValue & 0b00000000000000000100000000000000)
					fault = BATTERY_ERROR_BIT_14;
				else if (rs->unsignedIntValue & 0b00000000000000001000000000000000)
					fault = BATTERY_ERROR_BIT_15;
				else if (rs->unsignedIntValue & 0b00000000000000010000000000000000)
					fault = BATTERY_ERROR_BIT_16;
				else if (rs->unsignedIntValue & 0b00000000000000100000000000000000)
					fault = BATTERY_ERROR_BIT_17;
				else if (rs->unsignedIntValue & 0b00000000000001000000000000000000)
					fault = BATTERY_ERROR_BIT_18;
				else if (rs->unsignedIntValue & 0b00000000000010000000000000000000)
					fault = BATTERY_ERROR_BIT_19;
				else if (rs->unsignedIntValue & 0b00000000000100000000000000000000)
					fault = BATTERY_ERROR_BIT_20;
				else if (rs->unsignedIntValue & 0b00000000001000000000000000000000)
					fault = BATTERY_ERROR_BIT_21;
				else if (rs->unsignedIntValue & 0b00000000010000000000000000000000)
					fault = BATTERY_ERROR_BIT_22;
				else if (rs->unsignedIntValue & 0b00000000100000000000000000000000)
					fault = BATTERY_ERROR_BIT_23;
				else if (rs->unsignedIntValue & 0b00000001000000000000000000000000)
					fault = BATTERY_ERROR_BIT_24;
				else if (rs->unsignedIntValue & 0b00000010000000000000000000000000)
					fault = BATTERY_ERROR_BIT_25;
				else if (rs->unsignedIntValue & 0b00000100000000000000000000000000)
					fault = BATTERY_ERROR_BIT_26;
				else if (rs->unsignedIntValue & 0b00001000000000000000000000000000)
					fault = BATTERY_ERROR_BIT_27;
				else if (rs->unsignedIntValue & 0b00010000000000000000000000000000)
					fault = BATTERY_ERROR_BIT_28;
				else if (rs->unsignedIntValue & 0b00100000000000000000000000000000)
					fault = BATTERY_ERROR_BIT_29;
				else if (rs->unsignedIntValue & 0b01000000000000000000000000000000)
					fault = BATTERY_ERROR_BIT_30;
				else if (rs->unsignedIntValue & 0b10000000000000000000000000000000)
					fault = BATTERY_ERROR_BIT_31;
				else
					fault = "Unknown"; // Shouldn't happen.
				snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "0x%lX - %s", rs->unsignedIntValue, fault);
			}
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_CHARGE_ENERGY_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			// Lifetime charge in kWh
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * BATTERY_KWH_MULTIPLIER);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_DISCHARGE_ENERGY_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			// Lifetime discharge in kWh
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * BATTERY_KWH_MULTIPLIER);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_ENERGY_CHARGE_FROM_GRID_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			// Lifetime charge from grid in kWh
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * BATTERY_KWH_MULTIPLIER);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_POWER:
		{
			// Type: Short
			// 1W/bit, - Charge, + Discharge
			// Current battery power
			sprintf(rs->dataValueFormatted, "%d", rs->signedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_REMAINING_TIME:
		{
			// Type: Unsigned Short
			// 1 minute/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_CHARGE_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit (Rate_SOC UPS_SOC)
			// Zero for me, however cannot verify as not sure purpose
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_IMPLEMENTATION_DISCHARGE_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit (Rate_SOC UPS_SOC)
			// Zero for me, however cannot verify as not sure purpose
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_REMAINING_CHARGE_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit (Rate_SOC Remain_SOC)
			// Zero for me, however cannot verify as not sure purpose
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_REMAINING_DISCHARGE_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit (Remain_SOC UPS_SOC)
			// Zero for me, however cannot verify as not sure purpose
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * BATTERY_SOC_MULTIPLIER);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_MAX_CHARGE_POWER:
		{
			// Type: Unsigned Short
			// 1W/bit
			// Zero for me, however cannot verify as not sure purpose
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_MAX_DISCHARGE_POWER:
		{
			// Type: Unsigned Short
			// 1W/bit
			// 5350 for me, however I'm on a B3 and I'd have thought M4856-P batteries were around 3000
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_RW_BATTERY_MOS_CONTROL:
		{
			// Type: Unsigned Short
			// <<BATTERY MOS CONTROL LOOKUP>>
			if (rs->unsignedShortValue == BATTERY_MOS_CONTROL_CLOSE)
				strcpy(rs->dataValueFormatted, BATTERY_MOS_CONTROL_CLOSE_DESC);
			else if (rs->unsignedShortValue == BATTERY_MOS_CONTROL_OPEN)
				strcpy(rs->dataValueFormatted, BATTERY_MOS_CONTROL_OPEN_DESC);

			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_SOC_CALIBRATION:
		{
			// Type: Unsigned Short
			// <<BATTERY SOC CALIBRATION LOOKUP>>
			if (rs->unsignedShortValue == BATTERY_SOC_CALIBRATION_DISABLE)
				strcpy(rs->dataValueFormatted, BATTERY_SOC_CALIBRATION_DISABLE_DESC);
			else if (rs->unsignedShortValue == BATTERY_SOC_CALIBRATION_ENABLE)
				strcpy(rs->dataValueFormatted, BATTERY_SOC_CALIBRATION_ENABLE_DESC);

			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_SINGLE_CUT_ERROR_CODE:
		{
			// Type: Unsigned Short
			// //
			// Zero for me, unable to verify
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_BATTERY_HOME_R_BATTERY_FAULT_1_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_2_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_3_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_4_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_5_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_FAULT_6_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_1_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_2_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_3_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_4_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_5_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_BATTERY_HOME_R_BATTERY_WARNING_6_1:
		{
			// Type: Unsigned Integer
			// //
			// Zero for me at present, unable to verify
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_VOLTAGE_L1:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_VOLTAGE_L2:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_VOLTAGE_L3:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Always zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_CURRENT_L1:
		{
			// Type: Short
			// 0.1A/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_CURRENT_L2:
		{
			// Type: Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_CURRENT_L3:
		{
			// Type: Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_POWER_L1_1:
		{
			// Type: Integer
			// 1W/bit
			// My B3 returns a number I don't recognise.  It's load of house or battery related but doesn't match the web interface
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_POWER_L2_1:
		{
			// Type: Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_POWER_L3_1:
		{
			// Type: Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_POWER_TOTAL_1:
		{
			// Type: Integer
			// 1W/bit
			// My B3 returns 65526.  Or 65.526kW  What the?
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L1:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L2:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_VOLTAGE_L3:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_CURRENT_L1:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// 9 for me, so 0.9A
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_CURRENT_L2:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_CURRENT_L3:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_BACKUP_POWER_L1_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_BACKUP_POWER_L2_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_BACKUP_POWER_L3_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_BACKUP_POWER_TOTAL_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_FREQUENCY:
		{
			// Type: Unsigned Short
			// 0.1Hz/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * FREQUENCY_MULTIPLIER);
			break;
		}
		case REG_INVERTER_HOME_R_PV1_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV1_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV1_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_PV2_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV2_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV2_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_PV3_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV3_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV3_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_PV4_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV4_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV4_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_PV5_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV5_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV5_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_PV6_VOLTAGE:
		{
			// Type: Unsigned Short
			// 0.1V/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV6_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_PV6_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_INVERTER_TEMP:
		{
			// Type: Unsigned Short
			// 0.1D/bit
			// Mine returns 2720, so assuming actually multiplied by 0.01 to bring to something realistic
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * INVERTER_TEMP_MULTIPLIER);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_WARNING_1_1:
		{
			// Type: Unsigned Integer
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_INVERTER_WARNING_2_1:
		{
			// Type: Unsigned Integer
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_INVERTER_FAULT_1_1:
		{
			// Type: Unsigned Integer
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_INVERTER_FAULT_2_1:
		{
			// Type: Unsigned Integer
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_INVERTER_HOME_R_INVERTER_TOTAL_PV_ENERGY_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * 0.1);
			break;
		}

		case REG_INVERTER_HOME_R_WORKING_MODE:
		{
			// Type: Unsigned Short
			// <<Note5 - INVERTER OPERATION LOOKUP>>

			switch (rs->unsignedShortValue)
			{
			case INVERTER_OPERATION_MODE_WAIT_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_WAIT_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_ONLINE_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_ONLINE_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_UPS_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_UPS_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_BYPASS_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_BYPASS_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_ERROR_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_ERROR_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_DC_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_DC_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_SELF_TEST_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_SELF_TEST_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_CHECK_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_CHECK_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_UPDATE_MASTER_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_UPDATE_MASTER_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_UPDATE_SLAVE_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_UPDATE_SLAVE_MODE_DESC);
				break;
			}
			case INVERTER_OPERATION_MODE_UPDATE_ARM_MODE:
			{
				strcpy(rs->dataValueFormatted, INVERTER_OPERATION_MODE_UPDATE_ARM_MODE_DESC);
				break;
			}


			}
			break;
		}
#ifdef EMS_35_36
		case REG_INVERTER_HOME_R_INVERTER_BAT_VOLTAGE:
		{
			// Type: Unsigned Short
			// 1V/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_BAT_CURRENT:
		{
			// Type: Unsigned Short
			// 0.1A/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * 0.1);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_BAT_POWER:
		{
			// Type: Signed Short
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%d", rs->signedShortValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_TOTAL_REACT_POWER_1:
		{
			// Type: Signed Int
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_TOTAL_APPARENT_POWER_1:
		{
			// Type: Signed Int
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_FREQUENCY:
		{
			// Type: Unsigned Short
			// 0.01Hz/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * FREQUENCY_MULTIPLIER);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_BACKUP_FREQUENCY:
		{
			// Type: Unsigned Short
			// 0.01Hz/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * FREQUENCY_MULTIPLIER);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_POWER_FACTOR:
		{
			// Type: Signed Short
			// 0.01/bit
			sprintf(rs->dataValueFormatted, "%0.02f", rs->signedShortValue * 0.01);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_1_1:
		{
			// Type: Unsigned Int
			// Note 27 : XXX TBD XXX
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_2_1:
		{
			// Type: Unsigned Int
			// Note 27 : XXX TBD XXX
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_3_1:
		{
			// Type: Unsigned Int
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_INVERTER_HOME_R_INVERTER_FAULT_EXTEND_4_1:
		{
			// Type: Unsigned Int
			// Reserve
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
#endif // EMS_35_36
		case REG_INVERTER_HOME_R_PV_TOTAL_POWER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_INVERTER_INFO_R_MASTER_SOFTWARE_VERSION_1:
		{
			// Type: Unsigned Char
			// //
			strcpy(rs->dataValueFormatted, rs->characterValue);
			break;
		}




		case REG_INVERTER_INFO_R_SLAVE_SOFTWARE_VERSION_1:
		{
			// Type: Unsigned Char
			// //
			strcpy(rs->dataValueFormatted, rs->characterValue);
			break;
		}




		case REG_INVERTER_INFO_R_SERIAL_NUMBER_1:
		{
			// Type: Unsigned Char
			// //
			strcpy(rs->dataValueFormatted, rs->characterValue);
			break;
		}

		case REG_INVERTER_INFO_R_ARM_SOFTWARE_VERSION_1:
		{
			// Type: Unsigned Char
			// //
			strcpy(rs->dataValueFormatted, rs->characterValue);
			break;
		}


		case REG_SYSTEM_INFO_RW_FEED_INTO_GRID_PERCENT:
		{
			// Type: Unsigned Short
			// 1%/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_R_SYSTEM_FAULT:
		{
			// Type: Unsigned Integer
			// Note 6 : XXX TBD XXX
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_SYSTEM_INFO_RW_SYSTEM_TIME_YEAR_MONTH:
		{
			// Type: Unsigned Short
			// 0xYYMM, 0x1109 = 2017/09
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_RW_SYSTEM_TIME_DAY_HOUR:
		{
			// Type: Unsigned Short
			// 0xDDHH, 0x1109 = 17th Day/9th Hour
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_RW_SYSTEM_TIME_MINUTE_SECOND:
		{
			// Type: Unsigned Short
			// 0xmmss, 0x1109 = 17th Min/9th Sec
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}


		case REG_SYSTEM_INFO_R_EMS_SN_BYTE_1_2:
		{
			// Type: Unsigned Short
			// EMS SN: ASCII 0x414C=='AL'
			strcpy(rs->dataValueFormatted, rs->characterValue);
			// Clear the characterValue as we are customising this one
			rs->characterValue[0] = 0;
			break;
		}
		
		case REG_SYSTEM_INFO_R_EMS_VERSION_HIGH:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_R_EMS_VERSION_MIDDLE:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_R_EMS_VERSION_LOW:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_R_PROTOCOL_VERSION:
		{
			// Type: Unsigned Short
			// //
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_INFO_R_EMS_VERSION_LOW_SUFFIX_1:
		{
			// Type: Unsigned Char
			// //
			strcpy(rs->dataValueFormatted, rs->characterValue);
			break;
		}
		case REG_SYSTEM_CONFIG_RW_MAX_FEED_INTO_GRID_PERCENT:
		{
			// Type: Unsigned Short
			// 1%/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_CONFIG_RW_PV_CAPACITY_STORAGE_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Always returns zero for me
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_SYSTEM_CONFIG_RW_PV_CAPACITY_OF_GRID_INVERTER_1:
		{
			// Type: Unsigned Integer
			// 1W/bit
			// Returns the PV capacity as defined on the web site
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}

		case REG_SYSTEM_CONFIG_RW_SYSTEM_MODE:
		{
			// Type: Unsigned Short
			// <<SYSTEM MODE LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case SYSTEM_MODE_AC:
			{
				strcpy(rs->dataValueFormatted, SYSTEM_MODE_AC_DESC);
				break;
			}
			case SYSTEM_MODE_DC:
			{
				strcpy(rs->dataValueFormatted, SYSTEM_MODE_DC_DESC);
				break;
			}
			case SYSTEM_MODE_HYBRID:
			{
				strcpy(rs->dataValueFormatted, SYSTEM_MODE_HYBRID_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;

		}
		case REG_SYSTEM_CONFIG_RW_METER_CT_SELECT:
		{
			// Type: Unsigned Short
			// <<METER CT SELECT LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case METER_CT_SELECT_GRID_AND_PV_USE_CT:
			{
				strcpy(rs->dataValueFormatted, METER_CT_SELECT_GRID_AND_PV_USE_CT_DESC);
				break;
			}
			case METER_CT_SELECT_GRID_AND_PV_USE_METER:
			{
				strcpy(rs->dataValueFormatted, METER_CT_SELECT_GRID_AND_PV_USE_METER_DESC);
				break;
			}
			case METER_CT_SELECT_GRID_USE_CT_PV_USE_METER:
			{
				strcpy(rs->dataValueFormatted, METER_CT_SELECT_GRID_USE_CT_PV_USE_METER_DESC);
				break;
			}
			case METER_CT_SELECT_GRID_USE_METER_PV_USE_CT:
			{
				strcpy(rs->dataValueFormatted, METER_CT_SELECT_GRID_USE_METER_PV_USE_CT_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_SYSTEM_CONFIG_RW_BATTERY_READY:
		{
			// Type: Unsigned Short
			// <<BATTERY READY LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case BATTERY_READY_OFF:
			{
				strcpy(rs->dataValueFormatted, BATTERY_READY_OFF_DESC);
				break;
			}
			case BATTERY_READY_ON:
			{
				strcpy(rs->dataValueFormatted, BATTERY_READY_ON_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_SYSTEM_CONFIG_RW_IP_METHOD:
		{
			// Type: Unsigned Short
			// <<IP METHOD LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case IP_METHOD_DHCP:
			{
				strcpy(rs->dataValueFormatted, IP_METHOD_DHCP_DESC);
				break;
			}
			case IP_METHOD_STATIC:
			{
				strcpy(rs->dataValueFormatted, IP_METHOD_STATIC_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_SYSTEM_CONFIG_RW_LOCAL_IP_1:
		{
			// Type: Unsigned Integer
			// 0xC0A80101 192.168.1.1
			// This isn't my local IP.... Just seems to return 192.168.1.1 regardless.
			sprintf(rs->dataValueFormatted, "%u.%u.%u.%u", rs->data[0], rs->data[1], rs->data[2], rs->data[3]);
			break;
		}

		case REG_SYSTEM_CONFIG_RW_SUBNET_MASK_1:
		{
			// Type: Unsigned Integer
			// 0xFFFFFF01 255.255.255.0
			sprintf(rs->dataValueFormatted, "%u.%u.%u.%u", rs->data[0], rs->data[1], rs->data[2], rs->data[3]);
			break;
		}

		case REG_SYSTEM_CONFIG_RW_GATEWAY_1:
		{
			// Type: Unsigned Integer
			// 0xC0A80101 192.168.1.1
			sprintf(rs->dataValueFormatted, "%u.%u.%u.%u", rs->data[0], rs->data[1], rs->data[2], rs->data[3]);
			break;
		}

		case REG_SYSTEM_CONFIG_RW_MODBUS_ADDRESS:
		{
			// Type: Unsigned Short
			// default 0x55 (85 base ten)
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_CONFIG_RW_MODBUS_BAUD_RATE:
		{
			// Type: Unsigned Short
			// <<MODBUS BAUD RATE LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case MODBUS_BAUD_RATE_9600:
			{
				strcpy(rs->dataValueFormatted, MODBUS_BAUD_RATE_9600_DESC);
				break;
			}
			case MODBUS_BAUD_RATE_115200:
			{
				strcpy(rs->dataValueFormatted, MODBUS_BAUD_RATE_115200_DESC);
				break;
			}
			case MODBUS_BAUD_RATE_256000:
			{
				strcpy(rs->dataValueFormatted, MODBUS_BAUD_RATE_256000_DESC);
				break;
			}
			case MODBUS_BAUD_RATE_19200:
			{
				strcpy(rs->dataValueFormatted, MODBUS_BAUD_RATE_19200_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_TIMING_RW_TIME_PERIOD_CONTROL_FLAG:
		{
			// Type: Unsigned Short
			// <<TIME PERIOD CONTROL FLAG LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case TIME_PERIOD_CONTROL_FLAG_DISABLE:
			{
				strcpy(rs->dataValueFormatted, TIME_PERIOD_CONTROL_FLAG_DISABLE_DESC);
				break;
			}
			case TIME_PERIOD_CONTROL_FLAG_ENABLE:
			{
				strcpy(rs->dataValueFormatted, TIME_PERIOD_CONTROL_FLAG_ENABLE_DESC);
				break;
			}
			case TIME_PERIOD_CONTROL_FLAG_ENABLE_CHARGE:
			{
				strcpy(rs->dataValueFormatted, TIME_PERIOD_CONTROL_FLAG_ENABLE_CHARGE_DESC);
				break;
			}
			case TIME_PERIOD_CONTROL_FLAG_ENABLE_DISCHARGE:
			{
				strcpy(rs->dataValueFormatted, TIME_PERIOD_CONTROL_FLAG_ENABLE_DISCHARGE_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_TIMING_RW_UPS_RESERVE_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit
			// Corresponds to Discharging Cut off SOC (%) on web interface.  Doesn't appear to need multiplying
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_CHARGE_CUT_SOC:
		{
			// Type: Unsigned Short
			// 0.1/bit
			// Corresponds to Charging Stops at SOC in web interface, doesn't appear to need multiplying
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_START_TIME_1:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_START_TIME_2:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2:
		{
			// Type: Unsigned Short
			// 1H/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
#ifdef EMS_35_36
		case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_1_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_1_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_START_TIME_2_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_DISCHARGE_STOP_TIME_2_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_START_TIME_1_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_1_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_START_TIME_2_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_TIMING_RW_TIME_CHARGE_STOP_TIME_2_MIN:
		{
			// Type: Unsigned Short
			// 1min/bit
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
#endif // EMS_35_36
		case REG_DISPATCH_RW_DISPATCH_START:
		{
			// Type: Unsigned Short
			// <<DISPATCH START LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case DISPATCH_START_START:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_START_START_DESC);
				break;
			}
			case DISPATCH_START_STOP:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_START_STOP_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_DISPATCH_RW_ACTIVE_POWER_1:
		{
			// Type: Integer
			// 1W/bit Offset: 32000 load<32000

			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);

			break;
		}

		case REG_DISPATCH_RW_REACTIVE_POWER_1:
		{
			// Type: Integer
			// 1Var/bit Offset: 32000 load<32000

			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_DISPATCH_RW_DISPATCH_MODE:
		{
			// Type: Unsigned Short
			// <<Note7 - DISPATCH MODE LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_BATTERY_ONLY_CHARGED_VIA_PV_DESC);
				break;
			}
			case DISPATCH_MODE_STATE_OF_CHARGE_CONTROL:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_STATE_OF_CHARGE_CONTROL_DESC);
				break;
			}
			case DISPATCH_MODE_LOAD_FOLLOWING:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_LOAD_FOLLOWING_DESC);
				break;
			}
			case DISPATCH_MODE_MAXIMISE_OUTPUT:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_MAXIMISE_OUTPUT_DESC);
				break;
			}
			case DISPATCH_MODE_NORMAL_MODE:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_NORMAL_MODE_DESC);
				break;
			}
			case DISPATCH_MODE_OPTIMISE_CONSUMPTION:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_OPTIMISE_CONSUMPTION_DESC);
				break;
			}
			case DISPATCH_MODE_MAXIMISE_CONSUMPTION:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_MAXIMISE_CONSUMPTION_DESC);
				break;
			}
			case DISPATCH_MODE_ECO_MODE:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_ECO_MODE_DESC);
				break;
			}
			case DISPATCH_MODE_FCAS_MODE:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_FCAS_MODE_DESC);
				break;
			}
			case DISPATCH_MODE_PV_POWER_SETTING:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_PV_POWER_SETTING_DESC);
				break;
			}
			case DISPATCH_MODE_NO_BATTERY_CHARGE:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_NO_BATTERY_CHARGE_DESC);
				break;
			}
			case DISPATCH_MODE_BURNIN_MODE:
			{
				strcpy(rs->dataValueFormatted, DISPATCH_MODE_BURNIN_MODE_DESC);
				break;
			}
			default:
			{
				strcpy(rs->dataValueFormatted, "Unknown");
				break;
			}
			}
			break;
		}
		case REG_DISPATCH_RW_DISPATCH_SOC:
		{
			// Type: Unsigned Short
			// 0.4%/bit, 95=SOC of 38%
			// Reduce it back to a percent
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedShortValue * DISPATCH_SOC_MULTIPLIER);
			break;
		}
		case REG_DISPATCH_RW_DISPATCH_TIME_1:
		{
			// Type: Unsigned Integer
			// 1S/bit
			sprintf(rs->dataValueFormatted, "%lu", rs->unsignedIntValue);
			break;
		}
		case REG_DISPATCH_RW_DISPATCH_PARA_7:
		{
			// Type: Unsigned Short
			//
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_DISPATCH_RW_DISPATCH_PARA_8:
		{
			// Type: Unsigned Short
			//
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}



		case REG_AUXILIARY_R_EMS_DI0:
		{
			// Type: Unsigned Short
			// EPO, BatteryMOS cut off
			// No documentation as to value
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_AUXILIARY_R_EMS_DI1:
		{
			// Type: Unsigned Short
			// Reserved
			sprintf(rs->dataValueFormatted, "%u", rs->unsignedShortValue);
			break;
		}
		case REG_SYSTEM_OP_R_PV_INVERTER_ENERGY_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			// Zero for me.  Multiplier is *presumably* wrong in documentation, see below
			// ###HERE###
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * TOTAL_ENERGY_MULTIPLIER);
			break;
		}

		case REG_SYSTEM_OP_R_SYSTEM_TOTAL_PV_ENERGY_1:
		{
			// Type: Unsigned Integer
			// 0.1kWh/bit
			// My value was 308695, and according to web interface my total PV is 3086kWh, so multiplier seems wrong
			// ###HERE###
			sprintf(rs->dataValueFormatted, "%0.02f", rs->unsignedIntValue * TOTAL_ENERGY_MULTIPLIER);
			break;
		}

		case REG_SYSTEM_OP_R_SYSTEM_FAULT_1:
		{
			// Type: Unsigned Integer
			// <<Note6 - SYSTEM ERROR LOOKUP>>
			if (_serialNumberPrefix[0] == 'A' && _serialNumberPrefix[1] == 'L') {
				if (rs->unsignedIntValue == 0) {
					strcpy(rs->dataValueFormatted, "0");
				} else {
					const char *error;
					if (rs->unsignedIntValue & 0b00000000000000000000000000000001)
						error = SYSTEM_ERROR_AL_BIT_0;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000000010)
						error = SYSTEM_ERROR_AL_BIT_1;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000000100)
						error = SYSTEM_ERROR_AL_BIT_2;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000001000)
						error = SYSTEM_ERROR_AL_BIT_3;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000010000)
						error = SYSTEM_ERROR_AL_BIT_4;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000100000)
						error = SYSTEM_ERROR_AL_BIT_5;
					else if (rs->unsignedIntValue & 0b00000000000000000000000001000000)
						error = SYSTEM_ERROR_AL_BIT_6;
					else if (rs->unsignedIntValue & 0b00000000000000000000000010000000)
						error = SYSTEM_ERROR_AL_BIT_7;
					else if (rs->unsignedIntValue & 0b00000000000000000000000100000000)
						error = SYSTEM_ERROR_AL_BIT_8;
					else if (rs->unsignedIntValue & 0b00000000000000000000001000000000)
						error = SYSTEM_ERROR_AL_BIT_9;
					else if (rs->unsignedIntValue & 0b00000000000000000000010000000000)
						error = SYSTEM_ERROR_AL_BIT_10;
					else if (rs->unsignedIntValue & 0b00000000000000000000100000000000)
						error = SYSTEM_ERROR_AL_BIT_11;
					else if (rs->unsignedIntValue & 0b00000000000000000001000000000000)
						error = SYSTEM_ERROR_AL_BIT_12;
					else if (rs->unsignedIntValue & 0b00000000000000000010000000000000)
						error = SYSTEM_ERROR_AL_BIT_13;
					else if (rs->unsignedIntValue & 0b00000000000000000100000000000000)
						error = SYSTEM_ERROR_AL_BIT_14;
					else if (rs->unsignedIntValue & 0b00000000000000001000000000000000)
						error = SYSTEM_ERROR_AL_BIT_15;
					else if (rs->unsignedIntValue & 0b00000000000000010000000000000000)
						error = SYSTEM_ERROR_AL_BIT_16;
					else
						error = "Unknown";
					snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "0x%lX - %s", rs->unsignedIntValue, error);
				}
			} else if (_serialNumberPrefix[0] == 'A' && _serialNumberPrefix[1] == 'E') {
				if (rs->unsignedIntValue == 0) {
					strcpy(rs->dataValueFormatted, "0");
				} else {
					const char *error;
					if (rs->unsignedIntValue & 0b00000000000000000000000000000001)
						error = SYSTEM_ERROR_AE_BIT_0;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000000010)
						error = SYSTEM_ERROR_AE_BIT_1;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000000100)
						error = SYSTEM_ERROR_AE_BIT_2;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000001000)
						error = SYSTEM_ERROR_AE_BIT_3;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000010000)
						error = SYSTEM_ERROR_AE_BIT_4;
					else if (rs->unsignedIntValue & 0b00000000000000000000000000100000)
						error = SYSTEM_ERROR_AE_BIT_5;
					else if (rs->unsignedIntValue & 0b00000000000000000000000001000000)
						error = SYSTEM_ERROR_AE_BIT_6;
					else if (rs->unsignedIntValue & 0b00000000000000000000000010000000)
						error = SYSTEM_ERROR_AE_BIT_7;
					else if (rs->unsignedIntValue & 0b00000000000000000000000100000000)
						error = SYSTEM_ERROR_AE_BIT_8;
					else if (rs->unsignedIntValue & 0b00000000000000000000001000000000)
						error = SYSTEM_ERROR_AE_BIT_9;
					else if (rs->unsignedIntValue & 0b00000000000000000000010000000000)
						error = SYSTEM_ERROR_AE_BIT_10;
					else if (rs->unsignedIntValue & 0b00000000000000000000100000000000)
						error = SYSTEM_ERROR_AE_BIT_11;
					else if (rs->unsignedIntValue & 0b00000000000000000001000000000000)
						error = SYSTEM_ERROR_AE_BIT_12;
					else if (rs->unsignedIntValue & 0b00000000000000000010000000000000)
						error = SYSTEM_ERROR_AE_BIT_13;
					else if (rs->unsignedIntValue & 0b00000000000000000100000000000000)
						error = SYSTEM_ERROR_AE_BIT_14;
					else if (rs->unsignedIntValue & 0b00000000000000001000000000000000)
						error = SYSTEM_ERROR_AE_BIT_15;
					else if (rs->unsignedIntValue & 0b00000000000000010000000000000000)
						error = SYSTEM_ERROR_AE_BIT_16;
					else if (rs->unsignedIntValue & 0b00000000000000100000000000000000)
						error = SYSTEM_ERROR_AE_BIT_17;
					else if (rs->unsignedIntValue & 0b00000000000001000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_18;
					else if (rs->unsignedIntValue & 0b00000000000010000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_19;
					else if (rs->unsignedIntValue & 0b00000000000100000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_20;
					else if (rs->unsignedIntValue & 0b00000000001000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_21;
					else if (rs->unsignedIntValue & 0b00000000010000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_22;
					else if (rs->unsignedIntValue & 0b00000000100000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_23;
					else if (rs->unsignedIntValue & 0b00000001000000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_24;
					else if (rs->unsignedIntValue & 0b00000010000000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_25;
					else if (rs->unsignedIntValue & 0b00000100000000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_26;
					else if (rs->unsignedIntValue & 0b00001000000000000000000000000000)
						error = SYSTEM_ERROR_AE_BIT_27;
					else
						error = "Unknown";
					snprintf(rs->dataValueFormatted, sizeof(rs->dataValueFormatted), "0x%lX - %s", rs->unsignedIntValue, error);
				}
			} else {
				strcpy(rs->dataValueFormatted, "Unknown");
			}
			break;
		}

		case REG_SAFETY_TEST_RW_GRID_REGULATION:
		{
			// Type: Unsigned Short
			// <<GRID REGULATION LOOKUP>>
			switch (rs->unsignedShortValue)
			{
			case GRID_REGULATION_AL_0:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_0_DESC);
				break;
			}
			case GRID_REGULATION_AL_1:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_1_DESC);
				break;
			}
			case GRID_REGULATION_AL_2:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_2_DESC);
				break;
			}
			case GRID_REGULATION_AL_3:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_3_DESC);
				break;
			}
			case GRID_REGULATION_AL_4:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_4_DESC);
				break;
			}
			case GRID_REGULATION_AL_5:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_5_DESC);
				break;
			}
			case GRID_REGULATION_AL_6:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_6_DESC);
				break;
			}
			case GRID_REGULATION_AL_7:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_7_DESC);
				break;
			}
			case GRID_REGULATION_AL_8:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_8_DESC);
				break;
			}
			case GRID_REGULATION_AL_9:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_9_DESC);
				break;
			}
			case GRID_REGULATION_AL_10:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_10_DESC);
				break;
			}
			case GRID_REGULATION_AL_11:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_11_DESC);
				break;
			}
			case GRID_REGULATION_AL_12:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_12_DESC);
				break;
			}
			case GRID_REGULATION_AL_13:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_13_DESC);
				break;
			}
			case GRID_REGULATION_AL_14:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_14_DESC);
				break;
			}
			case GRID_REGULATION_AL_15:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_15_DESC);
				break;
			}
			case GRID_REGULATION_AL_16:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_16_DESC);
				break;
			}
			case GRID_REGULATION_AL_17:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_17_DESC);
				break;
			}
			case GRID_REGULATION_AL_18:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_18_DESC);
				break;
			}
			case GRID_REGULATION_AL_19:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_19_DESC);
				break;
			}
			case GRID_REGULATION_AL_20:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_20_DESC);
				break;
			}
			case GRID_REGULATION_AL_21:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_21_DESC);
				break;
			}
			case GRID_REGULATION_AL_22:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_22_DESC);
				break;
			}
			case GRID_REGULATION_AL_23:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_23_DESC);
				break;
			}
			case GRID_REGULATION_AL_24:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_24_DESC);
				break;
			}
			case GRID_REGULATION_AL_25:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_25_DESC);
				break;
			}
			case GRID_REGULATION_AL_26:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_26_DESC);
				break;
			}
			case GRID_REGULATION_AL_27:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_27_DESC);
				break;
			}
			case GRID_REGULATION_AL_28:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_28_DESC);
				break;
			}
			case GRID_REGULATION_AL_29:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_29_DESC);
				break;
			}
			case GRID_REGULATION_AL_30:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_30_DESC);
				break;
			}
			case GRID_REGULATION_AL_31:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_31_DESC);
				break;
			}
			case GRID_REGULATION_AL_32:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_32_DESC);
				break;
			}
			case GRID_REGULATION_AL_33:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_33_DESC);
				break;
			}
			case GRID_REGULATION_AL_34:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_34_DESC);
				break;
			}
			case GRID_REGULATION_AL_35:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_35_DESC);
				break;
			}
			case GRID_REGULATION_AL_36:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_36_DESC);
				break;
			}
			case GRID_REGULATION_AL_37:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_37_DESC);
				break;
			}
			case GRID_REGULATION_AL_38:
			{
				strcpy(rs->dataValueFormatted, GRID_REGULATION_AL_38_DESC);
				break;
			}
			}
			break;
		}

		case REG_CUSTOM_LOAD:
		{
			// Type: Signed Integer
			// 1W/bit
			// Current load of house
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}

		case REG_CUSTOM_SYSTEM_DATE_TIME:
		{
			// Custom date/time returned as text based on the three registers.
			createFormattedDateTime(rs->dataValueFormatted, rs->data[0], rs->data[1], rs->data[2], rs->data[3], rs->data[4], rs->data[5]);

			// Clear the characterValue as we are customising this one
			rs->characterValue[0] = 0;
			break;
		}

		case REG_CUSTOM_GRID_CURRENT_A_PHASE:
		{
			// Type: Signed Short
			// 0.1A
			// Current amps of Phase A
			sprintf(rs->dataValueFormatted, "%d", rs->signedShortValue);
			break;
		}

		case REG_CUSTOM_TOTAL_SOLAR_POWER:
		{
			// Type: Signed Integer
			// 1W/bit
			// Current generation
			// Positive = Load pulling / In theory should never be negative.
			sprintf(rs->dataValueFormatted, "%ld", rs->signedIntValue);
			break;
		}
		}

	}
	return result;
}






/*
createFormattedDateTime

Takes a series of uint8_t variables and converts into UK formatted date/time value (dd/MMM/yyyy HH:mm:ss)
*/
void RegisterHandler::createFormattedDateTime(char *target, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
	char months[12][4] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
	sprintf(target, "%02d/%s/20%d %02d:%02d:%02d", day, months[month - 1], year, hour, minute, second);
}









/*
readRawRegister

Sends a basic Read Data Register request to the Alpha system and returns back for the calling function to do onward procesing
*/
modbusRequestAndResponseStatusValues RegisterHandler::readRawRegister(uint16_t registerAddress, modbusRequestAndResponse* rs)
{
	modbusRequestAndResponseStatusValues result;

	// Generate a frame with CRC placeholders of 0, 0 at the end
	uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_READDATAREGISTER,
			    (uint8_t)((registerAddress >> 8) & 0xff), (uint8_t)(registerAddress & 0xff),
			    0, rs->registerCount,
			    0, 0 };

	// And send to the device, it's all synchronos so by the time we get a response we will know if success or failure
	result = _modBus->sendModbus(frame, sizeof(frame), rs);

	if (result == modbusRequestAndResponseStatusValues::readDataRegisterSuccess)
	{
		// Maybe we will want to do something?
	}

	return result;
}









/*
writeSingleRegister

Sends a basic Write Single Register request to the Alpha system and returns back for the calling function to do onward procesing
*/
modbusRequestAndResponseStatusValues RegisterHandler::writeRawSingleRegister(uint16_t registerAddress, uint16_t value, modbusRequestAndResponse* rs)
{
	modbusRequestAndResponseStatusValues result;

	// Generate a frame with CRC placeholders of 0, 0 at the end
	uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_WRITESINGLEREGISTER,
			    (uint8_t)((registerAddress >> 8) & 0xff), (uint8_t)(registerAddress & 0xff),
			    (uint8_t)((value >> 8) & 0xff), (uint8_t)(value & 0xff),
			    0, 0 };

	// And send to the device, it's all synchronos so by the time we get a response we will know if success or failure
	result = _modBus->sendModbus(frame, sizeof(frame), rs);

	if (result == modbusRequestAndResponseStatusValues::writeSingleRegisterSuccess)
	{
		// Maybe we will want to do something?
	}

	return result;
}






/*
writeDataRegister

Sends a basic Write Data Register request to the Alpha system and returns back for the calling function to do onward procesing
This expects number of registers (1 for 2 byte registers, 2 for 4 byte registers) in the structure before called
*/
modbusRequestAndResponseStatusValues RegisterHandler::writeRawDataRegister(uint16_t registerAddress, uint32_t value, modbusRequestAndResponse* rs)
{
	modbusRequestAndResponseStatusValues result = modbusRequestAndResponseStatusValues::preProcessing;
	if (rs->registerCount == 1)
	{
		uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_WRITEDATAREGISTER,
				    (uint8_t)((registerAddress >> 8) & 0xff), (uint8_t)(registerAddress & 0xff),
				    0, rs->registerCount, 2,
				    (uint8_t)((value >> 8) & 0xff), (uint8_t)(value & 0xff),
				    0, 0 };
		result = _modBus->sendModbus(frame, sizeof(frame), rs);
	}
	else if (rs->registerCount == 2)
	{
		uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_WRITEDATAREGISTER,
				    (uint8_t)((registerAddress >> 8) & 0xff), (uint8_t)(registerAddress & 0xff),
				    0, rs->registerCount, 4,
				    (uint8_t)((value >> 24) & 0xff), (uint8_t)((value >> 16) & 0xff), (uint8_t)((value >> 8) & 0xff), (uint8_t)(value & 0xff),
				    0, 0 };
		result = _modBus->sendModbus(frame, sizeof(frame), rs);
	}
	// And now it has been sent to the device, the response is essentially synchronos so by the time we get a response we will know if success or failure

	if (result == modbusRequestAndResponseStatusValues::writeDataRegisterSuccess)
	{
		// Maybe we will want to do something?
	}

	return result;
}

/*
writeDispatchRegisters

Sends all of the data for the Dispatch Registers as a single event
This expects 3 values: Mode, ActivePower, and TargetSOC
This writes 5 parameters (registers): Start (1), ActivePower (2), ReactivePower (2), Mode (1), SOC (1), Time (2)
*/
modbusRequestAndResponseStatusValues RegisterHandler::writeDispatchRegisters(uint32_t activePower, uint16_t mode, uint16_t socTarget, modbusRequestAndResponse* rs)
{
	modbusRequestAndResponseStatusValues result;
	uint8_t	frame[] = { ALPHA_SLAVE_ID, MODBUS_FN_WRITEDATAREGISTER,
			(uint8_t)((REG_DISPATCH_RW_DISPATCH_START >> 8) & 0xff), (uint8_t)(REG_DISPATCH_RW_DISPATCH_START & 0xff),
			0, 9, 18,										// 9 registers (1+2+2+1+1+2) and 9*2
			(uint8_t)((DISPATCH_START_START >> 8) & 0xff), (uint8_t)(DISPATCH_START_START & 0xff),	// Start/Stop
			(uint8_t)((activePower >> 24) & 0xff), (uint8_t)((activePower >> 16) & 0xff),		// Active Power
			(uint8_t)((activePower >> 8) & 0xff), (uint8_t)(activePower & 0xff),
			0x00, 0x00, 0x00, 0x00,									// Reactive Power (just use zero)
			(uint8_t)((mode >> 8) & 0xff), (uint8_t)(mode & 0xff),					// Dispatch Mode
			(uint8_t)((socTarget >> 8) & 0xff), (uint8_t)(socTarget & 0xff),			// SOC Target
			0x7F, 0xFF, 0xFF, 0xFF,									// Time (essentially forever)
			0, 0 };
	result = _modBus->sendModbus(frame, sizeof(frame), rs);
	// And now it has been sent to the device, the response is essentially synchronos so by the time we get a response we will know if success or failure

	if (result == modbusRequestAndResponseStatusValues::writeDataRegisterSuccess)
	{
		// Maybe we will want to do something?
	}

	return result;
}
