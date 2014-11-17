// *****************************************************************************
//
//     Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_DEVICE_ENUMS_H__
#define __PV_DEVICE_ENUMS_H__


typedef enum
{
    PvAccessUnknown = 0,
    PvAccessOpen,
    PvAccessControl,
    PvAccessExclusive,
    PvAccessReadOnly,

} PvAccessType;


typedef enum
{
    PvDeviceClassUnknown = 0,
    PvDeviceClassTransmitter,
    PvDeviceClassReceiver,
    PvDeviceClassTransceiver,
    PvDeviceClassPeripheral,

} PvDeviceClass;


typedef enum
{
    PvUSBStatusNotInitialized = -1,
    PvUSBStatusConnected = 0,
    PvUSBStatusFailedEnumeration,
    PvUSBStatusGeneralFailure,
    PvUSBStatusCausedOvercurrent,
    PvUSBStatusNotEnoughPower,
    PvUSBStatusNotEnoughBandwidth,
    PvUSBStatusHubNestedTooDeeply,
    PvUSBStatusInLegacyHub,
    PvUSBStatusEnumerating,
    PvUSBStatusReset,

} PvUSBStatus;


typedef enum
{
    PvInterfaceTypeUSBHostController = 0,
    PvInterfaceTypeNetworkAdapter = 1

} PvInterfaceType;


typedef enum
{
    PvActionAckStatusOK = 0,
    PvActionAckStatusLate = 1,
    PvActionAckStatusOverflow = 2,
    PvActionAckStatusNoRefTime = 3,

} PvActionAckStatusEnum;


typedef enum
{
    PvDeviceInfoTypeGEV = 0,
    PvDeviceInfoTypePleoraProtocol,
    PvDeviceInfoTypeUSB,
    PvDeviceInfoTypeU3V

} PvDeviceInfoType;


typedef enum
{
    PvUSBSpeedUnknown = 0,
    PvUSBSpeedLow,
    PvUSBSpeedFull,
    PvUSBSpeedHigh,
    PvUSBSpeedSuper

} PvUSBSpeed;


typedef enum
{
    PvDeviceTypeUnknown = -1,
    PvDeviceTypeGEV = 0,
    PvDeviceTypeU3V = 1

} PvDeviceType;


#endif // __PV_DEVICE_ENUMS_H__
