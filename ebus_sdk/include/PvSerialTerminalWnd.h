// *****************************************************************************
//
//     Copyright (c) 2007, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVSERIALTERMINALWND_H___
#define __PVSERIALTERMINALWND_H___


#include <PvGUILib.h>
#include <PvWnd.h>
#include <PvDeviceSerialPort.h>

#ifdef WIN32
    class DeviceFinderWnd;
#endif // WIN32

class PV_GUI_API PvSerialTerminalWnd : public PvWnd
{
public:

	PvSerialTerminalWnd();
	virtual ~PvSerialTerminalWnd();

    PvResult SetDevice( IPvDeviceAdapter *aDevice );

    PvResult SetSerialPort( PvDeviceSerial aPort );
    PvDeviceSerial GetSerialPort() const;

protected:

private:

    // Not implemented
	PvSerialTerminalWnd( const PvSerialTerminalWnd & );
	const PvSerialTerminalWnd &operator=( const PvSerialTerminalWnd & );

};


#endif // __PVSERIALTERMINALWND_H___



