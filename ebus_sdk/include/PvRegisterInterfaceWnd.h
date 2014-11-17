// *****************************************************************************
//
//     Copyright (c) 2007, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PV_REGISTER_INTERFACE_WND_H___
#define __PV_REGISTER_INTERFACE_WND_H___


#include <PvGUILib.h>
#include <PvWnd.h>
#include <PvDeviceGEV.h>


class PV_GUI_API PvRegisterInterfaceWnd : public PvWnd
{
public:

	PvRegisterInterfaceWnd();
	virtual ~PvRegisterInterfaceWnd();

    PvResult SetDevice( PvDeviceGEV *aDevice );

protected:

private:

    // Not implemented
	PvRegisterInterfaceWnd( const PvRegisterInterfaceWnd & );
	const PvRegisterInterfaceWnd &operator=( const PvRegisterInterfaceWnd & );

};


#endif // __PV_REGISTER_INTERFACE_WND_H___



