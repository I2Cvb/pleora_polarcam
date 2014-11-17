// *****************************************************************************
//
//     Copyright (c) 2007, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

#ifndef __PVGUILIB_H__
#define __PVGUILIB_H__


#ifdef PV_GUI_EXPORTS

    // Define the export symbol
      #if !defined( PT_LIB_STATIC ) && defined( WIN32 )
            #define PV_GUI_API __declspec( dllexport )
      #else
            #define PV_GUI_API
      #endif // PT_LIB_STATIC

#else // PV_GUI_EXPORTS

    // Define the import symbol

      #if !defined( PT_LIB_STATIC ) && defined( WIN32 )
            #define PV_GUI_API __declspec( dllimport )
      #else
            #define PV_GUI_API
      #endif // PT_LIB_STATIC

    // Define the suffix used for the static version of the library file
    #ifdef PT_LIB_STATIC
        #define PT_SUFFIX_STATIC "s"
    #else // PT_LIB_STATIC
        #define PT_SUFFIX_STATIC
    #endif // PT_LIB_STATIC

    // Define the suffix used for the debug version of the library file
    #if defined( _PT_DEBUG_ ) && defined( PT_DEBUG_ENABLED )
        #define PT_SUFFIX_DEBUG "Dbg"
    #else // defined( _PT_DEBUG_ ) && defined( PT_DEBUG_ENABLED )
        #define PT_SUFFIX_DEBUG
    #endif // defined( _PT_DEBUG_ ) && defined( PT_DEBUG_ENABLED )

    // Define the suffix used for the 64-bit version of the library file
    #ifdef _PT_64_
        #define PT_SUFFIX_64 "64"
    #else // _PT_64_
        #define PT_SUFFIX_64
    #endif // _PT_64_

    // With debug and/or static, there is an hypen after the library name
    #if defined( PT_LIB_STATIC ) || ( defined( _PT_DEBUG_ ) && defined( PT_DEBUG_ENABLED ) )
        #define PT_SUFFIX_HYPHEN "_"
    #else
        #define PT_SUFFIX_HYPHEN
    #endif

	// Ask the compiler to link the required version of the library
    #pragma comment( lib, "PvGUI" PT_SUFFIX_64 PT_SUFFIX_HYPHEN PT_SUFFIX_STATIC PT_SUFFIX_DEBUG ".lib" )

	// Undefined the symbols used to link the required library file
    #undef PT_SUFFIX_STATIC
    #undef PT_SUFFIX_DEBUG
    #undef PT_SUFFIX_64
    #undef PT_SUFFIX_HYPHEN

#endif // PV_GUI_EXPORTS


#include <PvTypes.h>
#include <PvString.h>
#include <PvResult.h>



#endif // __PVGUILIB_H__


