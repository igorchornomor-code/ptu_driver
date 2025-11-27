/*****************************************************************************
*****              (C)2014 FLIR Commercial Systems, Inc.                 *****
*****                       All Rights Reserved.                         *****
*****                                                                    *****
*****     Source data and code may NOT be distributed without the        *****
*****     prior written consent from FLIR Commercial Systems, Inc.       *****
*****                                                                    *****
*****     FLIR Commercial Systems, Inc. reserves the right to make       *****
*****     changes without further notice to any content herein to        *****
*****     improve reliability, function or design. FLIR Commercial       *****
*****     Systems, Inc. shall not assume any liability arising from      *****
*****     the application or use of this code, data or function.         *****
*****                                                                    *****
*****     FLIR Commercial Systems, Inc.                                  *****
*****     Motion Control Systems                                         *****
*****     www.flir.com/mcs                                               *****
*****     mcs-support@flir.com                                           *****
*****************************************************************************/
/* \file libcpi.c
 *
 * DLL exporting wrapper for the CPI.
 */

#include <windows.h>

/* blank DllMain */
BOOL WINAPI DllMain(HINSTANCE hInstDll, DWORD reason, LPVOID lpvRes){
    switch (reason) {
    case DLL_PROCESS_ATTACH:
        break;
    case DLL_THREAD_ATTACH:
        break;
    case DLL_THREAD_DETACH:
        break;
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

