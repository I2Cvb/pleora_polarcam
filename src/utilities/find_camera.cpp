/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:	Alberto Invernizzi                       		           *
 *   Email:     alby.inve@gmail.com                                        *
 *   Date:      08/10/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#include <iostream>
#include <vector>

#include <PvSystem.h>

int main(int argc, char **argv)
{
    PvSystem system;
    PvResult result = system.Find();

    if(!result.IsOK())
    {
        std::cout << result.GetDescription().GetAscii() << std::endl;
        return -1;
    }

    std::vector<const PvNetworkAdapter *> network_ifs;

    for(int i=0;i<system.GetInterfaceCount();i++)
    {
        if(system.GetInterface(i)->GetType() == PvInterfaceTypeNetworkAdapter)
            network_ifs.push_back(static_cast<const PvNetworkAdapter *>(system.GetInterface(i)));
    }

    for(int i=0;i<network_ifs.size();i++){
        const PvNetworkAdapter & net_if = *(network_ifs[i]);
        std::cout << net_if.GetDisplayID().GetAscii() << " has " << net_if.GetDeviceCount() << " interfaces" << std::endl;
        for(int k=0;k<net_if.GetDeviceCount();k++)
        {
            const PvDeviceInfo & device_info = *(net_if.GetDeviceInfo(k));
            std::cout << "\t" << k << ": " <<
                         device_info.GetManufacturerInfo().GetAscii() << " " <<
                         device_info.GetModelName().GetAscii() << " " <<
                         "MAC:" << device_info.GetUniqueID().GetAscii() << " " <<
                         "IP:" << device_info.GetConnectionID().GetAscii() << " " <<
                         "SN:" << device_info.GetSerialNumber().GetAscii() << std::endl;
        }
    }

    return 0;
}
