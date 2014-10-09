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

#include <PvSystem.h>
#include <PvDeviceGEV.h>

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " MAC_ADDRESS IP_ADDRESS [NETMASK] [GATEWAY]" << std::endl <<
                     "Default Netmask: 255.255.255.0" << std::endl <<
                     "Default Gateway: 0.0.0.0" << std::endl;
        return 0;
    }

    std::string mac_address = std::string(argv[1]);

    std::string ip_address = std::string(argv[2]);

    std::string netmask;
    if (argc >= 4)
        netmask = std::string(argv[2]);
    else
        netmask = "255.255.255.0";

    std::string gateway;
    if (argc >= 5)
        gateway = std::string(argv[3]);
    else
        gateway = "0.0.0.0";

    PvDeviceGEV::SetIPConfiguration(PvString(mac_address.c_str()),PvString(ip_address.c_str()),PvString(netmask.c_str()),PvString(gateway.c_str()));

    // THIS IS ONLY THE CHECK
    PvSystem system;
    const PvDeviceInfo * device_info;
    PvResult result = system.FindDevice(PvString(mac_address.c_str()),&device_info);
    if(!result.IsOK())
    {
        std::cout << result.GetDescription().GetAscii() << std::endl;
        return -1;
    }

    if(device_info->GetConnectionID().GetAscii() == ip_address)
        std::cout << "PhotonFocus Camera MAC " << mac_address << " [SN: " << device_info->GetSerialNumber().GetAscii() << "] now has the IP " << device_info->GetConnectionID().GetAscii() << std::endl;
    else
        std::cout << "ERROR! The IP of the PhotonFocus Camera MAC " << mac_address << " [SN: " << device_info->GetSerialNumber().GetAscii() << "] has still the IP " << device_info->GetConnectionID().GetAscii() << std::endl;

    return 0;
}
