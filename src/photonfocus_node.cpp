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

#include <ros/ros.h>

#include <iostream>
#include <photonfocuscamera.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"photonfocus_node");
    ros::NodeHandle node_handle("~");

    IRALab::PhotonFocus::Camera left("149.132.178.99");
    return 0;
}
