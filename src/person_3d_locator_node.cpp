/*=========================================================================
Filename : src/person_3d_locator_node.cpp
Project  : Person 3D Locator
Purpose  : ROS node for 3D Person locator
---------------------------------------------------------------------------
---------------------------------------------------------------------------
Version Number : 1.0
Last Updated   : September 22, 2021
Updated By     : Manodhayan K
---------------------------------------------------------------------------
Copyright (c) 2012 Mobiveil. All rights reserved.
---------------------------------------------------------------------------
This file contains trade secrets of Mobiveil. 
No part may be reproduced or transmitted in any form by any means or for 
any purpose without the express written permission of Mobiveil.
---------------------------------------------------------------------------
Revision History
---------------------------------------------------------------------------


---------------------------------------------------------------------------
Known Issues
---------------------------------------------------------------------------

---------------------------------------------------------------------------
To Do List
---------------------------------------------------------------------------
*/

#include "../include/person_3d_locator/person_3d_locator.h"

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main function for person 3D locator node
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    ros::NodeHandle nodeHandler(NODE_NAME);

    Person3DLocator Person3DLocator(nodeHandler);
    Person3DLocator.Init();
    Person3DLocator.Run();
    
    return 0;

}
