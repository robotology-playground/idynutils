#include "drc_shared/yarp_ft_interface.h"
#include <yarp/dev/IAnalogSensor.h>

yarp_ft_interface::yarp_ft_interface(std::string kinematic_chain,
                                     std::string module_prefix_with_no_slash,
                                     std::string robot_name)
{
    
    yarp::os::Property FT_prop;
    FT_prop.put("device", "analogsensorclient");
    FT_prop.put("robotName", robot_name.c_str());
    FT_prop.put("remote", "/"+robot_name+"/"+kinematic_chain+"/analog:o/forceTorque");
    FT_prop.put("local", "/"+robot_name+"/"+module_prefix_with_no_slash+"/"+kinematic_chain+"/analog:i/forceTorque");
    FT_prop.put("rate", 1);
    
    polyDriver_FT.open(FT_prop);
    if(!polyDriver_FT.isValid())
        printf("error opening analogSensor");
    
    polyDriver_FT.view(this->FT_sensor);
    ft_channels = FT_sensor->getChannels();
    
}



yarp::sig::Vector yarp_ft_interface::sense()
{
    #if (FT_ENABLED==TRUE)
    FT_sensor->read(input);
    #endif
    return input;
} 
