/* File : pydynutils.i */
%module pydynutils
%include std_string.i
%include std_vector.i
%include std_map.i
%include std_pair.i

%{
/* Note : always include headers following the inheritance order */

#include "idynutils/idynutils.h"
#include "idynutils/yarp_single_chain_interface.h"
#include "idynutils/ControlType.hpp"
#include "idynutils/RobotUtils.h"
%}

/* Note : always include headers following the inheritance order */
%ignore RobotUtils::setImpedance(const yarp::sig::Vector& Kq, const yarp::sig::Vector& Dq);
%ignore RobotUtils::getImpedance(yarp::sig::Vector& Kq, yarp::sig::Vector& Dq);
%ignore RobotUtils::setImpedance(const ImpedanceMap& impedance_map);
%ignore RobotUtils::getImpedance(ImpedanceMap& impedance_map);
%ignore RobotUtils::sense(yarp::sig::Vector& q, yarp::sig::Vector& qdot, yarp::sig::Vector& tau);

%include "iCub/iDynTree/DynTree.h"
%include "idynutils/idynutils.h"
%include "idynutils/ControlType.hpp"
%include "idynutils/yarp_single_chain_interface.h"
%include "idynutils/RobotUtils.h"

namespace std {
  %template(NamesVector) vector<string>;
};

%feature("autodoc",3);
