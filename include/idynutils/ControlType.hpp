/*
 * Copyright (C) 2014 Walkman
 * Author: Luca Muratore, Alessio Rocchi
 * email:  luca.muratore@iit.it, alessio.rocchi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __CONTROLTYPE_HPP__
#define __CONTROLTYPE_HPP__

#include <yarp/dev/IInteractionMode.h>

namespace walkman {
    /**
     * @brief The ControlType class represents the type of control running on the robot.
     * All possible control types are listed in the walkman::controlTypes namespace
     *
     * The way ControlType is implemented at the moment is through a std::pair
     * This wraps the new modes so that only the five control types are available:
     * - position (for setpoints, automatically generates trajectories between setpoints)
     * - position direct (no trajectory interpolation between position references: used when sending trajectories)
     * - torque
     * - impedance control
     * - idle
     */
    class ControlType {
    private:
	/**
	 * @brief isCompatibleWithInteractionMode checks if the current control mode is compatible with the interaction mode; when the function returns 
	 * false the interaction mode should be set to VOCAB_IM_UNKNOWN.
	 * 
	 * @return true if the current control mode is compatible with the interaction mode.
	 */
	bool isCompatibleWithInteractionMode() const;
    protected:
        std::pair<int, yarp::dev::InteractionModeEnum> _controlType;
    public:

        /**
         * @brief toString returns a string representing the control type
         * @return the control type name
         */
        std::string toString() const;

        /**
         * @brief ControlType build a ControlType object. By default it is set to walkman::controlTypes::none
         * meaning by default setting a freshly created ControlType will not change the current robot control type.
         */
        ControlType();

        /**
         * @brief fromYarp tries to create a ControlType object from a yarp ControlMode, InteractionMode pair
         * @return a ControlType if the ControlMode/InteractionMode pair is supported. Otherwise throws an exception
         */
        static ControlType fromYarp(const int& controlMode,
                                    const yarp::dev::InteractionModeEnum& interactionMode) throw();

        /**
         * @brief toYarp returns the yarp control mode and yarp interaction mode
         * corresponding to this control type
         * @return a pair<yarp controlMode, yarp interactionMode>
         */
        std::pair<const int, const yarp::dev::InteractionModeEnum> toYarp() const;

        bool operator==(const ControlType& controlType) const;

        bool operator!=(const ControlType& controlType) const;
	
    };

    /*********************************************************************************
     *                        AVAILABLE CONTROL TYPES                                *
     *********************************************************************************/
    namespace controlTypes {
        /**
         * @brief impedance this ControlType specified impedance control for the robot.
         * While in this control type, it is possible to set a stiffness and damping value for the
         * low level impedance controller. It is then possible to impose a torque offset, which can be
         * used for example to implement gravity compensation. Of course, it is possible to command a desired
         * trajectory for the joint positions.
         */
        const ControlType impedance = ControlType::fromYarp(VOCAB_CM_POSITION_DIRECT, yarp::dev::VOCAB_IM_COMPLIANT);

        /**
         * @brief position this ControlType allows to send setpoints to the robot. The control will take care
         * of creating a trajectory using a trapezoidal velocity profile to reach the setpoint. You can specify
         * the reference speed used by the interpolation.
         */
        const ControlType position = ControlType::fromYarp(VOCAB_CM_POSITION, yarp::dev::VOCAB_IM_STIFF);

        /**
         * @brief positionDirect this ControlType allows to send trajectories for the robot joint positions.
         */
        const ControlType positionDirect = ControlType::fromYarp(VOCAB_CM_POSITION_DIRECT, yarp::dev::VOCAB_IM_STIFF);

        /**
         * @brief torque this ControlType sets the robot in torque mode. It is equivalent to an impedance mode
         * where we specify Kp, Kd equal to 0
         */
        const ControlType torque = ControlType::fromYarp(VOCAB_CM_TORQUE, yarp::dev::VOCAB_IM_UNKNOWN);

        /**
         * @brief idle this controlType implies we set our robot to a non controlled status. By setting this
         * the robot will collapse. It is needed (when the robot is hanging from a a crane) to switch from
         * compliant control modes (impedance) to stiff control modes (position, positionDirect), since a transition between
         * these control types is not considered to be safe.
         */
        const ControlType idle = ControlType::fromYarp(VOCAB_CM_IDLE, yarp::dev::VOCAB_IM_UNKNOWN);

        /**
         * @brief none this ControlType implies we do not want to change the current robot control type
         */
        const ControlType none = ControlType::fromYarp(VOCAB_CM_UNKNOWN, yarp::dev::VOCAB_IM_UNKNOWN);
    }

    /*********************************************************************************/


    inline ControlType::ControlType()  {
        _controlType = std::make_pair(VOCAB_CM_UNKNOWN, yarp::dev::VOCAB_IM_UNKNOWN);
    }

    inline std::string ControlType::toString() const {
        if(*this == walkman::controlTypes::idle) return "Idle";
        if(*this == walkman::controlTypes::impedance) return "Impedance";
        if(*this == walkman::controlTypes::position) return "position";
        if(*this == walkman::controlTypes::positionDirect) return "position direct";
        if(*this == walkman::controlTypes::torque) return "torque";
        if(*this == walkman::controlTypes::none) return "none";
        return "Unrecognized control type";
    }

    inline ControlType ControlType::fromYarp(const int& controlMode,
                                const yarp::dev::InteractionModeEnum& interactionMode) throw() {
        using namespace yarp::dev;
        if(controlMode == VOCAB_CM_POSITION && interactionMode == VOCAB_IM_COMPLIANT)
            throw("unknown control type impedance + position. Maybe try impedance + positionDirect");
        else if(    controlMode == VOCAB_CM_TORQUE  ||
                    controlMode == VOCAB_CM_IDLE    ||
                    (controlMode == VOCAB_CM_POSITION_DIRECT && interactionMode == VOCAB_IM_COMPLIANT) ||
                    (controlMode == VOCAB_CM_POSITION && interactionMode == VOCAB_IM_STIFF)            ||
                    (controlMode == VOCAB_CM_POSITION_DIRECT && interactionMode == VOCAB_IM_STIFF)     ||
                    ((controlMode == VOCAB_CM_UNKNOWN || controlMode == -1) &&
                     (interactionMode == VOCAB_IM_UNKNOWN || interactionMode == -1))) {
            ControlType ct;
            ct._controlType = std::make_pair(controlMode, interactionMode);
            return ct;
        }
        else throw("unknown control type");
    }

    inline std::pair<const int, const yarp::dev::InteractionModeEnum>  ControlType::toYarp() const {
        return _controlType;
    }

    inline bool ControlType::operator!=(const ControlType& controlType) const {
        return !(this->operator ==(controlType));
    }

    inline bool ControlType::operator==(const ControlType& controlType) const {
        return 	( ( controlType._controlType.first == this->_controlType.first ) && 
		 ( !this->isCompatibleWithInteractionMode() || 
		   ( controlType._controlType.second == this->_controlType.second ) ) ) ;
    }

    inline std::ostream& operator<<(std::ostream& os, const ControlType& controlType) {
        os << controlType.toString();
        return os;
    }
    
    inline bool ControlType::isCompatibleWithInteractionMode() const
    {
	return ( this->_controlType.first == VOCAB_CM_POSITION || 
		 this->_controlType.first == VOCAB_CM_POSITION_DIRECT );
    }
}
#endif // __CONTROLTYPE_HPP__
