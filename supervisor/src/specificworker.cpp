/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    	innerModel= new InnerModel ( "/home/robocomp/robocomp/files/innermodel/simpleworld.xml" );
	tag.model=innerModel;   	
	timer.start ( Period );

	return true;
}

void SpecificWorker::compute()
{
RoboCompGenericBase::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x,0,bState.z,0 ,bState.alpha,0);
	
	switch (estado)
			{	
				case State::SEARCH:	
					
					qDebug()<<"SEARH";
					qDebug()<<"Current en el search"<<current;
					qDebug()<<"tag getId"<<tag.getID();
					
					if(tag.getID() == current)
					{
						gotopoint_proxy->stop();
						gotopoint_proxy->go("base", tag.getCurrentPose().x(), tag.getCurrentPose().z(), 0);
						//differentialrobot_proxy->stopBase();
						//gotopoint_proxy->go("", tag.getCurrentPose().x(), tag.getCurrentPose().z(), 0);
						qDebug()<<"Llega";
						estado = State::WAIT;
						qDebug()<<"De SEARH  --> wait";
					}
					qDebug()<<"Girando";
					gotopoint_proxy ->turn(0.6);
					//differentialrobot_proxy->setSpeedBase(0,0.3);
					break;
				case State::WAIT:
					qDebug()<<"Entrando en wait"<<gotopoint_proxy->atTarget();
					
					if( gotopoint_proxy->atTarget() == true)
					{
						gotopoint_proxy->stop();
						//differentialrobot_proxy->stopBase();
						estado = State::SEARCH;
						qDebug()<<"De WAIT  --> SEARCH";
						current = (current+1)%4;
						qDebug()<<"Current"<<current;

					}
					/*else if( tag.areEquals() && tag.getID() == current)
					{
						
						gotopoint_proxy->go("", tag.getCurrentPose().x(), tag.getCurrentPose().z(), 0);
					}*/
					
					break;
			}
}


void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
//subscribesToCODE

}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	qDebug()<<"LLegando las april tags" << tags.front().tx << tags.front().tz;
	tag.copy(tags.front().tx, tags.front().tz, tags.front().id);

}


