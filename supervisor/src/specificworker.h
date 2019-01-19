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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);
	void newAprilTag(const tagsList &tags);
	int current =0;
	enum class State  { SEARCH, WAIT} ;  
	State estado = State::SEARCH;

public slots:
	void compute();

private:
	InnerModel *innerModel;
	struct Target
	{
		bool active =false;
		mutable QMutex m;
		QVec currentPose = QVec::zeros(3);
		QVec previousPose = QVec::zeros(3);
		int ident;
		InnerModel *model;
		

		void setActive(bool v)
		{
			QMutexLocker ml(&m);
			active=v;
		}
		float ang;
		
		int getID()
		{
			QMutexLocker ml(&m);
			return ident;
		}
		
		void copy(float x, float z, int id)
		{
			QMutexLocker ml(&m); 
			qDebug()<<"Posicion antes"<< x << z<<id;
			currentPose = model->transform("world",QVec::vec3(x,0,z) ,"rgbd");
			qDebug()<<"Posicion despues" <<currentPose.x()<<currentPose.z();
			ident=id;
			
		}
		bool areEquals()
		{
			 if( (currentPose-previousPose).norm2() > 100)
			 {
				 previousPose = currentPose;
				 return true;
			 }
			 return false;
		}
		
		QVec getCurrentPose()
		{
			QMutexLocker ml(&m);
			return currentPose;    
		}
	};
	Target tag;

};

#endif
