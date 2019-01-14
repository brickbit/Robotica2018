/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
#include <cmath>

//#include <~/robocomp/libs/qmat/include/qmat/qvec.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	float f1(float d);
	float f2(float r,float h, float Vx);
    void letsmove(const TLaserData &ldata, const TBaseState& bState);
    void startbug(const TLaserData &ldata, const TBaseState& bState);
    void bug(const TLaserData &ldata, const TBaseState& bState);
    void endbug(const TBaseState& bState);
    bool inTarget();
    float distanceToTarget(const TBaseState& bState);



public slots:
	void compute();

private:
	struct Target
	{
		mutable QMutex m;
		QVec coord = QVec::zeros(3);
		//float angl;
		bool newCoord = false;
	void setActive(bool newActive)
	{
		QMutexLocker lm(&m);
		newCoord = newActive;
	}
	void setCoord(float x, float z)
	{
		QMutexLocker lm(&m);
		coord.setItem(0,x);
		coord.setItem(1,0);
		coord.setItem(2,z);
		newCoord = true;
	}
	QVec getCoord()
	{
		QMutexLocker lm(&m);
		return coord;
	}
	bool isNewCoord()
	{
		QMutexLocker lm(&m);
		return (newCoord);
	}
	};
	enum class StateRobot
	{
        GOTO,BUG,STARTBUG,ENDBUG
    };
    StateRobot state;
    QVec startPoint;
    QLine2D path;
	bool initialized = false;
    float threshold = 220;
    float initialDistance;
    float module;
	std::shared_ptr<InnerModel> innerModel;
	int speed = 200;
	Target target;


};

#endif
