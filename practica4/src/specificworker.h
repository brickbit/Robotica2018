/*
 *    Copyright (C)2018 by Roberto García Romero África Malpartida Chaparro
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

	void gotoTarget(const TLaserData &ldata);
	void bug(const TLaserData &ldata, const TBaseState& bState);
	QLine2D linea; 
	bool obstacle(TLaserData ldata);
	bool targetAtSight(TLaserData ldata);
	float obstacleLeft( const TLaserData &tLaser);
	float distanceToLine(const TBaseState &bState);

public slots:
	void compute();

private:

	
	enum class State {IDLE,GOTO,BUG};
	State state = State::IDLE;

	struct Target
	{
		mutable QMutex m;
		QVec coord = QVec::zeros(3);
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

	std::shared_ptr<InnerModel> innerModel;
	int speed = 200;
	float rot = 0.507;
	float distanciaAnterior;
	Target target;
	


};

#endif
