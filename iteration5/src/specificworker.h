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
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <iostream>
#include <fstream>
#include <queue>

class SpecificWorker : public GenericWorker
{
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

	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		// Ice subscription
        void setPick(const Pick &myPick);
	void go(const string &nodo, const float x, const float y, const float alpha);
	void turn(const float speed);
	bool atTarget();
	void stop();
	void newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);
	void newAprilTag(const tagsList &tags);
		

	public slots:
		void compute();
		void saveToFile();
		void readFromFile();

	private:
		std::shared_ptr<InnerModel> innerModel;
		QGraphicsScene scene;
		QGraphicsView view;
		void draw();
		QGraphicsRectItem *robot;
		QGraphicsEllipseItem *noserobot;
		QVec target = QVec::zeros(3);
		std::string fileName = "map.txt";
		const int tilesize = 70;
		std::atomic<bool> targetReady = false;
		std::atomic<bool> planReady = false;
		QVec currentPoint;
		std::list<QVec> path;
		std::vector<QGraphicsEllipseItem *> greenPath;
		
		
		void updateVisitedCells(int x, int z);
		void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
		//void checkTransform(const RoboCompGenericBase::TBaseState &bState);
		
		/// Grid
		struct TCell
		{
			uint id;
			bool free;
			bool visited;
			QGraphicsRectItem* rect;
			float cost = 1;
			
			// method to save the value
			void save(std::ostream &os) const {	os << free << " " << visited; };
			void read(std::istream &is) {	is >> free >> visited ;};
		};
		
		using TDim = Grid<TCell>::Dimensions;
		Grid<TCell> grid;
        Target t;
	mutable QMutex mu;
		

};

#endif
