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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	std::cout << std::boolalpha;   
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	qDebug() << __FILE__ ;
	
	// Scene
	scene.setSceneRect(-2500, -2500, 5000, 5000);
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

	grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{0, true, false, nullptr, 0.} );
	
	for(auto &[key, value] : grid)
	{
		auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
		tile->setPos(key.x,key.z);
		value.rect = tile;
	}

	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
	noserobot->setBrush(Qt::magenta);
	
	readFromFile();
	
	timer.start();
	
	return true;
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
 	try
 	{
 		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);
		
		//updateVisitedCells(bState.x, bState.z);
		updateOccupiedCells(bState, ldata);
		std::vector<QVec>v;

        if(t.isNewCoord()){
				target = t.getCoord();
				path = grid.getOptimalPath(QVec::vec3(bState.x,0,bState.z), t.getCoord());
				if(!path.empty()){
					for(auto &p: path){
						greenPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::green), QBrush(Qt::green)));
					}

					currentPoint = path.front();
					path.pop_front();
				}
				t.setActive(false);
		}

		auto relative =  (innerModel->transform("base", QVec::vec3(currentPoint.x(), 0, currentPoint.z()), "world"));
		float angle = atan2(relative.x(), relative.z());
		float mod = relative.norm2();
		
		
		if(path.empty())
		{
			qDebug() << "Arrived to target";
			differentialrobot_proxy->setSpeedBase(0,0); 
		}
		else if(mod < 200)
		{
			qDebug() << "Picking new point";
			currentPoint = path.front();
			path.pop_front();
		}
		else if(abs(angle) > 0.4)//
		{
			differentialrobot_proxy->setSpeedBase(0, angle); 
		} else {
			differentialrobot_proxy->setSpeedBase(400, 0.3 * angle); 
		}

	}
 	catch(const Ice::Exception &e)
	{	std::cout  << e << std::endl; }
	
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
			view.setFixedSize(scrollArea->width(), scrollArea->height());
	draw();
	
}

void SpecificWorker::saveToFile()
{
	grid.saveToFile(fileName);
}

void SpecificWorker::readFromFile()
{
	std::ifstream myfile;
	myfile.open(fileName, std::ifstream::in);
	
	if(!myfile.fail())
	{
		//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
		for( auto &[k,v] : grid)
			delete v.rect;
		grid.clear();
		Grid<TCell>::Key key; TCell value;
		myfile >> key >> value;
		int k=0;
		while(!myfile.eof()) 
		{
			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
			tile->setPos(key.x,key.z);
			value.rect = tile;
			value.id = k++;
			value.cost = 1;
			grid.insert<TCell>(key,value);
			myfile >> key >> value;
		}
		myfile.close();	
		robot->setZValue(1);
		std::cout << grid.size() << " elements read to grid " << fileName << std::endl;
	}
	else
		throw std::runtime_error("Cannot open file");
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	auto *n = innerModel->getNode<InnerModelLaser>("laser");
	for(auto l: ldata)
	{
		auto r = n->laserTo("world", l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
			cell.free = false;
	}
}

void SpecificWorker::updateVisitedCells(int x, int z)
{
	static unsigned int cont = 0;
	auto [valid, cell] = grid.getCell(x, z); 
	if(valid)
	{
		auto &occupied = cell.visited;
		if(occupied)
		{
			occupied = false;
			cont++;
		}
		float percentOccupacy = 100. * cont / grid.size();
	}
}

void SpecificWorker::draw()
{
	for(auto &[key, value] : grid)
	{
// 		if(value.visited == false)
// 			value.rect->setBrush(Qt::lightGray);
		if(value.free == false)
			value.rect->setBrush(Qt::darkRed);
	}
	view.show();
}

/////////////// PATH PLANNING /////7




/////////////////////////////////////////////////////////77
/////////
//////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)
{
	qDebug() << "PRESSED ON: X: " << myPick.x << " Z: " << myPick.z;
  	t.setCoord(myPick.x, myPick.z);
	for(auto gp: greenPath)
		delete gp;
	greenPath.clear();
	path.clear();
}
// simple linear interpolation between two points
void SpecificWorker::lerp(QVec& dest, const QVec& a, const QVec& b, const float t)
{
	dest.setItem(0, a.x() + ( b.x()-a.x()*t ) );
	dest.setItem(1,0);
	dest.setItem(2, a.z() + ( b.z() - a.z() )*t );
}

// evaluate a point on a bezier-curve. t goes from 0 to 1.0
void SpecificWorker::aux(QVec &dest, std::vector<QVec>v, const float t)
{
	int size = v.size();
    int j = 0;
    while(v.size()!=1) {
		
        for (int i = 0+j; i < v.size(); i++) {
            lerp(v[i], v[i], v[i + 1], t);           // point between a and b (green)
        }
		QVec x = v.back();
        v.pop_back();
		v.insert(v.begin(),x);
        size--;
        j++;
		
    }
    dest = v.back();
}
std::list<QVec> SpecificWorker::bezier(std::list<QVec>l, int numPoints)
{
    std::list<QVec> res;
    for (int i=0; i<numPoints; ++i)
    {
		std::vector<QVec> v{ std::begin(l), std::end(l) };
        QVec p = QVec::zeros(3);
        float t = static_cast<float>(i)/999.0;
        aux(p,v,t);
        res.push_back(p);
    }
    return res;
}
