/*
 *    Copyright (C)2018 Roberto García Romero África Malpartida Chaparro
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

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
       

	try

	{	
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value); 
	}
	catch(std::exception e) { qFatal("Error reading config params"); }







	timer.start(Period);


	return true;
}

float SpecificWorker::f1(float d)
{
  	return (1/(1+exp(-d)-0.5));
}
float SpecificWorker::f2(float r,float h, float Vx)
{
	
	float y;
  
 	y=(-pow(Vx,2))/log(h);
  	return exp((-pow(r,2))/y);
  
}


void SpecificWorker::compute()
{
	
	
    
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 

		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });  //sort laser data from small to large distances using a lambda function.
        if ( target.isNewCoord() && initialized==false) 
        {
            startPoint = QVec::vec3(bState.x, 0, bState.z);
            path = QLine2D( startPoint, target.getCoord() );
            state=StateRobot::GOTO;
            initialized=true;
        }
        
        switch( state )
        {
            case StateRobot::GOTO:
                letsmove(ldata, bState);
                break;
            case StateRobot::STARTBUG:
                startbug(ldata, bState);
                break;
            case StateRobot::BUG:
                bug(ldata, bState);
                break;
            case StateRobot::ENDBUG:
                endbug(bState);
                break;
        }
		/*
			*/
		
    
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
	
}

void SpecificWorker::setPick(const Pick &myPick)
{
	qDebug() << "PRESSED ON: X: " << myPick.x << " Z: " << myPick.z;
	target.setCoord(myPick.x,myPick.z);

	
}
void SpecificWorker::letsmove( const TLaserData &ldata, const TBaseState& bState )
{
    float angle;
    if(target.isNewCoord())
    {
        //para ver si hay obstaculo
        if( ldata.front().dist < threshold )    
        {
            std::cout<<"sale hacia STARTBUG"<<std::endl;
            state = StateRobot::STARTBUG;
            return;
        }
        auto tw = target.getCoord();
        Rot2D rot(bState.alpha);
        auto y = QVec::vec2(tw[0], tw[2]);
        auto T = QVec::vec2(bState.x, bState.z);
        auto r = rot.invert()*(y-T);

        angle = atan2(r.x(),r.y());
        module = r.norm2();
        //para parar
        if(inTarget()==true) return;

        
        differentialrobot_proxy->setSpeedBase( 400 * f1(module)*f2(angle,0.9,0.1),angle);
    }
    std::cout<<" ESTADO GOTO "<<std::endl;
}
void SpecificWorker::startbug(const TLaserData &ldata, const TBaseState& bState)
{
        QVec initialPosition = QVec::vec3(bState.x, 0., bState.z);
        initialDistance = fabs(path.perpendicularDistanceToPoint(initialPosition));
        if(ldata.front().dist > threshold )    
        {
            state = StateRobot::BUG;
            differentialrobot_proxy->setSpeedBase(0, 0);
            return;
        }
        differentialrobot_proxy->setSpeedBase(0, 0.5);

    std::cout<<" ESTADO STARTBUG "<<std::endl;
}
void SpecificWorker::bug(const TLaserData &ldata, const TBaseState& bState)
{
    std::cout<<" ESTADO BUG "<<std::endl;
    const float alpha = log ( 0.1 ) /log ( 0.3 );
    //Calcular la distancia al objetivo
	const int laserpos = 85;
	float min = ldata[laserpos].dist;
	for(int i=laserpos-2; i<laserpos+2;i++)
	{
		if (ldata[i].dist < min)
			min = ldata[i].dist;
	}
    float dist =min;
    float distToLine = distanceToTarget(bState);
    
    if(inTarget()==true) {
	state = StateRobot::ENDBUG;	
	return;
	}
    
    //para ver si hay obstaculo
    if( ldata.front().dist < threshold )    
    {
        std::cout<<"sale hacia STARTBUG"<<std::endl;
        state = StateRobot::STARTBUG;
        return;
    }
    
    	else if (initialDistance < 100 and distToLine < 0)
	{
		state = StateRobot::GOTO;
		qDebug() << "Crossing the line: from BUG to GOTO";
		return;
	}
    float k=0.1;  // pendiente de la sigmoide
	float angle =  -((1./(1. + exp(-k*(dist - 450.))))-1./2.);		
	float v = 350 * exp ( - ( fabs ( angle ) * alpha ) ); 		
	qDebug() << angle << v;
	differentialrobot_proxy->setSpeedBase ( v ,angle );
    
}
void SpecificWorker::endbug(const TBaseState& bState)
{
    if ( target.isNewCoord()) 
    {
        startPoint = QVec::vec3(bState.x, 0, bState.z);
        path = QLine2D( startPoint, target.getCoord() );
        state=StateRobot::GOTO;
    }
    std::cout<<" ESTADO ENDBUG "<<std::endl;
    //letsmove(ldata);
}
bool SpecificWorker::inTarget()
{
    if(module < 50)
    {
        if(target.isNewCoord()) target.setActive(false);
        differentialrobot_proxy->setSpeedBase(0, 0); 
        std::cout<<"ESTOY EN EL TARGET "<<std::endl;
	state = StateRobot::ENDBUG;
        initialized=true;
        return true;					
    }	
    return false;
}
float SpecificWorker::distanceToTarget(const TBaseState& bState)
{
    QVec initialPosition = QVec::vec3(bState.x, 0., bState.z);
	float distanceOfPath = fabs(path.perpendicularDistanceToPoint(initialPosition));
	float distance = distanceOfPath - initialDistance;
	initialDistance = distanceOfPath;
	return distance;
}

