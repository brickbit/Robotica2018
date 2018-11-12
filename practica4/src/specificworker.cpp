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
		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		innerModel->updateTransformValues("base", bState.x, 0, bState.z ,0, bState.alpha, 0);

		switch( state )
		{

			case State::IDLE:
				if ( target.isNewCoord() )
					state = State::GOTO;
				break;

			case State::GOTO:
				cout <<"estado go to" <<endl;
				gotoTarget(ldata);
				break;

			case State::BUG:
				cout <<"estado bug" <<endl;
				bug(ldata, bState);
				
				break;

		}
	}
	catch( const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}	
	
}

void SpecificWorker::setPick(const Pick &myPick)
{
	std::cout << "hola caracola" << std::endl;
	qDebug() << "PRESSED ON: X: " << myPick.x << " Z: " << myPick.z;
	target.setCoord(myPick.x,myPick.z);
	
	
}

void SpecificWorker::gotoTarget(const TLaserData &ldata)
{
	 if( obstacle(ldata) == true)   // If ther is an obstacle ahead, then transit to BUG
	 {
		 state = State::BUG;
		return;

	 }
	 
	 auto rt = innerModel->transform("base", QVec::vec3(target.getCoord()[0], 0, target.getCoord()[2]) , "world");
     const float MAXADV = 800;
	 float dist = rt.norm2();
     float ang  = atan2(rt.x(), rt.z());

   	 if(dist < 100)          // If close to obstacle stop and transit to IDLE
	 {
		 state = State::IDLE;
	     target.setActive(false);//
		 differentialrobot_proxy->setSpeedBase(0,0);
		 return;
	 }

		 float adv = dist;

		 if ( fabs( ang) > 0.05 )
		 	 adv = 0;
		 
		 try{
	     	differentialrobot_proxy->setSpeedBase( MAXADV * f1(adv)*f2(ang,0.9,0.1),ang);
		 }catch ( const Ice::Exception &ex ) { std::cout << ex << std::endl; }


}

void SpecificWorker::bug(const TLaserData &ldata, const TBaseState& bState)
{
/*	if(targetAtSight(ldata)){
		state = State::GOTO;
		return;
	}	
	if(obstacle() == true){
		try{
			differentialrobot_proxy->setSpeedBase(0,0.3);
		}catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }	

	}else{
		differentialrobot_proxy->setSpeedBase(200,0);
		
	}
*/	
	if(obstacle(ldata) == false)
	{	
		const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
    	//float distanciaObstaculo = obstacleLeft(ldata);
		float diffToline = distanceToLine(bState);
		
		if(targetAtSight(ldata))
		{
			state = State::GOTO;
			return;
		}
		
		if (distanciaAnterior < 100 and diffToline < 0)
		{
      		state = State::GOTO;
     		//qDebug() << "Cruzando la linea : de BUG a GOTO";
      		return;
		}
		
		differentialrobot_proxy->setSpeedBase(200,0);
	}else{
    
	      try{
				differentialrobot_proxy->setSpeedBase(0, 0.3);
      	  }catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
	}	
}	
	
	  



bool SpecificWorker::obstacle(TLaserData ldata)
{
		const float threshold = 250;
		std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
    	return (ldata.front().dist < threshold);
		

}

bool SpecificWorker::targetAtSight(TLaserData ldata)
{
		QPolygonF polygon;
		for (auto l: ldata)
		{
   			auto lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
   			polygon << QPointF(lr.x(), lr.z());
		}
		auto t = target.getCoord();
		return  polygon.containsPoint(QPointF(target.coord[0], target.coord[2]), Qt::WindingFill);
}

float SpecificWorker::obstacleLeft(const TLaserData &tLaser){
	
		const int laserpos = 85;
  		float min = tLaser[laserpos].dist;
  		for(int i=laserpos-2; i<laserpos+2;i++)
    	{
      		if (tLaser[i].dist < min)
			min = tLaser[i].dist;
    	}
    	return min;
}

float SpecificWorker::distanceToLine(const TBaseState &bState){
  
  		QVec posicion = QVec::vec3(bState.x, 0., bState.z);
  		float distanciaActual = fabs(linea.perpendicularDistanceToPoint(posicion));
  		float diferencia = distanciaActual - distanciaAnterior;
 		distanciaAnterior = distanciaActual;
  
 		return diferencia;
  
}

