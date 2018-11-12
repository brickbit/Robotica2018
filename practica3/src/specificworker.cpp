/*
 *    Copyright (C)2018 La Afri Malpartida
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
	float angle, module;
	
    
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 

		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		
		
		if(target.isNewCoord())
		{
			
			auto tw = target.getCoord();
			Rot2D rot(bState.alpha);
			auto y = QVec::vec2(tw[0], tw[2]);
			auto T = QVec::vec2(bState.x, bState.z);
			auto r = rot.invert()*(y-T);


			angle = atan2(r.x(),r.y());
            module = r.norm2();
            
			if(module < 50)
			{
				if(target.isNewCoord()) target.setActive(false);
				differentialrobot_proxy->setSpeedBase(0, 0); 
				return;					
			}	
			
            differentialrobot_proxy->setSpeedBase( 400 * f1(module)*f2(angle,0.9,0.1),angle);
			


		}	
		
    
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

