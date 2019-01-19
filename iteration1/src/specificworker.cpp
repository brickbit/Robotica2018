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
	rot = 0.785;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}



void SpecificWorker::compute()
{
    const float threshold = 116.6;// 116.74; //millimeters
    //float rot = 0.785;  //rads per second 90 grados
	//std::cout << "--------primer rot = "<<rot << std::endl;


    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
        
        //float angle = atan2(400,ldata.back().dist); //calculate the angle 


    if( ldata.front().dist < threshold )    
    {
        //float angle = atan2(ldata.front().dist,400); //calculate the angle 
        std::cout << "*********"<<std::endl;
        //if( (ldata.front().angle> angle*(-1) && ldata.front().angle<0) )
		if(ldata.front().angle<0)
        {
            std::cout << "TURN LEFT because my distance is : "<<ldata.front().dist<<" , my angle is: "<<ldata.front().angle<< " and my threshold "<<threshold << std::endl;
            differentialrobot_proxy->setSpeedBase(5, rot);//gira a menos velocidad
            usleep(1500000);  //random wait between 1.5s and 0.1sec
        }
 
        else 
        {
            std::cout << "TURN RIGHT because my distance is : "<<ldata.front().dist<<" , my angle is: "<<ldata.front().angle<< " and my threshold "<<threshold << std::endl;
            differentialrobot_proxy->setSpeedBase(5, rot*(-1));//gira a menos velocidad
            usleep(1500000);
			//usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
        }
		
		rot = rot + 0.012;
		if(rot > 2*1.5707)
		{	
			rot = 0.785;
		}	
//        std::cout << "-------- rot = "<<rot << std::endl;

		
    }
    else 
    {
        //Se mueve recto0
        std::cout << "Me muevo recto "<<ldata.front().dist<<"con velocidad "<<speed<<std::endl;
     //   if (ldata.back().dist>3000)
	//	{
    //    	speed = ((ldata.back().dist - 3000)/10 )* 3 + 200;
	//		differentialrobot_proxy->setSpeedBase(speed, 0); 
    //    }
    //    else {
    //       differentialrobot_proxy->setSpeedBase(200, 0); 

    //    }
		if(ldata.front().dist > 124.5){
			
			speed = (ldata.front().dist - threshold + 50)*4;
			differentialrobot_proxy->setSpeedBase(speed, 0);
		}else
		{
			  differentialrobot_proxy->setSpeedBase(200, 0); 
		}

        
    
    }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
	
}



