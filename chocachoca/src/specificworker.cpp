/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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

const float SAFETY_THRESHOLD=400.f;

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    rot=0.8f;

    //initializeCODE
    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 
}

auto SpecificWorker::cono(const RoboCompLidar3D::TPoints &points, float minAngle, float maxAngle, float distance)
{
    return points | std::views::filter([=, this] (const auto &point)
    {
        const auto angle = std::atan2(std::sin(point.phi), std::cos(point.phi));
        const bool inAngle = minAngle <= maxAngle
            ? angle > minAngle and angle < maxAngle
            : angle > minAngle or angle < maxAngle;
        if (not inAngle) return false;

        if (point.distance2d < this->ROBOT_LENGTH / 2.1) 
            return false;

        if (distance > 0)
            return point.distance2d < distance;

        return true;
    });
}

void SpecificWorker::compute()
{
    fps.print("Compute worker", 3000);
    RoboCompLidar3D::TData data;
    try
    { 
        data = lidar3d_proxy->getLidarData("lidar", 0, 2 * M_PI, 3);
        draw_lidar(data.points, &viewer->scene);
    } 
    catch(const Ice::Exception &e)
    { 
        std::cout << "Error reading from Lidar: " << e << std::endl;
        return;
    }  
        
    auto [current_state, adv, rot] = chocachoca(data.points);
    
    move_robot(adv, rot);
}
	

void SpecificWorker::move_robot(float adv, float rot)
{
    try
    { 
        omnirobot_proxy->setSpeedBase(0.0, adv, rot);
    } 
    catch(const Ice::Exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::chocachoca(const RoboCompLidar3D::TPoints &points)
{
    try
    {
        const auto nearest = [this, &points](float min_angle, float max_angle)
        {
            auto cone = cono(points, min_angle, max_angle, 0.f);
            const auto point = std::ranges::min_element(cone,
                [](const auto &left, const auto &right)
                {
                    return left.distance2d < right.distance2d;
                });
            return point == cone.end() ? 3000.f : point->distance2d;
        };

        const auto front_distance = nearest(-0.35f, 0.35f);

        switch(state)
        {
            case State::FORWARD:
            {
                if (front_distance < 750.f)
                {
                    wall_side = 1;
                    ++turn_index;
                    turn_timer.restart();
                    state = State::TURN;
                    const auto turn_speed = 0.8f;
                    return {state, 0.f, static_cast<float>(wall_side) * turn_speed};
                }
                return {state, 900.f, 0.f};
            }   
            case State::TURN:
            {
                const auto turn_speed = 0.8f;
                const auto turn_duration_ms = static_cast<long long>(1000.f * (M_PI / 2.f) / turn_speed);
                if (turn_timer.elapsed() >= turn_duration_ms and front_distance > 950.f)
                {
                    if (turn_index == 2)
                    {
                        follow_phase = 1;
                        segment_timer.restart();
                        state = State::FOLLOW_WALL;
                    }
                    else if (turn_index == 3)
                    {
                        follow_phase = 2;
                        segment_timer.restart();
                        state = State::FOLLOW_WALL;
                    }
                    else
                    {
                        state = State::FOLLOW_WALL;
                    }
                    return {state, 0.f, 0.f};
                }
                return {state, 0.f, static_cast<float>(wall_side) * turn_speed};
            }
            case State::FOLLOW_WALL:
            {
                if (follow_phase == 1)
                {
                    const auto descend_speed = 900.f;
                    const auto descend_distance = 2500.f;
                    const auto descend_duration_ms = static_cast<long long>(1000.f * descend_distance / descend_speed);
                    if (segment_timer.elapsed() >= descend_duration_ms)
                    {
                        follow_phase = 0;
                        ++turn_index;
                        turn_timer.restart();
                        state = State::TURN;
                        return {state, 0.f, static_cast<float>(wall_side) * 0.8f};
                    }
                    return {state, descend_speed, 0.f};
                }

                if (follow_phase == 2)
                {
                    const auto shift_speed = 900.f;
                    const auto shift_distance = 1250.f;
                    const auto shift_duration_ms = static_cast<long long>(1000.f * shift_distance / shift_speed);
                    if (segment_timer.elapsed() >= shift_duration_ms or front_distance < 750.f)
                    {
                        spiral_rot = 1.6f;
                        spiral_timer.restart();
                        state = State::SPIRAL;
                        return {state, 0.f, 0.f};
                    }
                    return {state, shift_speed, 0.f};
                }

                if (front_distance < 750.f)
                {
                    ++turn_index;
                    turn_timer.restart();
                    state = State::TURN;
                    return {state, 0.f, static_cast<float>(wall_side) * 0.8f};
                }
                return {state, 900.f, 0.f};
            }
            case State::SPIRAL:
            {
                const auto spiral_duration_ms = 16000LL;
                if (spiral_timer.elapsed() >= spiral_duration_ms)
                {
                    follow_phase = 0;
                    spiral_rot = 1.6f;
                    state = State::FOLLOW_WALL;
                    return {state, 0.f, 0.f};
                }

                if (front_distance < 750.f)
                {
                    follow_phase = 0;
                    turn_index = 4;
                    spiral_rot = 0.8f;
                    turn_timer.restart();
                    state = State::TURN;
                    return {state, 0.f, static_cast<float>(wall_side) * 0.8f};
                }

                const auto elapsed_seconds = static_cast<float>(spiral_timer.elapsed()) / 1000.f;
                spiral_rot = 1.6f - 0.08f * elapsed_seconds;
                if (spiral_rot < 0.35f)
                    spiral_rot = 0.35f;
                return {state, 900.f, spiral_rot};
            }
        return {state, 0.f, 0.f};
    }
	} catch(const Ice::Exception &e) {
        std::cerr << e.what() << '\n';
        
    }
    return {state, 0.f, 0.f};
}

void SpecificWorker::new_target_slot(QPointF p)
{
	qInfo() << p;
}

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
   static std::vector<QGraphicsItem*> draw_points;
   for (const auto &p : draw_points)
   {
      scene->removeItem(p);
      delete p;
   }
   draw_points.clear();
   
   auto cpoints = cono(points, -0.01,0.01,0);

   const QColor color("LightGreen");
   const QPen pen(color, 10);
   //const QBrush brush(color, Qt::SolidPattern);
   for (const auto &p : cpoints)
   {
      const auto dp = scene->addRect(-25, -25, 50, 50, pen);
      dp->setPos(p.x, p.y);
      draw_points.push_back(dp);   // add to the list of points to be deleted next time
   }
}


void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// RoboCompVisualElements::TObjects this->visualelements_proxy->getVisualObjects(TObjects objects)
// RoboCompVisualElements::void this->visualelements_proxy->setVisualObjects(TObjects objects)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

