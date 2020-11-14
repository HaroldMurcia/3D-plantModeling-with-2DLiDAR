#include "Laser.h"

namespace gazebo {

	Laser::Laser() {
		// TODO Auto-generated constructor stub

	}

	Laser::~Laser() {
		// TODO Auto-generated destructor stub
	}

	void Laser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
		this->nombreMundo = _parent->WorldName();
		//this->mundo = physics::get_world(this->nombreMundo);
		if(!ros::isInitialized()){
			int argc=0;
			char **argv=NULL;
			ros::init(argc,argv, "ray_laser", ros::init_options::NoSigintHandler);
		}
		using boost::dynamic_pointer_cast;
		this->sensorPtr=dynamic_pointer_cast<sensors::RaySensor>(_parent);
		this->topic="/scan";
		this->nodo.reset(new ros::NodeHandle("ray_laser"));

		/*ros::SubscribeOptions so=ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
				$this->topic+"_sub",
				1,
				boost::bind(&Laser::configurar, this, _1),
				ros::VoidPtr(),
				&this->cola
				);
		this->subscriber=this->nodo->subscribe(so);/**/

		ros::AdvertiseOptions ad=ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
						this->topic,
						1,
						boost::bind(&Laser::conexion, this),
						boost::bind(&Laser::desconexion, this),
						ros::VoidPtr(),
						&this->cola2
						);
		this->publisher=this->nodo->advertise(ad);
		this->nodoLaser_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
				this->nodoLaser_->Init(this->nombreMundo);
		//this->threadColas=std::thread(std::bind(&Laser::configurar, this));
		this->sensorPtr->SetActive(false);
		this->subcriptorLaser=this->nodoLaser_->Subscribe(this->sensorPtr->Topic(),&Laser::onScan, this);

	}

	void Laser::onScan(ConstLaserScanStampedPtr &_msg){

	  sensor_msgs::LaserScan laser_msg;
	  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
	  laser_msg.header.frame_id = "laser";
	  laser_msg.angle_min = _msg->scan().angle_min();
	  laser_msg.angle_max = _msg->scan().angle_max();
	  laser_msg.angle_increment = _msg->scan().angle_step();
	  laser_msg.time_increment = 0;
	  laser_msg.scan_time = 0;
	  laser_msg.range_min = _msg->scan().range_min();
	  laser_msg.range_max = _msg->scan().range_max();
	  laser_msg.ranges.resize(_msg->scan().ranges_size());
	  std::copy(_msg->scan().ranges().begin(),
	            _msg->scan().ranges().end(),
	            laser_msg.ranges.begin());
	  laser_msg.intensities.resize(_msg->scan().intensities_size());
	  std::copy(_msg->scan().intensities().begin(),
	            _msg->scan().intensities().end(),
	            laser_msg.intensities.begin());
	  this->publisher.publish(laser_msg);
	}
	/*void Laser::onScan(const sensor_msgs::LaserScanConstPtr& _msg) {
		sensor_msgs::LaserScan laser_msg;
		  laser_msg.header.stamp = ros::Time();
		  laser_msg.header.frame_id = "nombre";//this->frame_name_;
		  laser_msg.angle_min = _msg->angle_min;
		  laser_msg.angle_max = _msg->angle_max;
		  laser_msg.angle_increment = _msg->angle_increment;
		  laser_msg.time_increment = 0;
		  laser_msg.scan_time = 0;
		  laser_msg.range_min = _msg->range_min;
		  laser_msg.range_max = _msg->range_max;
		  laser_msg.ranges.resize(_msg->ranges.size());
		  std::copy(_msg->ranges.begin(),
		            _msg->ranges.end(),
		            laser_msg.ranges.begin());
		  laser_msg.intensities.resize(_msg->intensities.size());
		  std::copy(_msg->intensities.begin(),
		            _msg->intensities.end(),
		            laser_msg.intensities.begin());
		this->publisher.publish(laser_msg);
	}/**/


	void Laser::conexion() {
		this->conectados++;
		gzdbg()<<this->conectados;
		if(this->conectados==1){
			this->subcriptorLaser=this->nodoLaser_->Subscribe(this->sensorPtr->Topic(),&Laser::onScan, this);
		}
	}

	void Laser::desconexion() {
		this->conectados--;
		gzdbg()<<this->conectados;
		if (this->conectados == 0)
			this->subcriptorLaser.reset();
	}

	void Laser::configurar(sensor_msgs::LaserScan::ConstPtr& _msg) {
	}
	void Laser::thread() {

		/*//gzdbg()<<&this->sensorPtr;
		sensor_msgs::LaserScan laser_msg;
		//this->sensorPtr->FillMsg(laser_msg);
		laser_msg.header.stamp = ros::Time();
		laser_msg.header.frame_id = "";
		laser_msg.angle_min = this->sensorPtr->AngleMin();
		laser_msg.angle_max = this->sensorPtr->AngleMax();
		laser_msg.angle_increment = this->sensorPtr->GetAngleResolution();
		laser_msg.time_increment = 0;  // instantaneous simulator scan
		laser_msg.scan_time = 0;  // not sure whether this is correct
		laser_msg.range_min = this->sensorPtr->RangeMin();
		laser_msg.range_max = this->sensorPtr->RangeMax();
		laser_msg.ranges.resize(this->sensorPtr->RangeCount());
		/*this->sensorPtr->GetRanges(laser_msg.ranges.begin());
		laser_msg.intensities.resize(this->sensorPtr->get);
		std::copy(_msg->scan().intensities().begin(),
				_msg->scan().intensities().end(),
				laser_msg.intensities.begin());/**/
		//this->publisher.publish(laser_msg);
	}
GZ_REGISTER_SENSOR_PLUGIN(Laser);



} /* namespace gazebo */
