#include "battery_consumer.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "ROS_debugging.h"

#define BATTERY_CONSUMER_DEBUG

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() : consumerId(-1)
{
}

BatteryConsumerPlugin::~BatteryConsumerPlugin()
{
  if (this->battery && this->consumerId != -1)
    this->battery->RemoveConsumer(this->consumerId);
}

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != nullptr, "Model pointer is NULL");
  GZ_ASSERT(_sdf != nullptr, "SDF Element pointer is NULL");

#ifdef CONSUMER_DEBUG
  gzdbg << "started loading consumer \n";
#endif

  const std::string node_name = _sdf->Get<std::string>("ros_node");

  // check if the ros is up!
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, node_name, ros::init_options::NoSigintHandler);
  }

  this->model = _model;
  this->world = _model->GetWorld();

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->link = _model->GetLink(linkName);

  // Create battery
  std::string batteryName = _sdf->Get<std::string>("battery_name");
  this->battery = this->link->Battery(batteryName);

  // Add consumer and sets its power load
  this->powerLoad = _sdf->Get<double>("power_load");
  this->consumerId = this->battery->AddConsumer();
  this->battery->SetPowerLoad(this->consumerId, powerLoad);

  // Create ros node and publish stuff there!
  this->rosNode.reset(new ros::NodeHandle(node_name));

  this->set_power_load = this->rosNode->advertiseService(batteryName + "/set_power_load",
                                                         &BatteryConsumerPlugin::SetConsumerPowerLoad, this);

#ifdef CONSUMER_DEBUG
  gzdbg << "consumer loaded \n";
#endif

  ROS_GREEN_STREAM("Consumer loaded");
}

void BatteryConsumerPlugin::Init()
{
#ifdef CONSUMER_DEBUG
  gzdbg << "consumer is initialized \n";
#endif
  ROS_GREEN_STREAM("Consumer is initialized");
}

void BatteryConsumerPlugin::Reset()
{
#ifdef CONSUMER_DEBUG
  gzdbg << "consumer is reset \n";
#endif
  ROS_GREEN_STREAM("Consumer is reset");
}

bool BatteryConsumerPlugin::SetConsumerPowerLoad(brass_gazebo_battery::SetLoad::Request& req,
                                                 brass_gazebo_battery::SetLoad::Response& res)
{
  lock.lock();
  double load = this->powerLoad;
  this->powerLoad = req.power_load;
  this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

#ifdef BATTERY_CONSUMER_DEBUG
  gzdbg << "Power load of consumer has changed from:" << load << ", to:" << this->powerLoad << "\n";
#endif
  ROS_GREEN_STREAM("Power load of consumer has changed to: " << this->powerLoad);

  lock.unlock();
  res.result = true;
  return true;
}