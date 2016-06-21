#include "mavrosflight/param_manager.h"

#include <stdint.h>

namespace mavrosflight
{

ParamManager::ParamManager(MavROSflight *mavrosflight) :
  unsaved_changes_(false),
  first_param_received_(false),
  param_count_(0),
  initialized_(false),
  mavrosflight_(mavrosflight)
{
}

ParamManager::~ParamManager()
{
  if (param_count_ > 0)
  {
    delete[] received_;
  }
}

bool ParamManager::is_param_id(std::string name)
{
  return (params_.find(name) != params_.end());
}

bool ParamManager::unsaved_changes()
{
  return unsaved_changes_;
}

void ParamManager::handle_param_value_msg(const mavlink_param_value_t &msg)
{
  if (!first_param_received_)
  {
    first_param_received_ = true;
    received_ = new bool[msg.param_count];
  }

  Param param;
  param.index = msg.param_index;
  param.type = (MAV_PARAM_TYPE) msg.param_type;

  switch (param.type)
  {
  case MAV_PARAM_TYPE_INT8:
    param.value = (int) (*(int8_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_INT16:
    param.value = (int) (*(int16_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_INT32:
    param.value = (int) (*(int32_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_UINT8:
    param.value = (unsigned int) (*(uint8_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_UINT16:
    param.value = (unsigned int) (*(uint16_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_UINT32:
    param.value = (unsigned int) (*(uint32_t*) &msg.param_value);
    break;
  case MAV_PARAM_TYPE_REAL32:
    param.value = msg.param_value;
    break;
  }

  std::string name(msg.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  params_[name] = param;
}

bool ParamManager::set_param_value(std::string name)
{
  if (!is_param_id(name))
    return false;

  Param *param = &params_[name];

  bool value_changed;
  float value_float;
  switch (param->type)
  {
  case MAV_PARAM_TYPE_INT8:
  case MAV_PARAM_TYPE_INT16:
  case MAV_PARAM_TYPE_INT32:
    value_changed = (param->value.i == meh);
    value_float = (float) (*(int) &param->value.i);
    break;
  case MAV_PARAM_TYPE_UINT8:
  case MAV_PARAM_TYPE_UINT16:
  case MAV_PARAM_TYPE_UINT32:
    value_changed = (param->value.u == meh);
    value_float = (float) (*(unsigned int) &param->value.u);
    break;
  case MAV_PARAM_TYPE_REAL32:
    value_changed = (param->value.f == meh);
    break;
  default:
    value_changed = false;
    break;
  }

  if (value_changed)
  {
    mavrosflight_->send_param_set(1, 1, name, meh);
    unsaved_changes_ = true;
  }
}

void ParamManager::write_params()
{
}

}
