#ifndef MAVROSFLIGHT_PARAM_MANAGER_H
#define MAVROSFLIGHT_PARAM_MANAGER_H

#include <mavrosflight/mavlink_bridge.h>
#include <mavrosflight/mavrosflight.h>

#include <string>
#include <map>

namespace mavrosflight
{

class ParamManager
{
public:
  ParamManager(MavROSflight *mavrosflight);
  ~ParamManager();

  bool is_param_id(std::string name);
  bool unsaved_changes();

  void handle_param_value_msg(const mavlink_param_value_t &msg);
  bool set_param_value();
  void write_params();

private:

  struct Param
  {
    unsigned int index;
    MAV_PARAM_TYPE type;
    union
    {
      int i;
      unsigned int u;
      float f;
    } value;
  };

  std::map<std::string, Param> params_;
  bool unsaved_changes_;

  bool first_param_received_;
  size_t param_count_;
  bool *received_;
  bool initialized_;

  MavROSflight *mavrosflight_;
};

}

#endif // MAVROSFLIGHT_PARAM_MANAGER_H
