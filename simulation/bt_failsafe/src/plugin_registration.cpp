#include <behaviortree_cpp/bt_factory.h>
#include "bt_failsafe/stop_robot.hpp"

namespace bt_failsafe
{
void RegisterNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<StopRobotCondition>("StopRobotCondition");
}
}  // namespace bt_failsafe

extern "C"
{
    void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
    {
        bt_failsafe::RegisterNodes(factory);
    }
}

