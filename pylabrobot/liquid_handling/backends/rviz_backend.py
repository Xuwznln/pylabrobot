      
import json
from typing import List, Optional, Union

from pylabrobot.liquid_handling.backends.backend import (
  LiquidHandlerBackend,
)
from pylabrobot.liquid_handling.standard import (
  Drop,
  DropTipRack,
  MultiHeadAspirationContainer,
  MultiHeadAspirationPlate,
  MultiHeadDispenseContainer,
  MultiHeadDispensePlate,
  Pickup,
  PickupTipRack,
  ResourceDrop,
  ResourceMove,
  ResourcePickup,
  SingleChannelAspiration,
  SingleChannelDispense,
)
from pylabrobot.resources import Resource, Tip

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
from rclpy.action import ActionClient
from unilabos_msgs.action import SendCmd
import re

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每秒发布一次
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # 初始化关节位置

        self.resource_action = None
        
        while self.resource_action is None:
            self.resource_action = self.check_tf_update_actions()
            time.sleep(1)
        
        self.resource_action_client = ActionClient(self, SendCmd, self.resource_action)
        while not self.resource_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待 TfUpdate 服务器...')
        # 获取所有topic并检查action status topics
        # print("-"*20)
        # print(self.resource_action)
        # print("-"*20)
        
        
    def check_tf_update_actions(self):
        topics = self.get_topic_names_and_types()

        
        for topic_item in topics:

            topic_name, topic_types = topic_item

            if 'action_msgs/msg/GoalStatusArray' in topic_types:
                # 删除 /_action/status 部分

                base_name = topic_name.replace('/_action/status', '')
                # 检查最后一个部分是否为 tf_update
                parts = base_name.split('/')
                if parts and parts[-1] == 'tf_update':
                    return base_name
                
        return None
    
    def send_resource_action(self, resource_id_list:list[str], link_name:str):
        goal_msg = SendCmd.Goal()
        str_dict = {}
        for resource in resource_id_list:
            str_dict[resource] = link_name

        goal_msg.command = json.dumps(str_dict)
        self.resource_action_client.send_goal_async(goal_msg)
    
    def resource_move(self, resource_id:str, link_name:str, channels:list[int]):
        resource = resource_id.rsplit("_",1)
        
        channel_list = ['A','B','C','D','E','F','G','H']

        resource_list = []
        match = re.match(r'([a-zA-Z_]+)(\d+)', resource[1])
        if match:
            number = match.group(2)
            for channel in channels:
                resource_list.append(f"{resource[0]}_{channel_list[channel]}{number}")

        if len(resource_list) > 0:
            self.send_resource_action(resource_list, link_name)

    def timer_callback(self):
        self.publish_joint_states(self.joint_positions)
    
    def publish_joint_states(self, joint_positions):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['PLR_STATION_deck_first_joint', 'PLR_STATION_deck_second_joint', 'PLR_STATION_deck_third_joint', 'PLR_STATION_deck_fourth_joint']
        joint_state.position = joint_positions
        joint_state.velocity = []
        joint_state.effort = []
        self.publisher.publish(joint_state)
        # print(self.joint_positions)


    def inverse_kinematics(self,x, y, z):


        first_joint = -y/1000 + 0.163
        second_joint = -x/1000 + 0.1775
        third_joint = z/1000


        # 第四个关节没有用
        fourth_joint = 0.0

        return [first_joint, second_joint, third_joint, fourth_joint]

    def move_to_xyz(self,x, y, z, speed=0.1, rate=10):
        
        # rclpy.init()
        # joint_state_publisher = JointStatePublisher()


        try:
            # 计算逆运动学
            joint_positions_target = self.inverse_kinematics(x, y, z) 
            loop_flag = 0


            while loop_flag < 4:
                loop_flag = 0
                for i in range(4):
                    distance = joint_positions_target[i] - self.joint_positions[i]
                    if distance == 0:
                        loop_flag += 1
                        continue
                    minus_flag = distance/abs(distance)
                    if abs(distance) > speed/rate:
                        self.joint_positions[i] += minus_flag * speed/rate
                    else :
                        self.joint_positions[i] = joint_positions_target[i]
                        loop_flag += 1
                        

                # 发布关节状态
                self.publish_joint_states(self.joint_positions)
                time.sleep(1/rate)


        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt caught. Exiting...')
        # finally:
        #     joint_state_publisher.destroy_node()
        #     rclpy.shutdown()


    def goback(self,speed=0.1, rate=10):
        # rclpy.init()
        # joint_state_publisher = JointStatePublisher()

        try:
            # 计算逆运动学
            joint_positions_target = self.inverse_kinematics(0, 0, 0) 
            loop_flag = 0


            while loop_flag < 4:
                loop_flag = 0
                for i in range(4):
                    distance = joint_positions_target[i] - self.joint_positions[i]
                    if distance == 0:
                        loop_flag += 1
                        continue
                    minus_flag = distance/abs(distance)
                    if abs(distance) > speed/rate:
                        self.joint_positions[i] += minus_flag * speed/rate
                    else :
                        self.joint_positions[i] = joint_positions_target[i]
                        loop_flag += 1

                        
                # 发布关节状态
                self.publish_joint_states(self.joint_positions)
                time.sleep(1/rate)  


        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt caught. Exiting...')
        # finally:
        #     joint_state_publisher.destroy_node()
        #     rclpy.shutdown()

class LiquidHandlerRvizBackend(LiquidHandlerBackend):
  """Chatter box backend for device-free testing. Prints out all operations."""

  _pip_length = 5
  _vol_length = 8
  _resource_length = 20
  _offset_length = 16
  _flow_rate_length = 10
  _blowout_length = 10
  _lld_z_length = 10
  _kwargs_length = 15
  _tip_type_length = 12
  _max_volume_length = 16
  _fitting_depth_length = 20
  _tip_length_length = 16
  # _pickup_method_length = 20
  _filter_length = 10

  def __init__(self, num_channels: int = 8):
    """Initialize a chatter box backend."""
    super().__init__()
    self._num_channels = num_channels
# rclpy.init()
    if not rclpy.ok():
        rclpy.init()
    self.joint_state_publisher = None

  async def setup(self):
    self.joint_state_publisher = JointStatePublisher()
    await super().setup()

    print("Setting up the liquid handler.")

  async def stop(self):
    print("Stopping the liquid handler.")

  def serialize(self) -> dict:
    return {**super().serialize(), "num_channels": self.num_channels}

  @property
  def num_channels(self) -> int:
    return self._num_channels

  async def assigned_resource_callback(self, resource: Resource):
    print(f"Resource {resource.name} was assigned to the liquid handler.")

  async def unassigned_resource_callback(self, name: str):
    print(f"Resource {name} was unassigned from the liquid handler.")

  async def pick_up_tips(self, ops: List[Pickup], use_channels: List[int], **backend_kwargs):
    print("Picking up tips:")
    # print(ops.tip)
    header = (
      f"{'pip#':<{LiquidHandlerRvizBackend._pip_length}} "
      f"{'resource':<{LiquidHandlerRvizBackend._resource_length}} "
      f"{'offset':<{LiquidHandlerRvizBackend._offset_length}} "
      f"{'tip type':<{LiquidHandlerRvizBackend._tip_type_length}} "
      f"{'max volume (µL)':<{LiquidHandlerRvizBackend._max_volume_length}} "
      f"{'fitting depth (mm)':<{LiquidHandlerRvizBackend._fitting_depth_length}} "
      f"{'tip length (mm)':<{LiquidHandlerRvizBackend._tip_length_length}} "
      # f"{'pickup method':<{ChatterboxBackend._pickup_method_length}} "
      f"{'filter':<{LiquidHandlerRvizBackend._filter_length}}"
    )
    # print(header)

    for op, channel in zip(ops, use_channels):
      offset = f"{round(op.offset.x, 1)},{round(op.offset.y, 1)},{round(op.offset.z, 1)}"
      row = (
        f"  p{channel}: "
        f"{op.resource.name[-30:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{op.tip.__class__.__name__:<{LiquidHandlerRvizBackend._tip_type_length}} "
        f"{op.tip.maximal_volume:<{LiquidHandlerRvizBackend._max_volume_length}} "
        f"{op.tip.fitting_depth:<{LiquidHandlerRvizBackend._fitting_depth_length}} "
        f"{op.tip.total_tip_length:<{LiquidHandlerRvizBackend._tip_length_length}} "
        # f"{str(op.tip.pickup_method)[-20:]:<{ChatterboxBackend._pickup_method_length}} "
        f"{'Yes' if op.tip.has_filter else 'No':<{LiquidHandlerRvizBackend._filter_length}}"
      )
      # print(row)
      # print(op.resource.get_absolute_location())
      coordinate = op.resource.get_absolute_location()
      x = coordinate.x
      y = coordinate.y
      z = coordinate.z + 70
      # print(x, y, z)
      # print("moving")
      self.joint_state_publisher.move_to_xyz(x, y, 0)
      self.joint_state_publisher.move_to_xyz(x, y, z)
      self.joint_state_publisher.resource_move(op.resource.name, "PLR_STATION_deck_third_link", [0,1,2,3,4,5,6,7])
      self.joint_state_publisher.move_to_xyz(x, y, 0)
    #   goback()




  async def drop_tips(self, ops: List[Drop], use_channels: List[int], **backend_kwargs):
    print("Dropping tips:")
    header = (
      f"{'pip#':<{LiquidHandlerRvizBackend._pip_length}} "
      f"{'resource':<{LiquidHandlerRvizBackend._resource_length}} "
      f"{'offset':<{LiquidHandlerRvizBackend._offset_length}} "
      f"{'tip type':<{LiquidHandlerRvizBackend._tip_type_length}} "
      f"{'max volume (µL)':<{LiquidHandlerRvizBackend._max_volume_length}} "
      f"{'fitting depth (mm)':<{LiquidHandlerRvizBackend._fitting_depth_length}} "
      f"{'tip length (mm)':<{LiquidHandlerRvizBackend._tip_length_length}} "
      # f"{'pickup method':<{ChatterboxBackend._pickup_method_length}} "
      f"{'filter':<{LiquidHandlerRvizBackend._filter_length}}"
    )
    # print(header)

    for op, channel in zip(ops, use_channels):
      offset = f"{round(op.offset.x, 1)},{round(op.offset.y, 1)},{round(op.offset.z, 1)}"
      row = (
        f"  p{channel}: "
        f"{op.resource.name[-30:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{op.tip.__class__.__name__:<{LiquidHandlerRvizBackend._tip_type_length}} "
        f"{op.tip.maximal_volume:<{LiquidHandlerRvizBackend._max_volume_length}} "
        f"{op.tip.fitting_depth:<{LiquidHandlerRvizBackend._fitting_depth_length}} "
        f"{op.tip.total_tip_length:<{LiquidHandlerRvizBackend._tip_length_length}} "
        # f"{str(op.tip.pickup_method)[-20:]:<{ChatterboxBackend._pickup_method_length}} "
        f"{'Yes' if op.tip.has_filter else 'No':<{LiquidHandlerRvizBackend._filter_length}}"
      )
      # print(row)

      coordinate = op.resource.get_absolute_location()
      x = coordinate.x
      y = coordinate.y
      z = coordinate.z + 70
      # print(x, y, z)
      # print("moving")
      self.joint_state_publisher.move_to_xyz(x, y, 0)
      self.joint_state_publisher.move_to_xyz(x, y, z)
      self.joint_state_publisher.resource_move(op.resource.name, "world", use_channels)
      self.joint_state_publisher.move_to_xyz(x, y, 0)
    #   goback()

  async def aspirate(
    self,
    ops: List[SingleChannelAspiration],
    use_channels: List[int],
    **backend_kwargs,
  ):
    print("Aspirating:")
    header = (
      f"{'pip#':<{LiquidHandlerRvizBackend._pip_length}} "
      f"{'vol(ul)':<{LiquidHandlerRvizBackend._vol_length}} "
      f"{'resource':<{LiquidHandlerRvizBackend._resource_length}} "
      f"{'offset':<{LiquidHandlerRvizBackend._offset_length}} "
      f"{'flow rate':<{LiquidHandlerRvizBackend._flow_rate_length}} "
      f"{'blowout':<{LiquidHandlerRvizBackend._blowout_length}} "
      f"{'lld_z':<{LiquidHandlerRvizBackend._lld_z_length}}  "
      # f"{'liquids':<20}" # TODO: add liquids
    )
    for key in backend_kwargs:
      header += f"{key:<{LiquidHandlerRvizBackend._kwargs_length}} "[-16:]
    # print(header)

    for o, p in zip(ops, use_channels):
      offset = f"{round(o.offset.x, 1)},{round(o.offset.y, 1)},{round(o.offset.z, 1)}"
      row = (
        f"  p{p}: "
        f"{o.volume:<{LiquidHandlerRvizBackend._vol_length}} "
        f"{o.resource.name[-20:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{str(o.flow_rate):<{LiquidHandlerRvizBackend._flow_rate_length}} "
        f"{str(o.blow_out_air_volume):<{LiquidHandlerRvizBackend._blowout_length}} "
        f"{str(o.liquid_height):<{LiquidHandlerRvizBackend._lld_z_length}} "
        # f"{o.liquids if o.liquids is not None else 'none'}"
      )
      for key, value in backend_kwargs.items():
        if isinstance(value, list) and all(isinstance(v, bool) for v in value):
          value = "".join("T" if v else "F" for v in value)
        if isinstance(value, list):
          value = "".join(map(str, value))
        row += f" {value:<15}"
      # print(row)
      coordinate = o.resource.get_absolute_location()
      x = coordinate.x
      y = coordinate.y
      z = coordinate.z + 70 
      # print(x, y, z)
      # print("moving")
      self.joint_state_publisher.move_to_xyz(x, y, 0)
      self.joint_state_publisher.move_to_xyz(x, y, z)
      self.joint_state_publisher.move_to_xyz(x, y, 0)

  async def dispense(
    self,
    ops: List[SingleChannelDispense],
    use_channels: List[int],
    **backend_kwargs,
  ):
    # print("Dispensing:")
    header = (
      f"{'pip#':<{LiquidHandlerRvizBackend._pip_length}} "
      f"{'vol(ul)':<{LiquidHandlerRvizBackend._vol_length}} "
      f"{'resource':<{LiquidHandlerRvizBackend._resource_length}} "
      f"{'offset':<{LiquidHandlerRvizBackend._offset_length}} "
      f"{'flow rate':<{LiquidHandlerRvizBackend._flow_rate_length}} "
      f"{'blowout':<{LiquidHandlerRvizBackend._blowout_length}} "
      f"{'lld_z':<{LiquidHandlerRvizBackend._lld_z_length}}  "
      # f"{'liquids':<20}" # TODO: add liquids
    )
    for key in backend_kwargs:
      header += f"{key:<{LiquidHandlerRvizBackend._kwargs_length}} "[-16:]
    # print(header)

    for o, p in zip(ops, use_channels):
      offset = f"{round(o.offset.x, 1)},{round(o.offset.y, 1)},{round(o.offset.z, 1)}"
      row = (
        f"  p{p}: "
        f"{o.volume:<{LiquidHandlerRvizBackend._vol_length}} "
        f"{o.resource.name[-20:]:<{LiquidHandlerRvizBackend._resource_length}} "
        f"{offset:<{LiquidHandlerRvizBackend._offset_length}} "
        f"{str(o.flow_rate):<{LiquidHandlerRvizBackend._flow_rate_length}} "
        f"{str(o.blow_out_air_volume):<{LiquidHandlerRvizBackend._blowout_length}} "
        f"{str(o.liquid_height):<{LiquidHandlerRvizBackend._lld_z_length}} "
        # f"{o.liquids if o.liquids is not None else 'none'}"
      )
      for key, value in backend_kwargs.items():
        if isinstance(value, list) and all(isinstance(v, bool) for v in value):
          value = "".join("T" if v else "F" for v in value)
        if isinstance(value, list):
          value = "".join(map(str, value))
        row += f" {value:<{LiquidHandlerRvizBackend._kwargs_length}}"
      # print(row)
      coordinate = o.resource.get_absolute_location()
      x = coordinate.x
      y = coordinate.y
      z = coordinate.z + 70
      # print(x, y, z)
      # print("moving")
      self.joint_state_publisher.move_to_xyz(x, y, 0)
      self.joint_state_publisher.move_to_xyz(x, y, z)
      self.joint_state_publisher.move_to_xyz(x, y, 0)   

  async def pick_up_tips96(self, pickup: PickupTipRack, **backend_kwargs):
    print(f"Picking up tips from {pickup.resource.name}.")

  async def drop_tips96(self, drop: DropTipRack, **backend_kwargs):
    print(f"Dropping tips to {drop.resource.name}.")

  async def aspirate96(
    self, aspiration: Union[MultiHeadAspirationPlate, MultiHeadAspirationContainer]
  ):
    if isinstance(aspiration, MultiHeadAspirationPlate):
      resource = aspiration.wells[0].parent
    else:
      resource = aspiration.container
    print(f"Aspirating {aspiration.volume} from {resource}.")

  async def dispense96(self, dispense: Union[MultiHeadDispensePlate, MultiHeadDispenseContainer]):
    if isinstance(dispense, MultiHeadDispensePlate):
      resource = dispense.wells[0].parent
    else:
      resource = dispense.container
    print(f"Dispensing {dispense.volume} to {resource}.")

  async def pick_up_resource(self, pickup: ResourcePickup):
    print(f"Picking up resource: {pickup}")

  async def move_picked_up_resource(self, move: ResourceMove):
    print(f"Moving picked up resource: {move}")

  async def drop_resource(self, drop: ResourceDrop):
    print(f"Dropping resource: {drop}")

  def can_pick_up_tip(self, channel_idx: int, tip: Tip) -> bool:
    return True
    