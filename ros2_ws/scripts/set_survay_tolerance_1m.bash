#! /bin/bash

ros2 action send_goal --feedback base action_base/action/Base "{action_id: [2, 1000]}"

