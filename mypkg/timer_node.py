# SPDX-FileCopyrightText: 2025 Shoma Takatori
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class PomodoroTimer(Node):

    def __init__(self):
        super().__init__('pomodoro_timer')
        self.work_duration = 25 * 60
        self.current_time = self.work_duration
        self.is_running = False
        self.publisher_ = self.create_publisher(String, 'mypkg/countdown', 10)
        self.srv = self.create_service(
            SetBool, 'mypkg/control', self.control_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('mypkg Pomodoro Timer Ready.')

    def control_callback(self, request, response):
        if request.data:
            if self.current_time <= 0:
                self.current_time = self.work_duration
            self.is_running = True
            response.success = True
            response.message = "Timer Started"
        else:
            self.is_running = False
            response.success = True
            response.message = "Timer Paused"
        return response

    def timer_callback(self):
        if self.is_running and self.current_time > 0:
            self.current_time -= 1
            mins, secs = divmod(self.current_time, 60)
            msg = String()
            msg.data = f"Remaining: {mins:02d}:{secs:02d}"
            self.publisher_.publish(msg)
            if self.current_time == 0:
                self.is_running = False
                self.get_logger().info("Time up!")


def main(args=None):
    rclpy.init(args=args)
    node = PomodoroTimer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
