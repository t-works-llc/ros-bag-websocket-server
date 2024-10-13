#!/usr/bin/env python3

import rospy

import os
import sys
import time
import subprocess

#################### Configuration ####################
bag_dir = '/root/bag'

# when target_bag is empty, all bag files in bag_dir will be played
target_bag = ''

# if is_loop is True, all bag files will be played in loop
is_loop = True

#################### Configuration ####################

class BagPlayer:
  def __init__(self, bag_dir: str, target_bag: str, is_loop: bool):
    self.bag_dir = bag_dir
    self.target_bag = target_bag
    self.is_loop = is_loop

    self.bag_files = []
    self.current_bag_idx = 0
    self.current_bag = None

    self.load_bag_files()

  def load_bag_files(self):
    if self.target_bag:
      self.bag_files.append(self.target_bag)
    else:
      self.bag_files = [f for f in os.listdir(self.bag_dir) if f.endswith('.bag')]
  
  def play(self):
    if not self.bag_files:
      rospy.logerr('No bag files found')
      return

    while True:
      self.current_bag = os.path.join(self.bag_dir, self.bag_files[self.current_bag_idx])
      rospy.loginfo(f'Playing {self.current_bag}')

      topics = subprocess.run(['rosbag', 'info', '--yaml', self.current_bag], stdout=subprocess.PIPE).stdout.decode('utf-8')
      topics = [t.split(':')[1].strip() for t in topics.split('\n') if 'topic: ' in t]
      
      topic_log = 'Topics in bag file:'
      for topic in topics:
        topic_log += f'\n - {topic}'
      rospy.loginfo(topic_log)

      try:
        subprocess.run(['rosbag', 'play', self.current_bag, '--clock'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      except subprocess.CalledProcessError as e:
        rospy.logerr(f'Error: {e}')
        break
      except KeyboardInterrupt:
        break

      self.current_bag = None
      self.current_bag_idx += 1
      if len(self.bag_files) <= self.current_bag_idx:
        if self.is_loop:
          self.current_bag_idx = 0
        else:
          break

if __name__ == '__main__':
  rospy.init_node('bagfile_player', anonymous=True)

  bag_player = BagPlayer(bag_dir, target_bag, is_loop)
  bag_player.play()
