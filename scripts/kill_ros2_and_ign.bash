#!/bin/bash
kill -9 $(ps aux | grep "ign gazebo" | awk '{print $2}')
kill -9 $(ps aux | grep ros | awk '{print $2}')
