#!/bin/bash
tmux -L catmux kill-session
pkill gzclient
pkill gzserver