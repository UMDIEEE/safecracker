#!/bin/bash
raspivid -hf -vf -o - -t 0 -fps 30 -b 900000 | ffmpeg -re -ar 44100 -ac 2 -acodec pcm_s16le -f s16le -ac 2 -i /dev/zero -f h264 -i - -vcodec copy -acodec aac -ab 128k -g 20 -strict experimental -f flv rtmp://live-iad.twitch.tv/app/live_STREAM_KEY_HERE

