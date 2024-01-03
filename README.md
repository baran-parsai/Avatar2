# Avatar2
The updated avatar

To run

Build somewhere with a microphone

ros2 run avatar2 sound_capture  --ros-args -p non_speaking_duration:=1.0 -p pause_threshold:=1.0

will capture audio utterances from the default microphone and publish them as message. The two parameters above
mean quiet air for 1 second is used to segment audio utterances. Change as you wish

ros2 run avatar2 sound_dump

will dump these as wav files which you can play using aplay. Good testing that your microphone is working and to have a colleciton of wav  files around to play with. There is a tool

ros2 run avatar2 sound_play 

which will take a bunch of pre-recorded wav files (actually more than this, but its a good place to start) and encode them into the system

Finally, 

ros2 run avatar2 sound_to_text

will take audio snippets and convert them into text

So to run everything (from a live microphone)

ros2 run avatar2 sound_capture  --ros-args -p non_speaking_duration:=1.0 -p pause_threshold:=1.0 &
ros2 run avatar2 sound_to_text &
ros2 topic echo /avatar2/message

will give you the messages as text

This all depends on two python libraries
* speech_recognition
* whisper

By default, whisper will consume cycles on your gpu. You can pass it the appropriate parameters to use your CPU instead.

Next items to code up

- a simple chatbot 
- a simple avatar
- to code up one or more of the sentiment modules from avatar v1 and encode them in. 
- to code up a speech to text module
- to code up an avatar in unity


