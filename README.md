# Avatar2
The updated avatar. This is a generic README.md that describes the in-flux state of the avatar as of June 2024.

The avatar uses ROS2 for message passing. The passing system is set up so that all messages are sent/received. So far this has not been an issue

Launch files
- avatar_microphone.launch.py This launches an audio listener. It must be run on a machine connected to the microphone that the avatar uses.
- avatar_audio.launch.py This deals with audio to text and text to audio
- avatar_recognizer_video.launch.py This captures the avatar camera data and publishes information about the person who is talking to the avatar
- ros_avatar.lauch.py This runs a local (in ros) user interface for the avatar.
- avatar_llm_*.py This runs an avatar

We have a number of scenarios for the avatar. The details for a given scenario should be in the scenario folder. Within a given scenario there should be a faces folder which includes the pkl and json representation of the known people of the scenario.
- hearing_clinic The hearing clinic
- vgr_lab An avatar for the lab
- regimental_museum An avatar for the museum

The debris folder is a place for things that were important once, but are no longer part of the main branch of the avatar.



