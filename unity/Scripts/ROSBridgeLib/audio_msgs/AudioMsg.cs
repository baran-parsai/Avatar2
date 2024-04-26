using SimpleJSON;
using UnityEngine;
using ROSBridgeLib.std_msgs;


/**
 * Define an Audio message
 * Version History
 *
 *  V1.1 - merger of audio input and audio output messages and inclusion of format
 *  V1.0 - hand crafted from the ros2 message
 * 
 */

namespace ROSBridgeLib {
	namespace avatar_msgs {
		public class AudioMsg : ROSBridgeMsg {
			private int _seq;
			private string _audio;
			private string _format;
			private HeaderMsg _header;

			public AudioMsg(JSONNode msg) {
				Debug.Log("Audio Input message being parsed");
				_header = new HeaderMsg(msg["header"]);
				_seq = msg["seq"].AsInt;
				_audio = msg["audio"];
				_format = msg["format"];
				Debug.Log("All done");
			}

			public static string GetMessageType()
			{
				return "avatar2_interfaces/AudioInput";
			}

			
			public AudioMsg(HeaderMsg header, int seq, string format, string audio) {
				_seq = seq;
				_header = header;
				_format = format;
				_audio = audio;
			}
			
			
			public HeaderMsg GetHeader() {
				return _header;
			}
			
			public int GetSeq() {
				return _seq;
			}

			public string GetAudio()
			{
				return _audio;
			}

			public string GetFormat()
            {
				return _format;
            }

			public override string ToString() {
				return "avatar2_interfaces/AudioInput[header=" + _header + ",  seq=" + _seq + ", aduio=" + _audio + ", format=" + _format + "]";
			}

			public override string ToYAMLString() {
				return "{\"header\" : " + _header + ", \"seq\" : " + _seq + ", \"format\" : " + _format +  ", \"audio\" : " + _audio + "}";
			}

		}
	}
}

