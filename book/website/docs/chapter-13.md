---
sidebar_position: 14
---

# Chapter 13: Voice-to-Action Systems

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Implement voice recognition using Whisper
- Parse voice commands for robotic actions
- Create speech-to-action pipelines
- Integrate voice systems with ROS 2
- Handle voice command ambiguity and errors

## 13.1 Introduction to Voice-to-Action Systems

Voice-to-action systems enable robots to understand spoken commands and execute appropriate actions. These systems make human-robot interaction more natural and intuitive, especially for non-technical users.

Key components:
- Speech recognition: Converting speech to text
- Natural language understanding: Interpreting the meaning
- Action mapping: Translating commands to robot actions
- Execution: Carrying out the requested actions

## 13.2 Whisper for Speech Recognition

OpenAI's Whisper is a state-of-the-art speech recognition model that can be used for robotic applications:

Features of Whisper:
- Multilingual support
- Robust to background noise
- Different model sizes for various applications
- Open-source implementation

Example Whisper usage:
```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("command.wav")
print(result["text"])
```

## 13.3 Voice Command Processing Pipeline

A typical voice-to-action pipeline includes:
1. Audio capture from microphones
2. Speech recognition using Whisper
3. Natural language processing
4. Command parsing and validation
5. Action mapping
6. Robot execution

## 13.4 Implementing Voice Recognition Node

Example ROS 2 node for voice recognition:

```python
import rclpy
from rclpy.node import Node
import whisper
import speech_recognition as sr
from std_msgs.msg import String

class VoiceRecognitionNode(Node):
    def __init__(self):
        super().__init__('voice_recognition_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        
        # Load Whisper model
        self.whisper_model = whisper.load_model("base")
        
        # Set up speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Start voice capture
        self.timer = self.create_timer(1.0, self.capture_voice)
    
    def capture_voice(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            try:
                audio = self.recognizer.listen(source, timeout=5.0)
                
                # Save audio to temporary file for Whisper
                audio_file = "temp_audio.wav"
                with open(audio_file, "wb") as f:
                    f.write(audio.get_wav_data())
                
                # Transcribe using Whisper
                result = self.whisper_model.transcribe(audio_file)
                command_text = result["text"]
                
                # Publish the recognized command
                msg = String()
                msg.data = command_text
                self.publisher_.publish(msg)
                self.get_logger().info(f'Recognized: {command_text}')
                
            except sr.WaitTimeoutError:
                self.get_logger().info('No speech detected')
            except Exception as e:
                self.get_logger().error(f'Error in voice recognition: {e}')
```

## 13.5 Command Parsing and Validation

Voice commands need to be parsed and validated:
- Intent recognition: Understanding the purpose of the command
- Entity extraction: Identifying objects, locations, etc.
- Validation: Ensuring commands are safe and executable
- Error handling: Managing unrecognized or ambiguous commands

## 13.6 Handling Ambiguity and Errors

Common challenges in voice-to-action systems:
- Background noise affecting recognition
- Accents and speech variations
- Ambiguous commands
- Homophones and similar-sounding words

Strategies to handle these:
- Confidence scoring for recognition results
- Context-aware disambiguation
- Confirmation requests for critical commands
- Error recovery mechanisms

## 13.7 Integration with Robot Systems

Voice systems integrate with robot systems through:
- ROS 2 topics for command communication
- Action servers for long-running tasks
- Parameter servers for configuration
- TF for spatial understanding

## 13.8 Chapter Summary

In this chapter, we explored voice-to-action systems using Whisper for speech recognition. We learned how to implement voice recognition nodes in ROS 2 and handle the challenges of processing spoken commands for robotic systems.

In the next chapter, we'll implement LLM-based task planning for robots.

## Exercises

1. Implement a Whisper-based voice recognition node
2. Create a command parser for common robot actions
3. Design error handling for ambiguous voice commands

---