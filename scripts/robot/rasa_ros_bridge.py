# rasa_ros_bridge.py

"""
rasa_ros_bridge.py

This script creates a `RasaBridge` class for communication between ROS (Robot Operating System) and Rasa, an AI chatbot framework. 

The `RasaBridge` class subscribes to ROS speech data, processes it through Rasa, and publishes the responses back to ROS. It also defines robot actions and uses `rospy.Publisher` to publish messages to ROS topics.

Future Enhancements:

1. **Customisable Actions**: Modification of the `ACTIONS` dictionary through an optional `actions` parameter in the `RasaBridge` constructor.
2. **Multiple Speech Subscribers**: Enhance `subscribe_to_speech` to allow subscribing to multiple voices simultaneously for multi-agent experiments.
"""

import rospy
from pyhri import HRIListener
from hri_msgs.msg import LiveSpeech
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from flask import Flask, request, jsonify
import requests

class RasaBridge:
    """Handles communication between ROS and Rasa."""

    ACTIONS = {
        "GO_TO": "action_go_to",
        "FOLLOW": "action_follow",
        "DO_NOT_FOLLOW": "action_do_not_follow",
        "OFFER_DRINK": "action_offer_drink",
    }

    def __init__(self):
        """Initialize the RasaBridge."""
        self.sender = "user"
        self.rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"
        self.speech_subscribers = {}
        self.publishers = {
            "speech": rospy.Publisher("/say", String, queue_size=1),
            "move_base": rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1),
            "action": rospy.Publisher("/action", String, queue_size=1),
            "destination": rospy.Publisher("/destination", String, queue_size=1),
            "twist": rospy.Publisher("/cmd_vel", Twist, queue_size=1),
        }

    def subscribe_to_speech(self, voice):
        """Subscribe to speech from a given voice."""
        self.speech_subscribers[voice.id] = rospy.Subscriber(
            voice.ns + "/speech", LiveSpeech, self.send_to_rasa
        )

    def send_to_rasa(self, msg):
        """Send a message to Rasa if it is final."""
        if not msg.final:
            return
        text = msg.final
        rospy.loginfo('Heard message: "%s"  -- sending it to Rasa...' % text)
        try:
            results = requests.post(
                self.rasa_endpoint, json={"sender": self.sender, "message": text}
            )
            results.raise_for_status()
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Failed to send message to Rasa: {e}")
            return
        results = results.json()
        for r in results:
            msg = String()
            msg.data = r["text"]
            self.publishers["speech"].publish(msg)

class RasaWebhook:
    """Handles the Flask app and Rasa webhook."""

    def __init__(self, rasa_bridge):
        """Initialize the RasaWebhook."""
        self.app = Flask(__name__)
        self.rasa_bridge = rasa_bridge
        self.app.add_url_rule("/webhook", "webhook", self.rasa_action, methods=["POST"])

    def rasa_action(self):
        """Handle a POST request from Rasa."""
        data = request.json
        action = data["next_action"]
        self.rasa_bridge.publishers["action"].publish(action)

        if action == self.rasa_bridge.ACTIONS["GO_TO"]:
            destination = data["tracker"]["slots"]["location"]
            self.rasa_bridge.publishers["destination"].publish(destination)
            rospy.logwarn("Got destination: %s" % destination)

        elif action == self.rasa_bridge.ACTIONS["FOLLOW"]:
            rospy.loginfo("Following")

        elif action == self.rasa_bridge.ACTIONS["DO_NOT_FOLLOW"]:
            rospy.loginfo("Not following")

        elif action == self.rasa_bridge.ACTIONS["OFFER_DRINK"]:
            drink = data["tracker"]["slots"]["drink"]
            self.rasa_bridge.publishers["destination"].publish("pantry")
            rospy.logwarn("Got drink: %s" % drink)
            self.rasa_bridge.publishers["action"].publish(self.rasa_bridge.ACTIONS["OFFER_DRINK"])

        else:
            rospy.loginfo("No action")

        return jsonify({"message": "Success"})

if __name__ == "__main__":
    rospy.init_node("rasa_bridge")
    hri = HRIListener()
    rasa_bridge = RasaBridge()
    hri.on_voice(rasa_bridge.subscribe_to_speech)
    rasa_webhook = RasaWebhook(rasa_bridge)
    rasa_webhook.app.run(host="0.0.0.0", port=5056, debug=True, use_reloader=False)