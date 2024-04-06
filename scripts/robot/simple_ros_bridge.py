# simple_ros_bridge.py

import rospy
from pyhri import HRIListener
from hri_msgs.msg import LiveSpeech
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion

import requests
from flask import Flask, request, jsonify
import psutil
import os

import json
from ..rasa.run_rasa import run_rasa

app = Flask(__name__)

sender = "user"

rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"

speech_subscribers = {}

speech_pub = rospy.Publisher("/say", String, queue_size=1)

move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

action_pub = rospy.Publisher("/action", String, queue_size=1)

destination_pub = rospy.Publisher("/destination", String, queue_size=1)

twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

def subscribe_to_speech(voice):
    speech_subscribers[voice.id] = rospy.Subscriber(
        voice.ns + "/speech", LiveSpeech, send_to_rasa
    )

def send_to_rasa(msg):

    if not msg.final:
        return
    # rospy.loginfo('Heard message: "%s"  -- sending it to Rasa...' % msg)
    text = msg.final
    # text = "Action completed: A Nice cuppa for you! Enjoy!"
    rospy.loginfo('Heard message: "%s"  -- sending it to Rasa...' % text)

    results = requests.post(
        rasa_endpoint, json={"sender": sender, "message": text}
    ).json()

    print(results)

    for r in results:
        msg = String()
        msg.data = r["text"]
        speech_pub.publish(msg)

@app.route("/webhook", methods=["POST"])
def rasa_action():
    data = request.json

    print("data: ", data)

    action = data["next_action"]
    print("action: ", action)
    action_pub.publish(action)

    move_base_msgs = PoseStamped()

    if action == "action_go_to":
        # print("data: ", data)

        destination = data["tracker"]["slots"]["location"]
        print("destination:", destination)
        destination_pub.publish(destination)

        rospy.logwarn("Got destination: %s" % destination)
        res = {"message": "Success"}

        # res = {"events": [], "responses": [{"text": "Right, off to %s I go then." % destination}]}
    elif action == "action_follow":
        print("following")
        action_pub.publish("action_follow")
        res = {"message": "Success"}
        # res = {"events": [], "responses": [{"text": "Sure, I can toddle along behind you."}]}
    elif action == "action_do_not_follow":
        print("not following")
        action_pub.publish("action_do_not_follow")
        res = {"message": "Success"}
        # res = {"events": [], "responses": [{"text": "Alright, I'll stay put then."}]}
    elif action == "action_offer_drink":
        print("offering drink")
        drink = data["tracker"]["slots"]["drink"]

        destination_pub.publish("pantry")
        rospy.logwarn("Got drink: %s" % drink)

        # res = {"events": [], "responses": [{"text": "Sure. Right away."}]}
        # res = {"events": [], "responses": [{"text": "Sure I will give you a %s" % drink}]}
        res = {"message": "Success"}

        action_pub.publish("action_offer_drink")
    else:
        print("no action")
        res = {"message": "Success"}
        # res = {"events": [], "responses": [{"text": "I am still learning. Could you please clarify?"}]}

    return jsonify(res)

if __name__ == "__main__":

    rospy.init_node("rasa_bridge")
    hri = HRIListener()
    hri.on_voice(subscribe_to_speech)
    run_rasa()

    # use same port as default RASA action server
    app.run(host="0.0.0.0", port=5056, debug=True, use_reloader=False)