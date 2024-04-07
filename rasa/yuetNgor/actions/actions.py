# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

# from typing import Any, Text, Dict, List
#
# from rasa_sdk import Action, Tracker
# from rasa_sdk.executor import CollectingDispatcher
#
#
# class ActionHelloWorld(Action):
#
#     def name(self) -> Text:
#         return "action_hello_world"
#
#     def run(self, dispatcher: CollectingDispatcher,
#             tracker: Tracker,
#             domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
#
#         dispatcher.utter_message(text="Hello World!")
#
#         return []

from rasa_sdk import Action

class ActionUtterMessage(Action):
    def name(self):
        return "utter_123"

    async def run(self, dispatcher, tracker, domain):
        # Here you would add the code to determine the message based on the completed action
        # For this example, let's assume the message is "Action completed"
        message = "Action completed"
        dispatcher.utter_message(text=message)

class ActionWelcomeMessage(Action):
    def name(self):
        return "action_welcome_message"

    async def run(self, dispatcher, tracker, domain):
        message = "Welcome! How can I assist you today?"
        dispatcher.utter_message(text=message)