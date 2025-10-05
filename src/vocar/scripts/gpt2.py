import openai
import os
import time

openai.api_key = "" 



system_prompt = """You are an intent classification assistant for a robot. Given a voice command, your job is to return the correct intent label. Only return the intent, nothing else. However, it is possible that the user input was not intended as a robot voice command. Thus if the voice command does not match any of the intents, output \"NONE\" instead.

Valid intents:
- go_to_door
- stop
- increase_speed
- decrease_speed
- move_backward
- turn_left
- turn_right
- go_to_parking
- move_forward
- go_to_kitchen
- go_to_restroom
- go_to_elavator
- find_person

Examples:

Command: \"uh go to the door now\"
Intent: go_to_door

Command: \"stop moving\"
Intent: stop

Command: \"speed up a bit\"
Intent: increase_speed

Command: \"take it easy you're going too fast\"
Intent: decrease_speed

Command: \"back up slowly\"
Intent: move_backward

Command: \"uh turn left here\"
Intent: turn_left

Command: \"swing right real quick\"
Intent: turn_right

Command: \"go chill in your parking spot\"
Intent: go_to_parking

Command: \"move ahead\"
Intent: move_forward

Please make indirect generalizations and inferences if possible. For example, \"let's go outside\" could indicate go_to_door.

"""

start = time.time()
response = openai.ChatCompletion.create(
    model="gpt-4",  
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": ""} 
    ],
    temperature=0.5,
    top_p=0.7,
    max_tokens=2000
)

intent_output = response["choices"][0]["message"]["content"].strip().lower()
print(intent_output)

print(f"Latency: {time.time() - start}")
start = time.time()
response = openai.ChatCompletion.create(
    model="gpt-4",  
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": ""} 
    ],
    temperature=0.5,
    top_p=0.7,
    max_tokens=2000
)

intent_output = response["choices"][0]["message"]["content"].strip().lower()
print(intent_output)
print(f"Latency: {time.time() - start}")
start = time.time()
response = openai.ChatCompletion.create(
    model="gpt-4",  
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": ""} 
    ],
    temperature=0.5,
    top_p=0.7,
    max_tokens=2000
)

intent_output = response["choices"][0]["message"]["content"].strip().lower()
print(intent_output)
print(f"Latency: {time.time() - start}")
